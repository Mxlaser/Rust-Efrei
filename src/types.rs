//! Structures et énumérations partagées par tout le projet.
//!
//! Ce module constitue la **fondation** de la simulation : il définit les
//! types de données manipulés par tous les autres modules (carte, robots,
//! communication, UI). Il ne contient *aucune* logique métier — uniquement
//! les types et quelques utilitaires de base — afin que chaque membre du
//! groupe puisse travailler sur ces types sans dépendre du reste du code.
//!
//! Auteur : Personne 1 (Carte & structures de données).

use std::collections::HashMap;

/// Position d'une case sur la grille de la carte.
///
/// L'origine `(0, 0)` est le coin supérieur gauche. `x` augmente vers la
/// droite, `y` augmente vers le bas (convention écran/terminal).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Position {
    pub x: usize,
    pub y: usize,
}

impl Position {
    /// Crée une nouvelle position.
    pub fn new(x: usize, y: usize) -> Self {
        Position { x, y }
    }

    /// Distance de Manhattan entre deux positions.
    ///
    /// C'est l'heuristique idéale pour un déplacement en 4 directions
    /// (haut/bas/gauche/droite). Elle sera réutilisée par le pathfinding
    /// des robots (Personne 2).
    pub fn manhattan_distance(&self, other: &Position) -> usize {
        let dx = self.x.abs_diff(other.x);
        let dy = self.y.abs_diff(other.y);
        dx + dy
    }

    /// Renvoie les positions voisines orthogonales (4-connexité) qui restent
    /// à l'intérieur d'une grille de taille `width` x `height`.
    ///
    /// Les bords sont gérés proprement : aucune position hors limites n'est
    /// renvoyée (pas de soustraction sur un `usize` à 0).
    pub fn neighbors(&self, width: usize, height: usize) -> Vec<Position> {
        let mut result = Vec::with_capacity(4);
        // Gauche
        if self.x > 0 {
            result.push(Position::new(self.x - 1, self.y));
        }
        // Droite
        if self.x + 1 < width {
            result.push(Position::new(self.x + 1, self.y));
        }
        // Haut
        if self.y > 0 {
            result.push(Position::new(self.x, self.y - 1));
        }
        // Bas
        if self.y + 1 < height {
            result.push(Position::new(self.x, self.y + 1));
        }
        result
    }
}

/// Type de ressource présent sur la carte.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ResourceKind {
    /// Source d'énergie — affichée `E` en vert.
    Energy,
    /// Gisement de cristaux — affiché `C` en magenta clair.
    Crystal,
}

impl ResourceKind {
    /// Caractère utilisé pour l'affichage dans le terminal.
    pub fn symbol(&self) -> char {
        match self {
            ResourceKind::Energy => 'E',
            ResourceKind::Crystal => 'C',
        }
    }
}

/// Un gisement de ressource posé sur une case.
///
/// La `quantity` est mutable : les robots collecteurs (Personne 2/3) la
/// décrémentent une unité à la fois. Quand elle atteint 0, le gisement est
/// considéré comme épuisé (voir [`Resource::is_depleted`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Resource {
    pub kind: ResourceKind,
    pub quantity: u32,
}

impl Resource {
    /// Crée un gisement avec une quantité donnée.
    pub fn new(kind: ResourceKind, quantity: u32) -> Self {
        Resource { kind, quantity }
    }

    /// Vrai si le gisement est vide (plus rien à collecter).
    pub fn is_depleted(&self) -> bool {
        self.quantity == 0
    }

    /// Retire une unité de ressource si possible.
    ///
    /// Renvoie `true` si une unité a effectivement été retirée, `false` si
    /// le gisement était déjà épuisé. Utilisé par les collecteurs.
    pub fn take_one(&mut self) -> bool {
        if self.quantity > 0 {
            self.quantity -= 1;
            true
        } else {
            false
        }
    }
}

/// Nature « statique » d'une case de la carte.
///
/// Les ressources ne sont **pas** stockées dans la grille de tuiles mais
/// dans [`World::resources`] : elles peuvent en effet apparaître/disparaître
/// (épuisement) et possèdent une quantité, contrairement au terrain qui ne
/// change pas. Une case portant une ressource a la tuile [`Tile::Empty`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tile {
    /// Case libre, traversable.
    Empty,
    /// Obstacle infranchissable — affiché `O` en cyan clair.
    Obstacle,
    /// Base centrale — affichée `#` en vert clair.
    Base,
}

impl Tile {
    /// Vrai si un robot peut se déplacer sur cette case.
    pub fn is_walkable(&self) -> bool {
        !matches!(self, Tile::Obstacle)
    }
}

/// Ce que les robots ont découvert collectivement — le seul état visible par
/// les collecteurs et les éclaireurs. Distinct de [`World::resources`] qui est
/// la vérité-terrain, invisible aux robots.
///
/// En phase concurrente (Personne 3), cette structure est enveloppée dans un
/// `Arc<Mutex<KnowledgeBase>>` partagé entre tous les threads de robots.
#[derive(Debug, Clone, Default)]
pub struct KnowledgeBase {
    known_resources: HashMap<Position, Resource>,
}

impl KnowledgeBase {
    pub fn new() -> Self {
        KnowledgeBase::default()
    }

    /// Enregistre une ressource découverte par un éclaireur.
    /// Met à jour la quantité si la position était déjà connue.
    pub fn report_resource(&mut self, pos: Position, resource: Resource) {
        self.known_resources.insert(pos, resource);
    }

    /// Lecture des ressources connues (pour le ciblage des collecteurs).
    pub fn known_resources(&self) -> &HashMap<Position, Resource> {
        &self.known_resources
    }

    /// Retire une entrée quand le gisement est épuisé ou confirmé vide.
    pub fn remove(&mut self, pos: &Position) {
        self.known_resources.remove(pos);
    }

    /// Vrai si aucune ressource n'est encore connue.
    pub fn is_empty(&self) -> bool {
        self.known_resources.is_empty()
    }
}

/// Le monde de la simulation : la carte et tout ce qu'elle contient.
///
/// C'est la structure centrale partagée. En phase concurrente (Personne 3),
/// elle sera encapsulée dans un `Arc<Mutex<World>>` pour être lue/écrite par
/// plusieurs threads sans course de données.
#[derive(Debug, Clone)]
pub struct World {
    /// Largeur de la carte (nombre de colonnes).
    pub width: usize,
    /// Hauteur de la carte (nombre de lignes).
    pub height: usize,
    /// Grille de terrain, stockée ligne par ligne (row-major).
    /// L'indice d'une case `(x, y)` est `y * width + x`.
    tiles: Vec<Tile>,
    /// Position de la base centrale (point de départ des robots).
    pub base: Position,
    /// Gisements de ressources, indexés par leur position.
    pub resources: HashMap<Position, Resource>,
}

impl World {
    /// Construit un monde vide (toutes les cases [`Tile::Empty`]) avec une
    /// base placée au centre. Sert de base à la génération (voir module
    /// `map`) et aux tests.
    ///
    /// # Panics
    /// Panique si `width` ou `height` vaut 0.
    pub fn new(width: usize, height: usize) -> Self {
        assert!(width > 0 && height > 0, "La carte doit avoir une taille non nulle");
        let base = Position::new(width / 2, height / 2);
        let mut world = World {
            width,
            height,
            tiles: vec![Tile::Empty; width * height],
            base,
            resources: HashMap::new(),
        };
        world.set_tile(base, Tile::Base);
        world
    }

    /// Vrai si la position est à l'intérieur de la carte.
    pub fn in_bounds(&self, pos: Position) -> bool {
        pos.x < self.width && pos.y < self.height
    }

    /// Indice linéaire d'une position dans le vecteur `tiles`.
    #[inline]
    fn index(&self, pos: Position) -> usize {
        pos.y * self.width + pos.x
    }

    /// Renvoie la tuile à une position donnée.
    ///
    /// Renvoie `None` si la position est hors limites.
    pub fn tile(&self, pos: Position) -> Option<Tile> {
        if self.in_bounds(pos) {
            Some(self.tiles[self.index(pos)])
        } else {
            None
        }
    }

    /// Modifie la tuile à une position donnée.
    ///
    /// # Panics
    /// Panique si la position est hors limites (erreur de programmation).
    pub fn set_tile(&mut self, pos: Position, tile: Tile) {
        assert!(self.in_bounds(pos), "set_tile hors limites : {:?}", pos);
        let idx = self.index(pos);
        self.tiles[idx] = tile;
    }

    /// Vrai si la case est un obstacle.
    pub fn is_obstacle(&self, pos: Position) -> bool {
        matches!(self.tile(pos), Some(Tile::Obstacle))
    }

    /// Vrai si un robot peut occuper cette case (dans la carte et non-obstacle).
    pub fn is_walkable(&self, pos: Position) -> bool {
        self.tile(pos).map(|t| t.is_walkable()).unwrap_or(false)
    }

    /// Renvoie le gisement de ressource présent sur la case, s'il existe.
    pub fn resource_at(&self, pos: Position) -> Option<&Resource> {
        self.resources.get(&pos)
    }

    /// Voisins traversables d'une position (4-connexité), utiles au pathfinding.
    pub fn walkable_neighbors(&self, pos: Position) -> Vec<Position> {
        pos.neighbors(self.width, self.height)
            .into_iter()
            .filter(|&p| self.is_walkable(p))
            .collect()
    }

    /// Accès en lecture seule à la grille de tuiles (pour l'UI / debug).
    pub fn tiles(&self) -> &[Tile] {
        &self.tiles
    }
}
