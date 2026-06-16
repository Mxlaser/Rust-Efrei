//! Comportements des robots : logique de décision **pure**.
//!
//! Ce module appartient à la **Personne 2** (comportements). Sa règle d'or :
//! une fonction `*_decide` est de la **logique pure**. Elle reçoit l'état du
//! robot (mutable, p. ex. pour faire avancer son générateur aléatoire) et une
//! **vue en lecture** du monde ([`World`]), puis renvoie une **intention**
//! ([`Action`]). Elle ne crée *aucun* thread, ne prend *aucun* lock, n'envoie
//! sur *aucun* channel, ne fait *aucun* `sleep` et ne mute *jamais* le monde.
//!
//! C'est la **Personne 3** (threads & communication) qui exécutera ces
//! décisions dans des threads et **appliquera** réellement les [`Action`] sur
//! le monde partagé. Le contrat est donc strict : *décider* (P2) et *appliquer*
//! (P3) sont deux étapes séparées.

use crate::types::{Position, Resource, World};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

/// Intention produite par un robot pour un tick de simulation.
///
/// C'est le **vocabulaire partagé** entre la décision (Personne 2) et son
/// application (Personne 3). Il est volontairement **complet et figé** : il
/// couvre les besoins de l'éclaireur *et* du collecteur, même si certaines
/// variantes ne seront branchées qu'au Sprint 2. P3 fera un `match` exhaustif
/// dessus pour faire évoluer le monde.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    /// Se déplacer vers une case **adjacente** et traversable.
    MoveTo(Position),
    /// Collecteur : ramasser une unité de ressource sur la case courante.
    #[allow(dead_code)]
    Collect(Position),
    /// Collecteur : décharger sa cargaison une fois revenu à la base.
    #[allow(dead_code)]
    Unload,
    /// Éclaireur : signaler une ressource observée à la position donnée.
    Report(Position, Resource),
    /// Rien à faire ce tick (p. ex. robot encerclé d'obstacles).
    Idle,
}

/// État propre à un robot **éclaireur**.
///
/// L'éclaireur explore la carte au hasard et signale les ressources qu'il
/// rencontre. Il embarque son **propre** générateur aléatoire ([`StdRng`])
/// initialisé par une graine, ce qui rend ses décisions **déterministes** et
/// donc testables.
pub struct ScoutState {
    /// Position courante du robot sur la grille.
    pub pos: Position,
    /// Générateur aléatoire personnel (choix de la direction d'exploration).
    rng: StdRng,
}

impl ScoutState {
    /// Crée un éclaireur à la position `start`, avec un PRNG initialisé par
    /// `seed`. À graine égale, la suite des décisions est reproductible.
    pub fn new(start: Position, seed: u64) -> Self {
        ScoutState {
            pos: start,
            rng: StdRng::seed_from_u64(seed),
        }
    }
}

/// Décide de l'**intention** d'un éclaireur pour le tick courant.
///
/// Logique pure : aucune mutation du monde, aucun effet de bord hormis
/// l'avancée du PRNG porté par `state`.
///
/// Priorités :
/// 1. Si une ressource se trouve sur la case courante, la signaler via
///    [`Action::Report`].
/// 2. Sinon, choisir au hasard une case **traversable et adjacente** et
///    renvoyer [`Action::MoveTo`].
/// 3. Si aucun voisin n'est traversable (robot encerclé), renvoyer
///    [`Action::Idle`].
///
/// Cette fonction **ne déplace pas** le robot : elle renvoie seulement
/// l'intention. C'est P3 qui, en appliquant l'`Action`, mettra à jour la
/// position (cf. le helper de test [`apply`]).
pub fn scout_decide(state: &mut ScoutState, world: &World) -> Action {
    if let Some(&resource) = world.resource_at(state.pos) {
        return Action::Report(state.pos, resource);
    }

    let neighbors = world.walkable_neighbors(state.pos);
    if neighbors.is_empty() {
        return Action::Idle;
    }

    let choice = state.rng.gen_range(0..neighbors.len());
    Action::MoveTo(neighbors[choice])
}

/// Helper **réservé aux tests** : applique une [`Action`] à un [`ScoutState`]
/// pour simuler une séquence de décisions.
///
/// En production, c'est la Personne 3 qui applique les actions sur le monde
/// partagé ; ce helper ne sert qu'à vérifier la reproductibilité d'une suite
/// de décisions sans dépendre du code de P3.
#[cfg(test)]
fn apply(state: &mut ScoutState, action: Action) {
    if let Action::MoveTo(next) = action {
        state.pos = next;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{ResourceKind, Tile};

    /// Sur une case portant une ressource → `Report` avec la bonne
    /// position et la bonne ressource.
    #[test]
    fn report_sur_ressource() {
        let mut world = World::new(5, 5);
        let pos = Position::new(1, 1);
        let res = Resource::new(ResourceKind::Crystal, 7);
        world.resources.insert(pos, res);

        let mut scout = ScoutState::new(pos, 42);
        assert_eq!(scout_decide(&mut scout, &world), Action::Report(pos, res));
    }

    /// Sur une case sans ressource → `MoveTo` vers une case **adjacente** et
    /// **traversable** (jamais un obstacle, jamais hors limites).
    #[test]
    fn moveto_case_valide() {
        let mut world = World::new(5, 5);
        let pos = Position::new(2, 2);
        // Mur d'obstacles autour sauf une ouverture : force le seul choix légal.
        world.set_tile(Position::new(1, 2), Tile::Obstacle);
        world.set_tile(Position::new(3, 2), Tile::Obstacle);
        world.set_tile(Position::new(2, 1), Tile::Obstacle);
        // (2, 3) reste libre.

        let mut scout = ScoutState::new(pos, 1);
        match scout_decide(&mut scout, &world) {
            Action::MoveTo(next) => {
                assert_eq!(next, Position::new(2, 3));
                assert!(world.is_walkable(next), "doit être traversable");
                assert_eq!(pos.manhattan_distance(&next), 1, "doit être adjacente");
            }
            other => panic!("attendu MoveTo, obtenu {other:?}"),
        }
    }

    /// Quelle que soit la graine, `MoveTo` ne vise qu'un voisin traversable.
    #[test]
    fn moveto_jamais_obstacle_ni_hors_limites() {
        let mut world = World::new(5, 5);
        let pos = Position::new(2, 2);
        world.set_tile(Position::new(1, 2), Tile::Obstacle);

        for seed in 0..50 {
            let mut scout = ScoutState::new(pos, seed);
            if let Action::MoveTo(next) = scout_decide(&mut scout, &world) {
                assert!(world.is_walkable(next));
                assert!(world.in_bounds(next));
                assert_eq!(pos.manhattan_distance(&next), 1);
            }
        }
    }

    /// Éclaireur encerclé d'obstacles → `Idle`.
    #[test]
    fn idle_si_encercle() {
        let mut world = World::new(5, 5);
        let pos = Position::new(2, 2);
        for n in pos.neighbors(world.width, world.height) {
            world.set_tile(n, Tile::Obstacle);
        }

        let mut scout = ScoutState::new(pos, 99);
        assert_eq!(scout_decide(&mut scout, &world), Action::Idle);
    }

    /// À graine fixe, une séquence de décisions est reproductible.
    #[test]
    fn sequence_reproductible() {
        let world = World::new(8, 8);
        let start = Position::new(4, 4);

        let run = || {
            let mut scout = ScoutState::new(start, 2024);
            let mut actions = Vec::new();
            for _ in 0..20 {
                let action = scout_decide(&mut scout, &world);
                apply(&mut scout, action);
                actions.push(action);
            }
            actions
        };

        assert_eq!(run(), run(), "même graine ⇒ même séquence");
    }

    /// `scout_decide` ne mute jamais le monde (vue en lecture seule).
    #[test]
    fn ne_mute_pas_le_monde() {
        let mut world = World::new(5, 5);
        world
            .resources
            .insert(Position::new(0, 0), Resource::new(ResourceKind::Energy, 3));
        let before = world.clone();

        let mut scout = ScoutState::new(Position::new(2, 2), 7);
        let _ = scout_decide(&mut scout, &world);

        assert_eq!(world.tiles(), before.tiles());
        assert_eq!(world.resources, before.resources);
    }
}
