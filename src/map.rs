//! Génération procédurale de la carte.
//!
//! Responsabilités (Personne 1) :
//!   * générer les obstacles à partir d'un **bruit de Perlin** ;
//!   * placer la base centrale ;
//!   * disperser les ressources (énergie / cristaux) avec des quantités
//!     aléatoires (50–200 unités), sur des cases libres.
//!
//! La génération est **déterministe pour une graine (`seed`) donnée**, ce qui
//! facilite le débogage et les tests : deux appels avec la même graine
//! produisent exactement la même carte.

use crate::types::{Position, Resource, ResourceKind, Tile, World};
use noise::{NoiseFn, Perlin};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

/// Paramètres de génération de la carte.
///
/// Regrouper la configuration ici permet d'ajuster facilement la difficulté
/// et la densité sans toucher à l'algorithme.
#[derive(Debug, Clone, Copy)]
pub struct MapConfig {
    pub width: usize,
    pub height: usize,
    /// Graine de génération (rend la carte reproductible).
    pub seed: u32,
    /// Échelle du bruit de Perlin : plus la valeur est petite, plus les
    /// formations d'obstacles sont grandes et lisses.
    pub noise_scale: f64,
    /// Seuil au-dessus duquel une case devient un obstacle (le bruit est
    /// normalisé dans `[0, 1]`). ~0.6 donne des amas d'obstacles épars.
    pub obstacle_threshold: f64,
    /// Nombre de sources d'énergie à placer.
    pub energy_count: usize,
    /// Nombre de gisements de cristaux à placer.
    pub crystal_count: usize,
}

impl Default for MapConfig {
    fn default() -> Self {
        MapConfig {
            width: 60,
            height: 30,
            seed: 42,
            noise_scale: 0.12,
            obstacle_threshold: 0.62,
            energy_count: 8,
            crystal_count: 8,
        }
    }
}

/// Quantité minimale d'une ressource (incluse).
const RESOURCE_MIN: u32 = 50;
/// Quantité maximale d'une ressource (incluse).
const RESOURCE_MAX: u32 = 200;

/// Génère un monde complet à partir d'une configuration.
///
/// Étapes :
///   1. création d'un monde vide avec base centrale ;
///   2. pose des obstacles via le bruit de Perlin ;
///   3. dégagement d'une zone autour de la base ;
///   4. placement aléatoire des ressources sur des cases libres.
pub fn generate(config: MapConfig) -> World {
    let mut world = World::new(config.width, config.height);
    let mut rng = StdRng::seed_from_u64(config.seed as u64);

    place_obstacles(&mut world, &config);
    clear_base_surroundings(&mut world);
    place_resources(&mut world, &config, &mut rng);

    world
}

/// Génère un monde avec la configuration par défaut.
pub fn generate_default() -> World {
    generate(MapConfig::default())
}

/// Place les obstacles en échantillonnant un bruit de Perlin.
///
/// Le bruit Perlin renvoie des valeurs dans `[-1, 1]` ; on les ramène dans
/// `[0, 1]` puis on compare au seuil. Les coordonnées sont multipliées par
/// `noise_scale` pour contrôler la « taille » des amas.
fn place_obstacles(world: &mut World, config: &MapConfig) {
    let perlin = Perlin::new(config.seed);

    for y in 0..world.height {
        for x in 0..world.width {
            let nx = x as f64 * config.noise_scale;
            let ny = y as f64 * config.noise_scale;
            // Perlin::get -> [-1, 1] ; on normalise vers [0, 1].
            let raw = perlin.get([nx, ny]);
            let normalized = (raw + 1.0) / 2.0;

            if normalized > config.obstacle_threshold {
                world.set_tile(Position::new(x, y), Tile::Obstacle);
            }
        }
    }
}

/// Dégage la base et ses cases adjacentes pour garantir que les robots
/// puissent toujours en sortir, même si le bruit y a posé des obstacles.
fn clear_base_surroundings(world: &mut World) {
    let base = world.base;
    world.set_tile(base, Tile::Base);

    for neighbor in base.neighbors(world.width, world.height) {
        if world.is_obstacle(neighbor) {
            world.set_tile(neighbor, Tile::Empty);
        }
    }
}

/// Place les ressources sur des cases libres choisies aléatoirement.
///
/// Une case est éligible si elle est traversable, n'est pas la base, et ne
/// porte pas déjà une ressource. Chaque gisement reçoit une quantité
/// aléatoire dans `[RESOURCE_MIN, RESOURCE_MAX]`.
fn place_resources(world: &mut World, config: &MapConfig, rng: &mut StdRng) {
    place_resource_kind(world, ResourceKind::Energy, config.energy_count, rng);
    place_resource_kind(world, ResourceKind::Crystal, config.crystal_count, rng);
}

/// Place `count` ressources d'un type donné.
fn place_resource_kind(world: &mut World, kind: ResourceKind, count: usize, rng: &mut StdRng) {
    let mut placed = 0;
    // Garde-fou : on borne le nombre d'essais pour éviter une boucle infinie
    // si la carte est saturée (cas limite : presque que des obstacles).
    let max_attempts = count.saturating_mul(50).max(200);
    let mut attempts = 0;

    while placed < count && attempts < max_attempts {
        attempts += 1;
        let pos = Position::new(rng.gen_range(0..world.width), rng.gen_range(0..world.height));

        if !world.is_walkable(pos) || pos == world.base || world.resource_at(pos).is_some() {
            continue;
        }

        let quantity = rng.gen_range(RESOURCE_MIN..=RESOURCE_MAX);
        world.resources.insert(pos, Resource::new(kind, quantity));
        placed += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn la_base_est_au_centre_et_traversable() {
        let world = generate(MapConfig::default());
        let expected = Position::new(world.width / 2, world.height / 2);
        assert_eq!(world.base, expected);
        assert_eq!(world.tile(world.base), Some(Tile::Base));
        assert!(world.is_walkable(world.base));
    }

    #[test]
    fn la_generation_est_deterministe() {
        let a = generate(MapConfig::default());
        let b = generate(MapConfig::default());
        assert_eq!(a.tiles(), b.tiles());
        assert_eq!(a.resources, b.resources);
    }

    #[test]
    fn le_bon_nombre_de_ressources_est_place() {
        let config = MapConfig::default();
        let world = generate(config);
        let energy = world
            .resources
            .values()
            .filter(|r| r.kind == ResourceKind::Energy)
            .count();
        let crystal = world
            .resources
            .values()
            .filter(|r| r.kind == ResourceKind::Crystal)
            .count();
        assert_eq!(energy, config.energy_count);
        assert_eq!(crystal, config.crystal_count);
    }

    #[test]
    fn les_quantites_sont_dans_la_plage() {
        let world = generate(MapConfig::default());
        for resource in world.resources.values() {
            assert!(resource.quantity >= RESOURCE_MIN);
            assert!(resource.quantity <= RESOURCE_MAX);
        }
    }

    #[test]
    fn les_ressources_ne_sont_pas_sur_des_obstacles_ni_la_base() {
        let world = generate(MapConfig::default());
        for pos in world.resources.keys() {
            assert!(world.is_walkable(*pos));
            assert_ne!(*pos, world.base);
        }
    }

    #[test]
    fn les_voisins_restent_dans_la_carte() {
        let world = World::new(10, 10);
        // Coin supérieur gauche : seulement 2 voisins valides.
        let corner = Position::new(0, 0);
        assert_eq!(corner.neighbors(world.width, world.height).len(), 2);
        // Case centrale : 4 voisins.
        let center = Position::new(5, 5);
        assert_eq!(center.neighbors(world.width, world.height).len(), 4);
    }

    #[test]
    fn une_ressource_s_epuise_correctement() {
        let mut r = Resource::new(ResourceKind::Energy, 2);
        assert!(!r.is_depleted());
        assert!(r.take_one());
        assert!(r.take_one());
        assert!(r.is_depleted());
        assert!(!r.take_one());
    }
}
