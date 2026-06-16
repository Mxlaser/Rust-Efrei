//! Démonstration de la partie « Carte & structures de données » (Personne 1).
//!
//! Ce binaire n'utilise **pas** encore Ratatui : il sert uniquement à valider
//! visuellement la génération de carte en l'affichant en ASCII dans le
//! terminal, avant que l'équipe n'intègre l'UI (Personne 4) et la concurrence
//! (Personne 3).
//!
//! Lancement :
//!   cargo run                 # carte par défaut
//!   cargo run -- 123          # carte avec la graine 123

use resource_collection_sim::map::{self, MapConfig};
use resource_collection_sim::sim::Simulation;
use resource_collection_sim::types::{Position, ResourceKind, Tile, World};

fn main() {
    // Graine optionnelle passée en argument de ligne de commande.
    let seed = std::env::args()
        .nth(1)
        .and_then(|s| s.parse::<u32>().ok())
        .unwrap_or(MapConfig::default().seed);

    let config = MapConfig {
        seed,
        ..MapConfig::default()
    };
    let world = map::generate(config);

    println!("=== Simulation de collecte de ressources — aperçu de la carte ===");
    println!(
        "Taille : {}x{}  |  graine : {}",
        world.width, world.height, seed
    );
    println!();

    render_ascii(&world);
    print_summary(&world);
    run_simulation(config);
}

fn run_simulation(config: MapConfig) {
    println!();
    println!("=== Simulation concurrente (2 éclaireurs + 3 collecteurs) ===");

    let world = map::generate(config);
    let total: u32 = world.resources.values().map(|r| r.quantity).sum();

    let sim = Simulation::new(world);
    let results = sim.run_scouts_and_collectors(2, 3, 42, 500_000);
    let collected: u32 = results.iter().sum();

    for (i, &units) in results.iter().enumerate() {
        println!("  Collecteur #{i} : {units} unités déposées");
    }
    println!("  Total collecté : {collected} / {total} unités");
}

/// Affiche la carte en ASCII, en respectant les symboles du cahier des charges.
///
/// Légende : `O` obstacle, `E` énergie, `C` cristal, `#` base, `.` case libre.
fn render_ascii(world: &World) {
    for y in 0..world.height {
        let mut line = String::with_capacity(world.width);
        for x in 0..world.width {
            let pos = Position::new(x, y);
            let ch = symbol_at(world, pos);
            line.push(ch);
        }
        println!("{line}");
    }
}

/// Détermine le caractère à afficher pour une case (ressource prioritaire sur
/// le terrain libre).
fn symbol_at(world: &World, pos: Position) -> char {
    if let Some(resource) = world.resource_at(pos) {
        return resource.kind.symbol();
    }
    match world.tile(pos) {
        Some(Tile::Obstacle) => 'O',
        Some(Tile::Base) => '#',
        Some(Tile::Empty) => '.',
        None => ' ',
    }
}

/// Affiche un récapitulatif chiffré de la carte générée.
fn print_summary(world: &World) {
    let total_tiles = world.width * world.height;
    let obstacles = world
        .tiles()
        .iter()
        .filter(|t| matches!(t, Tile::Obstacle))
        .count();

    let mut energy_sites = 0;
    let mut crystal_sites = 0;
    let mut energy_units = 0u32;
    let mut crystal_units = 0u32;
    for resource in world.resources.values() {
        match resource.kind {
            ResourceKind::Energy => {
                energy_sites += 1;
                energy_units += resource.quantity;
            }
            ResourceKind::Crystal => {
                crystal_sites += 1;
                crystal_units += resource.quantity;
            }
        }
    }

    println!();
    println!("--- Récapitulatif ---");
    println!("Base                : {:?}", world.base);
    println!(
        "Obstacles           : {} ({:.1}% de la carte)",
        obstacles,
        100.0 * obstacles as f64 / total_tiles as f64
    );
    println!("Sources d'énergie   : {energy_sites} gisements, {energy_units} unités au total");
    println!("Gisements cristaux  : {crystal_sites} gisements, {crystal_units} unités au total");
}
