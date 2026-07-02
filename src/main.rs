//! Interface terminal Ratatui — Personne 4.
//!
//! Assemble tout le projet : lance la simulation (Personne 3) dans un thread
//! séparé, reçoit les positions des robots via un channel `mpsc`, lit la carte
//! et les ressources via les handles partagés, et dessine le tout en temps réel.
//!
//! Touche : n'importe quelle touche quitte la simulation.

use std::collections::HashMap;
use std::io::{self, Stdout};
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use crossterm::event::{self, Event};
use crossterm::execute;
use crossterm::terminal::{
    disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen,
};
use ratatui::backend::CrosstermBackend;
use ratatui::layout::{Constraint, Direction, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Paragraph};
use ratatui::{Frame, Terminal};

use resource_collection_sim::comm::{RobotKind, RobotSnapshot};
use resource_collection_sim::map::{self, MapConfig};
use resource_collection_sim::sim::Simulation;
use resource_collection_sim::types::{Position, ResourceKind, Tile, World};

/// Nombre d'éclaireurs lancés par la simulation.
const N_SCOUTS: usize = 2;
/// Nombre de collecteurs lancés par la simulation.
const N_COLLECTORS: usize = 3;
/// Nombre maximum de ticks de simulation.
const MAX_TICKS: usize = 500_000;

fn main() -> io::Result<()> {
    // --- 1. Générer le monde et préparer la simulation ---------------------
    // Graine optionnelle passée en argument : `cargo run -- 123`.
    let seed = std::env::args()
        .nth(1)
        .and_then(|s| s.parse::<u32>().ok())
        .unwrap_or(42);

    let world = map::generate(MapConfig {
        seed,
        ..MapConfig::default()
    });

    let sim = Simulation::new(world);

    // Handles partagés pour lire le décor (terrain + ressources) au rendu.
    let world_handle = sim.world_handle();

    // --- 2. Lancer la simulation dans un thread séparé ---------------------
    // run_all_sending lance les éclaireurs (IDs 0..N_SCOUTS) ET les collecteurs
    // (IDs N_SCOUTS..) : les scouts peuplent eux-mêmes la KB en explorant
    // (vrai fog-of-war), plus besoin de discover_all().
    let (tx, rx) = mpsc::channel::<RobotSnapshot>();
    let sim_thread = thread::spawn(move || {
        sim.run_all_sending(N_SCOUTS, N_COLLECTORS, 42, MAX_TICKS, tx);
    });

    // --- 3. Préparer le terminal Ratatui -----------------------------------
    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    // --- 4. Boucle de rendu ------------------------------------------------
    let result = run_app(&mut terminal, &world_handle, rx);

    // --- 5. Restaurer le terminal quoi qu'il arrive ------------------------
    disable_raw_mode()?;
    execute!(terminal.backend_mut(), LeaveAlternateScreen)?;
    terminal.show_cursor()?;

    // On laisse le thread de simulation se terminer en arrière-plan.
    drop(sim_thread);

    result
}

/// Boucle principale : draine le channel, met à jour l'état, dessine, lit le clavier.
fn run_app(
    terminal: &mut Terminal<CrosstermBackend<Stdout>>,
    world_handle: &Arc<Mutex<World>>,
    rx: mpsc::Receiver<RobotSnapshot>,
) -> io::Result<()> {
    // Dernier snapshot connu par robot (indexé par id).
    let mut robots: HashMap<usize, RobotSnapshot> = HashMap::new();

    loop {
        // Draine tous les snapshots disponibles ce tour-ci (garde le plus récent).
        while let Ok(snap) = rx.try_recv() {
            robots.insert(snap.id, snap);
        }

        // Total déposé = somme des `deposited` de chaque robot.
        let total_deposited: u32 = robots.values().map(|r| r.deposited).sum();

        // Snapshot du monde pour ce rendu (clone court pour relâcher le lock vite).
        let world = world_handle.lock().unwrap().clone();

        terminal.draw(|f| ui(f, &world, &robots, total_deposited))?;

        // N'importe quelle touche quitte.
        if event::poll(Duration::from_millis(50))? {
            if let Event::Key(_) = event::read()? {
                break;
            }
        }
    }
    Ok(())
}

/// Dessine une frame : la carte à gauche, les infos à droite.
fn ui(f: &mut Frame, world: &World, robots: &HashMap<usize, RobotSnapshot>, total: u32) {
    let chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Min(0), Constraint::Length(28)])
        .split(f.area());

    draw_map(f, chunks[0], world, robots);
    draw_sidebar(f, chunks[1], world, robots, total);
}

/// Rendu de la grille avec le codage couleur exact demandé par le sujet.
fn draw_map(f: &mut Frame, area: Rect, world: &World, robots: &HashMap<usize, RobotSnapshot>) {
    // Positions occupées par un robot, avec son type (éclaireur 'x' / collecteur 'o').
    let robot_cells: HashMap<Position, RobotKind> =
        robots.values().map(|r| (r.pos, r.kind)).collect();

    let mut lines: Vec<Line> = Vec::with_capacity(world.height);

    for y in 0..world.height {
        let mut spans: Vec<Span> = Vec::with_capacity(world.width);
        for x in 0..world.width {
            let pos = Position::new(x, y);

            // Priorité d'affichage : robot > ressource > terrain.
            let (ch, color) = if let Some(kind) = robot_cells.get(&pos) {
                match kind {
                    RobotKind::Scout => ('x', Color::Red),         // Éclaireur
                    RobotKind::Collector => ('o', Color::Magenta), // Collecteur
                }
            } else if let Some(res) = world.resource_at(pos) {
                match res.kind {
                    ResourceKind::Energy => ('E', Color::Green),
                    ResourceKind::Crystal => ('C', Color::LightMagenta),
                }
            } else {
                match world.tile(pos) {
                    Some(Tile::Obstacle) => ('O', Color::LightCyan),
                    Some(Tile::Base) => ('#', Color::LightGreen),
                    Some(Tile::Empty) => ('.', Color::DarkGray),
                    None => (' ', Color::Reset),
                }
            };

            spans.push(Span::styled(ch.to_string(), Style::default().fg(color)));
        }
        lines.push(Line::from(spans));
    }

    let map_widget = Paragraph::new(lines)
        .block(Block::default().borders(Borders::ALL).title(" Carte "));
    f.render_widget(map_widget, area);
}

/// Panneau latéral : compteur de ressources et état des robots.
fn draw_sidebar(
    f: &mut Frame,
    area: Rect,
    world: &World,
    robots: &HashMap<usize, RobotSnapshot>,
    total: u32,
) {
    // Reste à collecter sur la carte (vérité terrain, pour information).
    let remaining: u32 = world.resources.values().map(|r| r.quantity).sum();

    let mut lines = vec![
        Line::from(Span::styled(
            "Collecté à la base",
            Style::default().add_modifier(Modifier::BOLD),
        )),
        Line::from(Span::styled(
            format!("  {total} unités"),
            Style::default().fg(Color::LightGreen),
        )),
        Line::from(""),
        Line::from(format!("Reste sur carte : {remaining}")),
        Line::from(""),
        Line::from(Span::styled(
            "Robots",
            Style::default().add_modifier(Modifier::BOLD),
        )),
    ];

    let mut ids: Vec<&usize> = robots.keys().collect();
    ids.sort();
    for id in ids {
        let r = &robots[id];
        let tag = match r.kind {
            RobotKind::Scout => "x",
            RobotKind::Collector => "o",
        };
        lines.push(Line::from(format!(
            "  {} #{} {} ({},{}) d:{}",
            tag, r.id, r.state_label, r.pos.x, r.pos.y, r.deposited
        )));
    }

    lines.push(Line::from(""));
    lines.push(Line::from(Span::styled(
        "Touche = quitter",
        Style::default().fg(Color::DarkGray),
    )));

    let sidebar = Paragraph::new(lines)
        .block(Block::default().borders(Borders::ALL).title(" Infos "));
    f.render_widget(sidebar, area);
}