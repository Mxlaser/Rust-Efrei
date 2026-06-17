//! Couche de concurrence — Personne 3.
//!
//! [`Simulation`] enveloppe le monde et la KB dans des `Arc<Mutex<>>` puis
//! applique les décisions de P2 dans des threads dédiés.
//!
//! ## Contrat d'application des actions (wiring P2 → P3)
//!
//! | Action             | Ce que P3 fait                                               |
//! |--------------------|--------------------------------------------------------------|
//! | `MoveTo(p)`        | `state.pos = p`                                              |
//! | `Collect(p)`       | `take_one()` sur world ; `carrying += 1` ; retire KB si épuisé |
//! | `Unload`           | `deposited += carrying ; carrying = 0`                       |
//! | `Report(p, res)`   | `kb.report_resource(p, res)`                                 |
//! | `Idle`             | rien                                                         |
//!
//! ## Ordre d'acquisition des verrous
//!
//! Toujours **`world` en premier, `kb` en second** — dans tout le code.

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc::Sender;
use std::sync::{Arc, Mutex};
use std::thread;

use crate::comm::{RobotKind, RobotSnapshot};
use crate::robots::{collector_decide, scout_decide, Action, CollectorState, ScoutState};
use crate::types::{KnowledgeBase, World};

// ---------------------------------------------------------------------------
// Simulation
// ---------------------------------------------------------------------------

pub struct Simulation {
    pub world: Arc<Mutex<World>>,
    pub kb: Arc<Mutex<KnowledgeBase>>,
}

impl Simulation {
    pub fn new(world: World) -> Self {
        Simulation {
            world: Arc::new(Mutex::new(world)),
            kb: Arc::new(Mutex::new(KnowledgeBase::new())),
        }
    }

    /// Handle clonable vers le monde — à passer à P4 pour le rendu.
    pub fn world_handle(&self) -> Arc<Mutex<World>> {
        Arc::clone(&self.world)
    }

    /// Handle clonable vers la KnowledgeBase — à passer à P2 pour les scouts.
    pub fn kb_handle(&self) -> Arc<Mutex<KnowledgeBase>> {
        Arc::clone(&self.kb)
    }

    /// Pré-remplit la KB avec toutes les ressources du monde.
    /// Simule le fait que les éclaireurs ont tout découvert — utile pour les tests.
    pub fn discover_all(&self) {
        let world = self.world.lock().unwrap();
        let mut kb = self.kb.lock().unwrap();
        for (&pos, &resource) in &world.resources {
            kb.report_resource(pos, resource);
        }
    }

    // -----------------------------------------------------------------------
    // Simulation complète : scouts + collecteurs
    // -----------------------------------------------------------------------

    /// Lance `n_scouts` éclaireurs et `n_collectors` collecteurs en parallèle.
    ///
    /// Chaque scout reçoit la graine `base_seed + index` pour que leurs
    /// marches aléatoires soient indépendantes.
    ///
    /// Retourne le vecteur des unités déposées à la base par chaque collecteur.
    pub fn run_scouts_and_collectors(
        &self,
        n_scouts: usize,
        n_collectors: usize,
        base_seed: u64,
        max_ticks: usize,
    ) -> Vec<u32> {
        let base = self.world.lock().unwrap().base;

        // Compteur décrémenté par chaque scout à sa fin — les collecteurs
        // n'ont le droit de s'arrêter que quand il atteint zéro.
        let scouts_remaining = Arc::new(AtomicUsize::new(n_scouts));

        // --- Threads éclaireurs ---
        let scout_handles: Vec<_> = (0..n_scouts)
            .map(|i| {
                let world = Arc::clone(&self.world);
                let kb = Arc::clone(&self.kb);
                let scouts_remaining = Arc::clone(&scouts_remaining);
                let seed = base_seed + i as u64;

                thread::spawn(move || {
                    let mut state = ScoutState::new(base, seed);

                    for _ in 0..max_ticks {
                        let action = {
                            let world = world.lock().unwrap();
                            scout_decide(&mut state, &world)
                        };
                        match action {
                            Action::MoveTo(p) => state.pos = p,
                            Action::Report(p, res) => {
                                kb.lock().unwrap().report_resource(p, res);
                            }
                            Action::Idle => {}
                            _ => {}
                        }
                    }
                    scouts_remaining.fetch_sub(1, Ordering::Relaxed);
                })
            })
            .collect();

        // --- Threads collecteurs ---
        let collector_handles: Vec<_> = (0..n_collectors)
            .map(|_| {
                let world = Arc::clone(&self.world);
                let kb = Arc::clone(&self.kb);
                let scouts_remaining = Arc::clone(&scouts_remaining);

                thread::spawn(move || {
                    let mut state = CollectorState::new(base, 10);
                    let mut deposited = 0u32;

                    for _ in 0..max_ticks {
                        let known = kb.lock().unwrap().known_resources().clone();

                        let action = {
                            let world = world.lock().unwrap();
                            collector_decide(&mut state, &world, &known)
                        };

                        apply_collector_action(&mut state, &mut deposited, action, &world, &kb);

                        // N'arrête que quand tous les scouts sont finis ET KB vide.
                        if matches!(action, Action::Idle) && state.carrying == 0
                            && scouts_remaining.load(Ordering::Relaxed) == 0
                            && kb.lock().unwrap().is_empty()
                        {
                            break;
                        }
                    }

                    deposited
                })
            })
            .collect();

        for h in scout_handles {
            h.join().unwrap();
        }
        collector_handles.into_iter().map(|h| h.join().unwrap()).collect()
    }

    /// Variante de [`run_scouts_and_collectors`] qui stream chaque tick via channel.
    ///
    /// Scouts : IDs `0..n_scouts`, symbole `'x'` (`RobotKind::Scout`).
    /// Collecteurs : IDs `n_scouts..n_scouts+n_collectors`, symbole `'o'` (`RobotKind::Collector`).
    ///
    /// ## Usage (P4)
    /// ```ignore
    /// let (tx, rx) = std::sync::mpsc::channel::<RobotSnapshot>();
    /// std::thread::spawn(move || sim.run_all_sending(2, 3, 42, 500_000, tx));
    /// // boucle Ratatui :
    /// while let Ok(snap) = rx.try_recv() {
    ///     // snap.kind == RobotKind::Scout → affiche 'x'
    ///     // snap.kind == RobotKind::Collector → affiche 'o'
    ///     robots.insert(snap.id, snap);
    /// }
    /// ```
    pub fn run_all_sending(
        &self,
        n_scouts: usize,
        n_collectors: usize,
        base_seed: u64,
        max_ticks: usize,
        tx: Sender<RobotSnapshot>,
    ) -> Vec<u32> {
        let base = self.world.lock().unwrap().base;
        let scouts_remaining = Arc::new(AtomicUsize::new(n_scouts));

        // --- Threads éclaireurs (IDs 0..n_scouts) ---
        let scout_handles: Vec<_> = (0..n_scouts)
            .map(|i| {
                let world = Arc::clone(&self.world);
                let kb = Arc::clone(&self.kb);
                let scouts_remaining = Arc::clone(&scouts_remaining);
                let tx = tx.clone();
                let seed = base_seed + i as u64;

                thread::spawn(move || {
                    let mut state = ScoutState::new(base, seed);

                    for _ in 0..max_ticks {
                        let action = {
                            let world = world.lock().unwrap();
                            scout_decide(&mut state, &world)
                        };
                        match action {
                            Action::MoveTo(p) => state.pos = p,
                            Action::Report(p, res) => {
                                kb.lock().unwrap().report_resource(p, res);
                            }
                            Action::Idle => {}
                            _ => {}
                        }
                        let _ = tx.send(RobotSnapshot {
                            id: i,
                            kind: RobotKind::Scout,
                            pos: state.pos,
                            state_label: action_label(action),
                            deposited: 0,
                        });
                    }
                    scouts_remaining.fetch_sub(1, Ordering::Relaxed);
                })
            })
            .collect();

        // --- Threads collecteurs (IDs n_scouts..n_scouts+n_collectors) ---
        let collector_handles: Vec<_> = (0..n_collectors)
            .map(|i| {
                let world = Arc::clone(&self.world);
                let kb = Arc::clone(&self.kb);
                let scouts_remaining = Arc::clone(&scouts_remaining);
                let tx = tx.clone();
                let id = n_scouts + i;

                thread::spawn(move || {
                    let mut state = CollectorState::new(base, 10);
                    let mut deposited = 0u32;

                    for _ in 0..max_ticks {
                        let known = kb.lock().unwrap().known_resources().clone();
                        let action = {
                            let world = world.lock().unwrap();
                            collector_decide(&mut state, &world, &known)
                        };
                        apply_collector_action(&mut state, &mut deposited, action, &world, &kb);
                        let _ = tx.send(RobotSnapshot {
                            id,
                            kind: RobotKind::Collector,
                            pos: state.pos,
                            state_label: action_label(action),
                            deposited,
                        });
                        if matches!(action, Action::Idle) && state.carrying == 0
                            && scouts_remaining.load(Ordering::Relaxed) == 0
                            && kb.lock().unwrap().is_empty()
                        {
                            break;
                        }
                    }
                    deposited
                })
            })
            .collect();

        for h in scout_handles {
            h.join().unwrap();
        }
        collector_handles.into_iter().map(|h| h.join().unwrap()).collect()
    }

    // -----------------------------------------------------------------------
    // Collecteurs seuls (KB pré-remplie via discover_all)
    // -----------------------------------------------------------------------

    /// Lance `n_collectors` collecteurs avec la KB déjà remplie.
    /// Utile pour les tests et les démos sans éclaireurs.
    pub fn run(&self, n_collectors: usize, max_ticks: usize) -> Vec<u32> {
        self.run_sending(n_collectors, max_ticks, None)
    }

    /// Variante de [`run`] qui envoie un [`RobotSnapshot`] après chaque tick.
    ///
    /// P4 passe un `Sender` issu d'un `mpsc::channel()` et lit les snapshots
    /// dans sa boucle de rendu avec `rx.try_recv()`.
    ///
    /// ## Usage (P4)
    /// ```ignore
    /// let (tx, rx) = std::sync::mpsc::channel::<RobotSnapshot>();
    /// std::thread::spawn(move || sim.run_sending(3, 200_000, Some(tx)));
    /// // boucle Ratatui :
    /// while let Ok(snap) = rx.try_recv() {
    ///     robot_states.insert(snap.id, snap);
    /// }
    /// ```
    pub fn run_sending(
        &self,
        n_collectors: usize,
        max_ticks: usize,
        tx: Option<Sender<RobotSnapshot>>,
    ) -> Vec<u32> {
        let base = self.world.lock().unwrap().base;

        let handles: Vec<_> = (0..n_collectors)
            .map(|id| {
                let world = Arc::clone(&self.world);
                let kb = Arc::clone(&self.kb);
                let tx = tx.clone();

                thread::spawn(move || {
                    let mut state = CollectorState::new(base, 10);
                    let mut deposited = 0u32;

                    for _ in 0..max_ticks {
                        let known = kb.lock().unwrap().known_resources().clone();

                        let action = {
                            let world = world.lock().unwrap();
                            collector_decide(&mut state, &world, &known)
                        };

                        apply_collector_action(&mut state, &mut deposited, action, &world, &kb);

                        if let Some(ref tx) = tx {
                            let _ = tx.send(RobotSnapshot {
                                id,
                                kind: RobotKind::Collector,
                                pos: state.pos,
                                state_label: action_label(action),
                                deposited,
                            });
                        }

                        if matches!(action, Action::Idle) && state.carrying == 0 {
                            if kb.lock().unwrap().is_empty() {
                                break;
                            }
                        }
                    }

                    deposited
                })
            })
            .collect();

        handles.into_iter().map(|h| h.join().unwrap()).collect()
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Applique une action de collecteur : mute state, world, kb selon le contrat.
fn apply_collector_action(
    state: &mut CollectorState,
    deposited: &mut u32,
    action: Action,
    world: &Arc<Mutex<World>>,
    kb: &Arc<Mutex<KnowledgeBase>>,
) {
    match action {
        Action::MoveTo(p) => state.pos = p,

        Action::Collect(p) => {
            let mut world = world.lock().unwrap();
            // Extraire le résultat avant tout `remove` pour satisfaire le borrow checker.
            let (took, now_depleted) = match world.resources.get_mut(&p) {
                Some(res) => {
                    let took = res.take_one();
                    let dep = res.is_depleted();
                    (took, dep)
                }
                None => (false, true), // course : déjà épuisé par un autre robot
            };
            if took {
                state.carrying += 1;
            }
            if now_depleted {
                world.resources.remove(&p);
                drop(world); // libère world AVANT de prendre kb
                kb.lock().unwrap().remove(&p);
            }
        }

        Action::Unload => {
            *deposited += state.carrying;
            state.carrying = 0;
        }

        Action::Idle | Action::Report(..) => {}
    }
}

/// Étiquette d'une action pour les snapshots de P4.
fn action_label(action: Action) -> &'static str {
    match action {
        Action::MoveTo(_) => "Moving",
        Action::Collect(_) => "Collecting",
        Action::Unload => "Unloading",
        Action::Report(..) => "Reporting",
        Action::Idle => "Idle",
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Position, Resource, ResourceKind, World};

    fn small_world(resources: &[(Position, ResourceKind, u32)]) -> World {
        let mut world = World::new(11, 11); // base en (5, 5)
        for &(pos, kind, qty) in resources {
            world.resources.insert(pos, Resource::new(kind, qty));
        }
        world
    }

    #[test]
    fn un_collecteur_ramasse_une_ressource() {
        let pos = Position::new(5, 0);
        let world = small_world(&[(pos, ResourceKind::Energy, 1)]);
        let sim = Simulation::new(world);
        sim.discover_all();

        let results = sim.run(1, 500);
        assert_eq!(results[0], 1);
    }

    #[test]
    fn un_collecteur_vide_ressource_multi_unites() {
        let pos = Position::new(5, 0);
        let qty = 4;
        let world = small_world(&[(pos, ResourceKind::Energy, qty)]);
        let sim = Simulation::new(world);
        sim.discover_all();

        let results = sim.run(1, 2000);
        assert_eq!(results[0], qty);
    }

    #[test]
    fn deux_collecteurs_se_repartissent_les_ressources() {
        let resources = [
            (Position::new(0, 0), ResourceKind::Energy, 2),
            (Position::new(10, 10), ResourceKind::Crystal, 2),
        ];
        let total: u32 = resources.iter().map(|(_, _, q)| q).sum();
        let world = small_world(&resources);
        let sim = Simulation::new(world);
        sim.discover_all();

        let results = sim.run(2, 1000);
        assert_eq!(results.iter().sum::<u32>(), total);
    }

    #[test]
    fn trois_collecteurs_epuisent_plusieurs_gisements() {
        let resources = [
            (Position::new(0, 0), ResourceKind::Energy, 3),
            (Position::new(10, 0), ResourceKind::Crystal, 3),
            (Position::new(0, 10), ResourceKind::Energy, 3),
        ];
        let world = small_world(&resources);
        let sim = Simulation::new(world);
        sim.discover_all();

        let results = sim.run(3, 2000);
        assert_eq!(results.iter().sum::<u32>(), 9u32);
    }

    #[test]
    fn kb_vide_apres_simulation() {
        let pos = Position::new(3, 3);
        let world = small_world(&[(pos, ResourceKind::Crystal, 2)]);
        let sim = Simulation::new(world);
        sim.discover_all();
        sim.run(1, 1000);

        assert!(sim.kb.lock().unwrap().is_empty());
    }

    #[test]
    fn run_sending_emet_des_snapshots() {
        use std::collections::HashMap as HMap;
        use std::sync::mpsc;

        let pos = Position::new(5, 0);
        let world = small_world(&[(pos, ResourceKind::Energy, 2)]);
        let sim = Simulation::new(world);
        sim.discover_all();

        let (tx, rx) = mpsc::channel::<RobotSnapshot>();
        sim.run_sending(1, 1000, Some(tx));

        let mut latest: HMap<usize, RobotSnapshot> = HMap::new();
        while let Ok(snap) = rx.try_recv() {
            latest.insert(snap.id, snap);
        }
        assert!(!latest.is_empty());
        assert_eq!(latest[&0].deposited, 2);
    }

    #[test]
    fn scouts_decouvrent_et_collecteurs_ramassent() {
        // Carte 11x11, 1 ressource — les scouts la découvrent, les collecteurs ramassent.
        let pos = Position::new(0, 0);
        let world = small_world(&[(pos, ResourceKind::Energy, 1)]);
        let sim = Simulation::new(world);
        // Pas de discover_all() : les scouts doivent trouver la ressource.

        let results = sim.run_scouts_and_collectors(2, 1, 42, 50_000);
        assert_eq!(results.iter().sum::<u32>(), 1);
    }

    #[test]
    fn kb_handle_est_partageable_avec_p2() {
        let world = small_world(&[]);
        let sim = Simulation::new(world);
        let kb = sim.kb_handle();

        let kb2 = Arc::clone(&kb);
        thread::spawn(move || {
            kb2.lock()
                .unwrap()
                .report_resource(Position::new(1, 1), Resource::new(ResourceKind::Energy, 5));
        })
        .join()
        .unwrap();

        assert!(!kb.lock().unwrap().is_empty());
    }
}