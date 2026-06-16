//! Couche de concurrence — Personne 3.
//!
//! [`Simulation`] enveloppe le monde et la base de connaissance dans des
//! `Arc<Mutex<>>` puis lance chaque collecteur dans son propre thread.
//!
//! ## Ordre d'acquisition des verrous
//!
//! Toujours : `world` en premier, `kb` en second.
//! Respecter cet ordre dans tout le code évite les interblocages (deadlocks).

use std::sync::mpsc::Sender;
use std::sync::{Arc, Mutex};
use std::thread;

use crate::comm::RobotSnapshot;
use crate::robots::{Collector, CollectorState};
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

    /// Handle clonable vers la KnowledgeBase — à passer à P2 pour les éclaireurs.
    ///
    /// ## Usage (P2)
    /// ```ignore
    /// let kb = sim.kb_handle();
    /// // dans le thread éclaireur :
    /// kb.lock().unwrap().report_resource(pos, resource);
    /// ```
    pub fn kb_handle(&self) -> Arc<Mutex<KnowledgeBase>> {
        Arc::clone(&self.kb)
    }

    /// Pré-remplit la KB avec toutes les ressources du monde.
    ///
    /// Dans la simulation complète c'est le rôle des éclaireurs (P2).
    /// Cette méthode sert aux tests et à la démo sans éclaireurs.
    pub fn discover_all(&self) {
        let world = self.world.lock().unwrap();
        let mut kb = self.kb.lock().unwrap();
        for (&pos, &resource) in &world.resources {
            kb.report_resource(pos, resource);
        }
    }

    /// Lance `n_collectors` robots collecteurs en parallèle.
    ///
    /// Chaque robot tourne dans son propre thread jusqu'à ce que la KB soit
    /// vide et qu'il soit `Idle`, ou jusqu'à atteindre `max_ticks`.
    ///
    /// Retourne le vecteur des unités déposées à la base par chaque robot.
    pub fn run(&self, n_collectors: usize, max_ticks: usize) -> Vec<u32> {
        let base = self.world.lock().unwrap().base;

        let handles: Vec<_> = (0..n_collectors)
            .map(|id| {
                let world = Arc::clone(&self.world);
                let kb = Arc::clone(&self.kb);

                thread::spawn(move || {
                    let mut collector = Collector::new(id, base);

                    for _ in 0..max_ticks {
                        // Ordre strict : world d'abord, kb ensuite.
                        let mut world = world.lock().unwrap();
                        let mut kb = kb.lock().unwrap();

                        if kb.is_empty() && matches!(collector.state, CollectorState::Idle) {
                            break;
                        }

                        collector.step(&mut world, &mut kb);
                    }

                    collector.deposited
                })
            })
            .collect();

        handles.into_iter().map(|h| h.join().unwrap()).collect()
    }

    /// Variante de [`run`] qui envoie un [`RobotSnapshot`] après chaque tick.
    ///
    /// P4 passe un `Sender` issu d'un `mpsc::channel()` et lit les snapshots
    /// dans sa boucle de rendu avec `rx.try_recv()`.
    ///
    /// Les erreurs d'envoi sont silencieuses : si P4 ferme le `Receiver`
    /// avant la fin de la simulation, les robots continuent simplement sans
    /// envoyer.
    ///
    /// ## Usage (P4)
    /// ```ignore
    /// let (tx, rx) = std::sync::mpsc::channel::<RobotSnapshot>();
    /// let sim_handle = {
    ///     let tx = tx.clone();
    ///     std::thread::spawn(move || sim.run_sending(3, 200_000, tx))
    /// };
    /// // boucle de rendu Ratatui :
    /// while let Ok(snap) = rx.try_recv() {
    ///     robot_states.insert(snap.id, snap);
    /// }
    /// ```
    pub fn run_sending(
        &self,
        n_collectors: usize,
        max_ticks: usize,
        tx: Sender<RobotSnapshot>,
    ) -> Vec<u32> {
        let base = self.world.lock().unwrap().base;

        let handles: Vec<_> = (0..n_collectors)
            .map(|id| {
                let world = Arc::clone(&self.world);
                let kb = Arc::clone(&self.kb);
                let tx = tx.clone();

                thread::spawn(move || {
                    let mut collector = Collector::new(id, base);

                    for _ in 0..max_ticks {
                        let mut world = world.lock().unwrap();
                        let mut kb = kb.lock().unwrap();

                        if kb.is_empty() && matches!(collector.state, CollectorState::Idle) {
                            break;
                        }

                        collector.step(&mut world, &mut kb);

                        let _ = tx.send(RobotSnapshot {
                            id: collector.id,
                            pos: collector.pos,
                            state_label: collector.state.label(),
                            deposited: collector.deposited,
                        });
                    }

                    collector.deposited
                })
            })
            .collect();

        handles.into_iter().map(|h| h.join().unwrap()).collect()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Position, Resource, ResourceKind, World};

    /// Monde minimal : pas d'obstacles, base au centre, ressources à la main.
    fn small_world(resources: &[(Position, ResourceKind, u32)]) -> World {
        let mut world = World::new(11, 11); // base en (5,5)
        for &(pos, kind, qty) in resources {
            world.resources.insert(pos, Resource::new(kind, qty));
        }
        world
    }

    #[test]
    fn un_collecteur_ramasse_une_ressource() {
        let pos = Position::new(5, 0); // 5 cases au-dessus de la base
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
        let collected: u32 = results.iter().sum();
        assert_eq!(collected, total, "Toutes les unités doivent être déposées");
    }

    #[test]
    fn trois_collecteurs_epuisent_plusieurs_gisements() {
        let resources = [
            (Position::new(0, 0), ResourceKind::Energy, 3),
            (Position::new(10, 0), ResourceKind::Crystal, 3),
            (Position::new(0, 10), ResourceKind::Energy, 3),
        ];
        let total: u32 = 9;
        let world = small_world(&resources);
        let sim = Simulation::new(world);
        sim.discover_all();

        let results = sim.run(3, 2000);
        let collected: u32 = results.iter().sum();
        assert_eq!(collected, total);
    }

    #[test]
    fn kb_vide_apres_simulation() {
        let pos = Position::new(3, 3);
        let world = small_world(&[(pos, ResourceKind::Crystal, 2)]);
        let sim = Simulation::new(world);
        sim.discover_all();
        sim.run(1, 1000);

        let kb = sim.kb.lock().unwrap();
        assert!(kb.is_empty(), "La KB doit être vide quand tout est collecté");
    }

    #[test]
    fn run_sending_emet_des_snapshots() {
        use std::sync::mpsc;
        use crate::comm::RobotSnapshot;
        use std::collections::HashMap;

        let pos = Position::new(5, 0);
        let world = small_world(&[(pos, ResourceKind::Energy, 2)]);
        let sim = Simulation::new(world);
        sim.discover_all();

        let (tx, rx) = mpsc::channel::<RobotSnapshot>();
        sim.run_sending(1, 1000, tx);

        // Collecte tous les snapshots et garde le dernier par robot.
        let mut latest: HashMap<usize, RobotSnapshot> = HashMap::new();
        while let Ok(snap) = rx.try_recv() {
            latest.insert(snap.id, snap);
        }

        assert!(!latest.is_empty(), "Au moins un snapshot doit avoir été reçu");
        assert_eq!(latest[&0].deposited, 2);
    }

    #[test]
    fn kb_handle_est_partageable_avec_p2() {
        let world = small_world(&[]);
        let sim = Simulation::new(world);
        let kb = sim.kb_handle();

        // Simule P2 qui rapporte une ressource depuis un autre thread.
        let kb2 = Arc::clone(&kb);
        thread::spawn(move || {
            use crate::types::{Resource, ResourceKind};
            kb2.lock()
                .unwrap()
                .report_resource(Position::new(1, 1), Resource::new(ResourceKind::Energy, 5));
        })
        .join()
        .unwrap();

        assert!(!kb.lock().unwrap().is_empty());
    }
}
