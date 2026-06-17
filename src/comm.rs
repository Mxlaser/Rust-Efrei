//! Communication entre threads — Personne 3.
//!
//! Ce module définit les types qui transitent par les channels `mpsc` entre
//! les threads de robots (P3) et l'interface graphique (P4).
//!
//! ## Flux de données
//!
//! ```text
//!  Thread collecteur ──(Sender<RobotSnapshot>)──► P4 render loop
//!  Thread éclaireur  ──(lock KB)──────────────► KnowledgeBase partagée
//! ```
//!
//! P4 utilise [`RobotSnapshot`] pour afficher les robots sans jamais
//! accéder aux locks — le channel est la seule frontière de synchronisation.

use crate::types::Position;

// ---------------------------------------------------------------------------
// Type de robot
// ---------------------------------------------------------------------------

/// Distingue les éclaireurs des collecteurs dans les snapshots.
///
/// P4 utilise ce champ pour choisir le symbole d'affichage :
/// `'x'` pour un éclaireur, `'o'` pour un collecteur.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RobotKind {
    Scout,
    Collector,
}

// ---------------------------------------------------------------------------
// Snapshot d'un robot (envoyé après chaque tick)
// ---------------------------------------------------------------------------

/// Instantané de l'état d'un robot, sérialisable par channel.
///
/// P4 reçoit un flux de ces structs via `mpsc::Receiver<RobotSnapshot>` et
/// maintient un `HashMap<usize, RobotSnapshot>` indexé par `id` pour rendre
/// la position de chaque robot à l'écran.
///
/// Les IDs sont uniques sur l'ensemble des robots (scouts + collecteurs) :
/// scouts → `0..n_scouts`, collecteurs → `n_scouts..n_scouts+n_collectors`.
#[derive(Debug, Clone)]
pub struct RobotSnapshot {
    /// Identifiant unique du robot (inchangé pendant toute la simulation).
    pub id: usize,
    /// Type du robot — détermine le symbole affiché par P4.
    pub kind: RobotKind,
    /// Position courante sur la grille.
    pub pos: Position,
    /// Étiquette de l'état courant : `"Idle"`, `"Moving"`, `"Collecting"`, `"Reporting"`, etc.
    pub state_label: &'static str,
    /// Unités déjà déposées à la base (toujours 0 pour un éclaireur).
    pub deposited: u32,
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc;

    #[test]
    fn snapshot_passe_par_channel() {
        let (tx, rx) = mpsc::channel::<RobotSnapshot>();

        let snap = RobotSnapshot {
            id: 0,
            kind: RobotKind::Collector,
            pos: Position::new(3, 4),
            state_label: "Moving",
            deposited: 7,
        };
        tx.send(snap.clone()).unwrap();

        let received = rx.recv().unwrap();
        assert_eq!(received.id, 0);
        assert_eq!(received.pos, snap.pos);
        assert_eq!(received.state_label, "Moving");
        assert_eq!(received.deposited, 7);
    }
}