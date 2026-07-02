use crate::types::Position;

// ---------------------------------------------------------------------------
// Type de robot
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RobotKind {
    Scout,
    Collector,
}

// ---------------------------------------------------------------------------
// Snapshot d'un robot (envoyé après chaque tick)
// ---------------------------------------------------------------------------

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
