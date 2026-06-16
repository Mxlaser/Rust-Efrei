//! Robots collecteurs — Personne 3.
//!
//! Chaque [`Collector`] est un thread indépendant qui lit la [`KnowledgeBase`]
//! (ressources découvertes par les éclaireurs) et collecte des unités, une par
//! tick, avant de revenir à la base.
//!
//! En phase concurrente le collecteur reçoit un `Arc<Mutex<KnowledgeBase>>`
//! et un `Arc<Mutex<World>>`. Ici on expose la logique pure (sans threads)
//! pour pouvoir la tester unitairement.

use std::collections::{HashMap, VecDeque};

use crate::types::{KnowledgeBase, Position, World};

// ---------------------------------------------------------------------------
// Machine à états
// ---------------------------------------------------------------------------

/// État interne d'un robot collecteur.
#[derive(Debug, Clone, PartialEq)]
pub enum CollectorState {
    /// Aucune cible — le robot attend une ressource dans la KnowledgeBase.
    Idle,
    /// Le robot suit le chemin `path` vers la ressource en `target`.
    MovingToTarget { target: Position, path: VecDeque<Position> },
    /// Le robot est sur la case ressource et collecte une unité par tick.
    Collecting { target: Position },
    /// Le robot rentre à la base avec `units` unités collectées.
    ReturningToBase { units: u32, path: VecDeque<Position> },
}

impl CollectorState {
    /// Étiquette courte pour l'affichage (P4) et les snapshots (comm).
    pub fn label(&self) -> &'static str {
        match self {
            CollectorState::Idle => "Idle",
            CollectorState::MovingToTarget { .. } => "Moving",
            CollectorState::Collecting { .. } => "Collecting",
            CollectorState::ReturningToBase { .. } => "Returning",
        }
    }
}

// ---------------------------------------------------------------------------
// Collecteur
// ---------------------------------------------------------------------------

/// Un robot collecteur.
#[derive(Debug)]
pub struct Collector {
    pub id: usize,
    pub pos: Position,
    /// Unités déposées à la base (inventaire définitif).
    pub deposited: u32,
    pub state: CollectorState,
}

impl Collector {
    pub fn new(id: usize, start: Position) -> Self {
        Collector {
            id,
            pos: start,
            deposited: 0,
            state: CollectorState::Idle,
        }
    }

    /// Avance d'un tick.
    ///
    /// `world` est utilisé pour le pathfinding (obstacles).
    /// `kb` est la seule source de vérité sur les ressources visibles.
    ///
    /// Retourne `true` si quelque chose a changé (utile pour le rendu).
    pub fn step(&mut self, world: &mut World, kb: &mut KnowledgeBase) -> bool {
        match self.state.clone() {
            CollectorState::Idle => self.tick_idle(world, kb),
            CollectorState::MovingToTarget { target, path } => {
                self.tick_moving(target, path, world, kb)
            }
            CollectorState::Collecting { target } => self.tick_collecting(target, world, kb),
            CollectorState::ReturningToBase { units, path } => {
                self.tick_returning(units, path, world)
            }
        }
    }

    // -----------------------------------------------------------------------
    // Transitions
    // -----------------------------------------------------------------------

    fn tick_idle(&mut self, _world: &World, kb: &KnowledgeBase) -> bool {
        let Some(target) = nearest_resource(self.pos, kb) else {
            return false; // rien à faire
        };
        let Some(path) = bfs(self.pos, target, _world) else {
            return false; // cible inatteignable
        };
        self.state = CollectorState::MovingToTarget {
            target,
            path: path.into(),
        };
        true
    }

    fn tick_moving(
        &mut self,
        target: Position,
        mut path: VecDeque<Position>,
        world: &World,
        kb: &KnowledgeBase,
    ) -> bool {
        // Si la ressource a disparu de la KB (épuisée par un autre robot) on abandonne.
        if !kb.known_resources().contains_key(&target) {
            self.state = CollectorState::Idle;
            return true;
        }
        if let Some(next) = path.pop_front() {
            self.pos = next;
            if self.pos == target {
                self.state = CollectorState::Collecting { target };
            } else {
                self.state = CollectorState::MovingToTarget { target, path };
            }
        } else {
            // Chemin vide mais pas encore arrivé : recalcule.
            self.state = CollectorState::Idle;
        }
        true
    }

    fn tick_collecting(
        &mut self,
        target: Position,
        world: &mut World,
        kb: &mut KnowledgeBase,
    ) -> bool {
        // On retire une unité de la vérité-terrain.
        if let Some(resource) = world.resources.get_mut(&target) {
            if resource.take_one() {
                if resource.is_depleted() {
                    world.resources.remove(&target);
                    kb.remove(&target);
                } else {
                    // Met à jour la quantité connue dans la KB.
                    kb.report_resource(target, *world.resources.get(&target).unwrap());
                }
                // Rentre à la base.
                let path = bfs(self.pos, world.base, world).unwrap_or_default();
                self.state = CollectorState::ReturningToBase {
                    units: 1,
                    path: path.into(),
                };
                return true;
            }
        }
        // Ressource absente ou épuisée — nettoyage et retour en Idle.
        kb.remove(&target);
        self.state = CollectorState::Idle;
        true
    }

    fn tick_returning(&mut self, units: u32, mut path: VecDeque<Position>, world: &World) -> bool {
        if let Some(next) = path.pop_front() {
            self.pos = next;
            if self.pos == world.base {
                self.deposited += units;
                self.state = CollectorState::Idle;
            } else {
                self.state = CollectorState::ReturningToBase { units, path };
            }
        } else {
            // Déjà à la base.
            self.deposited += units;
            self.state = CollectorState::Idle;
        }
        true
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Choisit la ressource connue la plus proche (distance de Manhattan).
fn nearest_resource(from: Position, kb: &KnowledgeBase) -> Option<Position> {
    kb.known_resources()
        .keys()
        .min_by_key(|&&pos| from.manhattan_distance(&pos))
        .copied()
}

/// BFS — renvoie le chemin (sans la case de départ, avec l'arrivée).
/// Renvoie `None` si `goal` est inaccessible.
pub fn bfs(start: Position, goal: Position, world: &World) -> Option<Vec<Position>> {
    if start == goal {
        return Some(vec![]);
    }

    let mut queue: VecDeque<Position> = VecDeque::new();
    let mut came_from: HashMap<Position, Position> = HashMap::new();

    queue.push_back(start);
    came_from.insert(start, start);

    while let Some(current) = queue.pop_front() {
        for neighbor in world.walkable_neighbors(current) {
            if came_from.contains_key(&neighbor) {
                continue;
            }
            came_from.insert(neighbor, current);
            if neighbor == goal {
                return Some(reconstruct(start, goal, &came_from));
            }
            queue.push_back(neighbor);
        }
    }
    None
}

fn reconstruct(
    start: Position,
    goal: Position,
    came_from: &HashMap<Position, Position>,
) -> Vec<Position> {
    let mut path = Vec::new();
    let mut current = goal;
    while current != start {
        path.push(current);
        current = came_from[&current];
    }
    path.reverse();
    path
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{KnowledgeBase, Resource, ResourceKind, World};

    fn simple_world() -> World {
        // 5x5, aucun obstacle, base au centre (2,2).
        World::new(5, 5)
    }

    fn world_with_resource(pos: Position) -> (World, KnowledgeBase) {
        let mut world = simple_world();
        let resource = Resource::new(ResourceKind::Energy, 3);
        world.resources.insert(pos, resource);
        let mut kb = KnowledgeBase::new();
        kb.report_resource(pos, resource);
        (world, kb)
    }

    #[test]
    fn idle_sans_kb_ne_bouge_pas() {
        let world = simple_world();
        let mut kb = KnowledgeBase::new();
        let mut robot = Collector::new(0, world.base);
        let changed = robot.step(&mut { world }, &mut kb);
        assert!(!changed);
        assert_eq!(robot.state, CollectorState::Idle);
    }

    #[test]
    fn idle_avec_ressource_cible() {
        let res_pos = Position::new(0, 0);
        let (mut world, mut kb) = world_with_resource(res_pos);
        let mut robot = Collector::new(0, world.base);
        robot.step(&mut world, &mut kb);
        assert!(matches!(
            robot.state,
            CollectorState::MovingToTarget { target, .. } if target == res_pos
        ));
    }

    #[test]
    fn collecte_complete_un_cycle() {
        let res_pos = Position::new(2, 0); // même colonne que la base, juste au-dessus
        let (mut world, mut kb) = world_with_resource(res_pos);
        let mut robot = Collector::new(0, world.base);

        // On fait tourner jusqu'à ce que le robot dépose (ou 50 ticks max).
        for _ in 0..50 {
            robot.step(&mut world, &mut kb);
            if robot.deposited > 0 {
                break;
            }
        }
        assert!(robot.deposited > 0, "Le robot n'a rien déposé en 50 ticks");
    }

    #[test]
    fn bfs_trouve_chemin_simple() {
        let world = simple_world();
        let path = bfs(Position::new(0, 0), Position::new(2, 2), &world);
        assert!(path.is_some());
        let path = path.unwrap();
        assert_eq!(*path.last().unwrap(), Position::new(2, 2));
    }

    #[test]
    fn bfs_meme_case_chemin_vide() {
        let world = simple_world();
        let path = bfs(Position::new(1, 1), Position::new(1, 1), &world);
        assert_eq!(path, Some(vec![]));
    }

    #[test]
    fn ressource_epuisee_retiree_de_la_kb() {
        let res_pos = Position::new(2, 0);
        let mut world = simple_world();
        // Quantité = 1 : épuisée après une seule collecte.
        let resource = Resource::new(ResourceKind::Energy, 1);
        world.resources.insert(res_pos, resource);
        let mut kb = KnowledgeBase::new();
        kb.report_resource(res_pos, resource);

        let mut robot = Collector::new(0, world.base);
        for _ in 0..50 {
            robot.step(&mut world, &mut kb);
            if robot.deposited > 0 { break; }
        }

        assert!(kb.is_empty(), "La KB doit être vide après épuisement");
        assert!(world.resources.get(&res_pos).is_none());
    }
}
