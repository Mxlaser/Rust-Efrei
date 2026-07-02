use crate::types::{Position, Resource, World};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use std::collections::{HashMap, HashSet, VecDeque};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    /// Se déplacer vers une case **adjacente** et traversable.
    MoveTo(Position),
    /// Collecteur : ramasser une unité de ressource sur la case courante.
    Collect(Position),
    /// Collecteur : décharger sa cargaison une fois revenu à la base.
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

pub struct CollectorState {
    /// Position courante du robot (mise à jour par P3 sur `MoveTo`).
    pub pos: Position,
    /// Quantité transportée (mise à jour par P3 : +1 sur `Collect`, 0 sur `Unload`).
    pub carrying: u32,
    /// Capacité maximale `K` avant de rentrer décharger.
    pub capacity: u32,
    /// Gisement connu actuellement visé — planification **privée**.
    target: Option<Position>,
}

impl CollectorState {
    /// Crée un collecteur à la position `start`, vide, de capacité `capacity`.
    pub fn new(start: Position, capacity: u32) -> Self {
        CollectorState {
            pos: start,
            carrying: 0,
            capacity,
            target: None,
        }
    }
}

/// Renvoie la **première case** à franchir depuis `from` pour rejoindre `goal`
/// par le plus court chemin, ou `None` si `goal` est inatteignable.
///
/// BFS classique (file FIFO + ensemble visité + `came_from`), expansion via
/// [`World::walkable_neighbors`] : il route donc sur les obstacles **vérité
/// terrain**. L'ordre des voisins étant fixe, le chemin trouvé est déterministe.
///
/// Pré-condition : `from != goal` (l'appelant teste l'arrivée avant). Le but
/// (case d'une ressource = `Tile::Empty`, ou base = `Tile::Base`) est supposé
/// traversable, donc atteignable par `walkable_neighbors`.
fn next_step_toward(world: &World, from: Position, goal: Position) -> Option<Position> {
    if from == goal {
        return None;
    }

    let mut visited: HashSet<Position> = HashSet::new();
    let mut came_from: HashMap<Position, Position> = HashMap::new();
    let mut queue: VecDeque<Position> = VecDeque::new();
    visited.insert(from);
    queue.push_back(from);

    while let Some(current) = queue.pop_front() {
        if current == goal {
            // Remonte la chaîne des prédécesseurs jusqu'à la case juste après
            // `from` : c'est le premier pas à franchir.
            let mut step = goal;
            while came_from[&step] != from {
                step = came_from[&step];
            }
            return Some(step);
        }
        for next in world.walkable_neighbors(current) {
            if visited.insert(next) {
                came_from.insert(next, current);
                queue.push_back(next);
            }
        }
    }

    None
}

/// Choisit, parmi les gisements connus, celui qui minimise
/// `(distance_manhattan(from, p), p.x, p.y)`.
///
/// Le tie-break par coordonnées rend le choix **déterministe** malgré l'ordre
/// non garanti d'une `HashMap`, ce qui évite toute oscillation de cible.
fn choose_target(known: &HashMap<Position, Resource>, from: Position) -> Option<Position> {
    known
        .keys()
        .copied()
        .min_by_key(|p| (from.manhattan_distance(p), p.x, p.y))
}

/// Achemine le collecteur vers la base pour décharger.
///
/// À la base → [`Action::Unload`] ; sinon un pas de BFS vers la base ; et, par
/// défense (base murée — ne devrait pas arriver), [`Action::Idle`].
fn go_unload(state: &CollectorState, world: &World) -> Action {
    if state.pos == world.base {
        Action::Unload
    } else {
        match next_step_toward(world, state.pos, world.base) {
            Some(next) => Action::MoveTo(next),
            None => Action::Idle,
        }
    }
}

pub fn collector_decide(
    state: &mut CollectorState,
    world: &World,
    known: &HashMap<Position, Resource>,
) -> Action {
    // Phase 1 — plein : on rentre décharger.
    if state.carrying >= state.capacity {
        return go_unload(state, world);
    }

    // Phase 2a — valider la cible courante, en choisir une sinon.
    let target_still_known = matches!(state.target, Some(t) if known.contains_key(&t));
    if !target_still_known {
        state.target = choose_target(known, state.pos);
    }

    // Phase 2b — viser la cible retenue.
    if let Some(target) = state.target {
        if state.pos == target {
            return Action::Collect(target);
        }
        match next_step_toward(world, state.pos, target) {
            Some(next) => return Action::MoveTo(next),
            // Cible inatteignable (murée) : on l'abandonne, on retombe en 2c.
            None => state.target = None,
        }
    }

    // Phase 2c — rien à viser : décharger si l'on porte, sinon patienter.
    if state.carrying > 0 {
        go_unload(state, world)
    } else {
        Action::Idle
    }
}

#[cfg(test)]
fn apply(state: &mut ScoutState, action: Action) {
    if let Action::MoveTo(next) = action {
        state.pos = next;
    }
}

#[cfg(test)]
fn apply_collector(state: &mut CollectorState, action: Action) {
    match action {
        Action::MoveTo(next) => state.pos = next,
        Action::Collect(_) => state.carrying = (state.carrying + 1).min(state.capacity),
        Action::Unload => state.carrying = 0,
        _ => {}
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

    /// Construit une table de gisements connus à partir de couples
    /// `(position, quantité)` (toujours du cristal, le `kind` n'importe pas ici).
    fn known_from(pairs: &[(Position, u32)]) -> HashMap<Position, Resource> {
        pairs
            .iter()
            .map(|&(p, q)| (p, Resource::new(ResourceKind::Crystal, q)))
            .collect()
    }

    /// Chemin libre vers une cible proche : chaque décision est un `MoveTo` qui
    /// rapproche, puis le collecteur ATTEINT la cible et renvoie `Collect`.
    #[test]
    fn seek_puis_collect() {
        let world = World::new(7, 7);
        let target = Position::new(5, 1);
        let known = known_from(&[(target, 4)]);

        let mut col = CollectorState::new(Position::new(1, 1), 10);
        let mut last = Action::Idle;
        for _ in 0..30 {
            last = collector_decide(&mut col, &world, &known);
            if last == Action::Collect(target) {
                break;
            }
            match last {
                Action::MoveTo(next) => {
                    assert!(world.is_walkable(next));
                    assert_eq!(col.pos.manhattan_distance(&next), 1);
                }
                other => panic!("attendu MoveTo en chemin, obtenu {other:?}"),
            }
            apply_collector(&mut col, last);
        }
        assert_eq!(last, Action::Collect(target));
        assert_eq!(col.pos, target);
    }

    /// Déjà sur la cible → `Collect(cible)` immédiatement.
    #[test]
    fn collect_sur_la_cible() {
        let world = World::new(5, 5);
        let target = Position::new(2, 2);
        let known = known_from(&[(target, 1)]);

        let mut col = CollectorState::new(target, 10);
        assert_eq!(
            collector_decide(&mut col, &world, &known),
            Action::Collect(target)
        );
    }

    /// `carrying == capacity` → retour à la base, puis `Unload` une fois arrivé.
    #[test]
    fn plein_rentre_et_decharge() {
        let world = World::new(7, 7);
        let known = known_from(&[(Position::new(0, 0), 5)]);

        let mut col = CollectorState::new(Position::new(1, 1), 3);
        col.carrying = 3;

        let mut last = Action::Idle;
        for _ in 0..40 {
            last = collector_decide(&mut col, &world, &known);
            if last == Action::Unload {
                break;
            }
            match last {
                Action::MoveTo(next) => assert!(world.is_walkable(next)),
                other => panic!("attendu MoveTo vers la base, obtenu {other:?}"),
            }
            apply_collector(&mut col, last);
        }
        assert_eq!(last, Action::Unload);
        assert_eq!(col.pos, world.base);
    }

    /// Un mur entre le collecteur et la cible : le BFS contourne. Chaque pas est
    /// traversable et la suite atteint bien la cible.
    #[test]
    fn contourne_le_mur() {
        let mut world = World::new(5, 5);
        // Mur vertical en x=2 sur y = 0..=3, laissant un passage en (2, 4).
        for y in 0..4 {
            world.set_tile(Position::new(2, y), Tile::Obstacle);
        }
        let target = Position::new(4, 0);
        let known = known_from(&[(target, 2)]);

        let mut col = CollectorState::new(Position::new(0, 0), 10);
        let mut reached = false;
        for _ in 0..40 {
            let action = collector_decide(&mut col, &world, &known);
            if action == Action::Collect(target) {
                reached = true;
                break;
            }
            match action {
                Action::MoveTo(next) => {
                    assert!(world.is_walkable(next), "le BFS ne traverse pas un mur");
                    assert_eq!(col.pos.manhattan_distance(&next), 1);
                }
                other => panic!("attendu MoveTo/Collect, obtenu {other:?}"),
            }
            apply_collector(&mut col, action);
        }
        assert!(
            reached,
            "le collecteur doit atteindre la cible en contournant"
        );
        assert_eq!(col.pos, target);
    }

    /// Cible périmée : on la retire de `known` → re-ciblage, jamais de `Collect`
    /// sur la ressource disparue.
    #[test]
    fn re_cible_si_perimee() {
        let world = World::new(7, 7);
        let stale = Position::new(6, 6);
        let fresh = Position::new(1, 0);

        // Premier tick : la cible la plus proche est verrouillée.
        let mut col = CollectorState::new(Position::new(0, 0), 10);
        let known_before = known_from(&[(fresh, 2)]);
        let first = collector_decide(&mut col, &world, &known_before);
        assert!(matches!(first, Action::MoveTo(_) | Action::Collect(_)));

        // La ressource `fresh` disparaît, seule `stale` reste connue.
        let known_after = known_from(&[(stale, 2)]);
        let mut saw_collect_on_fresh = false;
        for _ in 0..40 {
            let action = collector_decide(&mut col, &world, &known_after);
            if action == Action::Collect(fresh) {
                saw_collect_on_fresh = true;
            }
            if action == Action::Collect(stale) {
                break;
            }
            apply_collector(&mut col, action);
        }
        assert!(
            !saw_collect_on_fresh,
            "ne doit pas collecter une cible disparue"
        );
        assert_eq!(
            col.target,
            Some(stale),
            "doit s'être re-ciblé sur le gisement restant"
        );
    }

    /// `known` vide et rien en soute → `Idle` (on attend les éclaireurs).
    #[test]
    fn idle_si_rien_a_faire() {
        let world = World::new(5, 5);
        let known: HashMap<Position, Resource> = HashMap::new();

        let mut col = CollectorState::new(Position::new(1, 1), 10);
        assert_eq!(collector_decide(&mut col, &world, &known), Action::Idle);
    }

    /// `known` vide mais soute non vide → repart décharger à la base.
    #[test]
    fn retour_decharge_si_porte_sans_cible() {
        let world = World::new(7, 7);
        let known: HashMap<Position, Resource> = HashMap::new();

        let mut col = CollectorState::new(Position::new(1, 1), 10);
        col.carrying = 2; // porte sans cible connue
        match collector_decide(&mut col, &world, &known) {
            Action::MoveTo(next) => {
                assert!(world.is_walkable(next));
                // Le pas rapproche de la base.
                assert!(
                    next.manhattan_distance(&world.base) < col.pos.manhattan_distance(&world.base)
                );
            }
            other => panic!("attendu MoveTo vers la base, obtenu {other:?}"),
        }
    }

    /// Deux ressources équidistantes → cible déterministe (tie-break stable),
    /// reproductible sur plusieurs exécutions.
    #[test]
    fn cible_deterministe_si_equidistante() {
        let world = World::new(7, 7);
        let start = Position::new(3, 3);
        // (1,3) et (5,3) sont toutes deux à distance 2 de (3,3).
        let known = known_from(&[(Position::new(5, 3), 1), (Position::new(1, 3), 1)]);

        let decide_once = || {
            let mut col = CollectorState::new(start, 10);
            let action = collector_decide(&mut col, &world, &known);
            (action, col.target)
        };
        let first = decide_once();
        for _ in 0..20 {
            assert_eq!(decide_once(), first, "le choix de cible doit être stable");
        }
        // tie-break (dist, x, y) ⇒ (1,3) gagne sur (5,3).
        assert_eq!(first.1, Some(Position::new(1, 3)));
    }

    /// Pureté : `collector_decide` ne mute jamais le monde.
    #[test]
    fn collecteur_ne_mute_pas_le_monde() {
        let mut world = World::new(6, 6);
        world
            .resources
            .insert(Position::new(0, 0), Resource::new(ResourceKind::Energy, 9));
        let before = world.clone();
        let known = known_from(&[(Position::new(4, 4), 3)]);

        let mut col = CollectorState::new(Position::new(1, 1), 10);
        let _ = collector_decide(&mut col, &world, &known);

        assert_eq!(world.tiles(), before.tiles());
        assert_eq!(world.resources, before.resources);
    }
}
