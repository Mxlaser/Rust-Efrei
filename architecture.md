# Architecture — Simulation de collecte de ressources

Document vivant décrivant le découpage du crate `resource_collection_sim` et les
contrats entre membres du groupe.

## Vue d'ensemble des modules

| Module   | Responsable | Rôle |
|----------|-------------|------|
| `types`  | Personne 1  | Structures partagées (`Position`, `World`, `Resource`, `Tile`, …) |
| `map`    | Personne 1  | Génération procédurale de la carte (obstacles, ressources) |
| `robots` | Personne 2  | Comportements des robots (décision **pure**) |
| `comm`   | Personne 3  | Threads, channels, application des actions (à venir) |
| `ui`     | Personne 4  | Rendu terminal Ratatui (à venir) |

## Module `robots` (Personne 2)

### Principe directeur : décider ≠ appliquer

Les comportements sont de la **logique pure**. Une fonction `*_decide` :

- reçoit l'**état du robot** (mutable, p. ex. pour faire avancer son PRNG) et
  une **vue en lecture** du monde (`&World`) ;
- renvoie une **intention** (`Action`) ;
- ne crée **aucun** thread, ne prend **aucun** lock, n'envoie sur **aucun**
  channel, ne fait **aucun** `sleep`, et **ne mute jamais** le monde.

C'est la **Personne 3** qui exécutera ces décisions dans des threads et
**appliquera** réellement les `Action` sur le monde partagé
(`Arc<Mutex<World>>`). Les fonctions de décision manipulent des **données
brutes** (`&World`, et plus tard `&HashMap<Position, Resource>`), jamais le type
`KnowledgeBase` qui appartient à P3.

### `enum Action`

Vocabulaire **complet et figé** partagé entre la décision (P2) et son
application (P3). P3 fera un `match` exhaustif dessus. Certaines variantes ne
servent qu'au collecteur (Sprint 2) ; elles sont déjà présentes pour stabiliser
le contrat.

```rust
pub enum Action {
    MoveTo(Position),            // se déplacer vers une case adjacente traversable
    Collect(Position),           // collecteur : ramasser 1 unité ici (Sprint 2)
    Unload,                      // collecteur : décharger à la base (Sprint 2)
    Report(Position, Resource),  // éclaireur : signaler une ressource vue
    Idle,                        // rien à faire ce tick
}
```

### `struct ScoutState`

État propre à un robot **éclaireur** :

- `pos: Position` — position courante ;
- `rng: StdRng` — générateur aléatoire **personnel** (champ privé).

Constructeur : `ScoutState::new(start: Position, seed: u64) -> Self`. La graine
rend les décisions **déterministes**, donc testables.

### `fn scout_decide`

```rust
pub fn scout_decide(state: &mut ScoutState, world: &World) -> Action
```

**Sémantique** (logique pure, seul effet de bord : avancée du PRNG de `state`) :

1. Si `world.resource_at(state.pos)` renvoie une ressource → `Action::Report(pos, resource)`.
2. Sinon, tirer au hasard une case parmi `world.walkable_neighbors(state.pos)` →
   `Action::MoveTo(case)` (toujours **adjacente** et **traversable**).
3. Si aucun voisin traversable (robot encerclé) → `Action::Idle`.

`scout_decide` **ne déplace pas** le robot : il renvoie seulement l'intention.
Le déplacement effectif est de la responsabilité de P3 lors de l'application de
l'`Action`. (Un helper `apply`, compilé uniquement en test, simule cette
application pour vérifier la reproductibilité d'une séquence sans dépendre de
P3.)

### `struct CollectorState` (Sprint P2-2)

État d'un robot **collecteur** :

- `pos: Position` — position courante ;
- `carrying: u32` — quantité transportée ;
- `capacity: u32` — capacité `K` avant retour décharge (P3 passe `10`) ;
- `target: Option<Position>` — gisement visé, **planification privée**.

Constructeur : `CollectorState::new(start: Position, capacity: u32) -> Self`
(`carrying = 0`, `target = None`).

**Décisions de design** : le collecteur accumule jusqu'à `capacity` avant de
rentrer (le « 1 unité à la fois » est le *rythme*, 1/tick) ; il ne lit **jamais**
`world.resources` (vérité terrain = triche) — les gisements connus lui arrivent
en données brutes via `&HashMap<Position, Resource>`. Il ne lit le monde que pour
le terrain (cases traversables) et la base.

### `fn next_step_toward` — pathfinding BFS

```rust
fn next_step_toward(world: &World, from: Position, goal: Position) -> Option<Position>
```

BFS classique (file FIFO + ensemble visité + `came_from`), expansion via
`world.walkable_neighbors` (route donc sur les obstacles **vérité terrain** ; pas
de fog-of-war obstacles pour ce sprint). Renvoie la **première case** à franchir
depuis `from` vers `goal` (pas le chemin complet), ou `None` si `goal` est
inatteignable. L'ordre fixe des voisins rend le chemin **déterministe**.

### `fn collector_decide`

```rust
pub fn collector_decide(
    state: &mut CollectorState,
    world: &World,
    known: &HashMap<Position, Resource>,
) -> Action
```

**Sémantique** (logique pure ; ne mute **que** `state.target`) :

1. **Plein** (`carrying >= capacity`) : à la base → `Unload` ; sinon
   `MoveTo(next)` (BFS vers la base), ou `Idle` défensif si base injoignable.
2. **Collecte** (`carrying < capacity`) :
   - a. valider/choisir la cible : si `target` est `None` ou absente de `known`,
     prendre le gisement minimisant `(manhattan(pos, p), p.x, p.y)` — le tie-break
     par coordonnées rend le choix **déterministe** (pas d'oscillation malgré
     l'ordre non garanti de la `HashMap`) ;
   - b. cible atteinte (`pos == cible`) → `Collect(cible)` ; sinon `MoveTo(next)`
     (BFS) ; cible murée (BFS `None`) → on l'abandonne (`target = None`) ;
   - c. aucune cible valide/atteignable : `carrying > 0` → repartir décharger
     (comme phase 1) ; sinon → `Idle` (attendre les éclaireurs).

`collector_decide` ne touche **jamais** `carrying` ni le monde (seul `target` est
écrit). Les helpers `apply` / `apply_collector` (compilés en test) simulent P3
pour dérouler des cycles complets sans dépendre de son code.

### Contrat P2 ↔ P3

- **P2 décide** : `scout_decide` et `collector_decide` renvoient une `Action` à
  partir d'une vue en lecture du monde + données brutes. Aucune mutation, aucune
  concurrence. Le collecteur ne planifie que `target`.
- **P3 applique** : le moteur de threads consomme chaque `Action` et fait évoluer
  le `World` partagé ainsi que l'état des robots :
  - `MoveTo(p)` → met à jour `pos` ;
  - `Collect(p)` → appelle `take_one()` sur la **vraie** ressource de
    `world.resources` ; n'incrémente `carrying` **que** si la prise réussit ;
    retire le gisement de la base de connaissances (KB) s'il est épuisé.
    **Cas de course** : si la ressource a déjà disparu de `world.resources`
    (collectée par un autre robot entre-temps), traiter comme épuisée — ne pas
    incrémenter, ne pas planter ;
  - `Unload` → dépose la cargaison à la base, `carrying = 0` ;
  - `Report(p, r)` → insère/rafraîchit le gisement dans la KB partagée aux
    collecteurs.

Cette séparation garantit que toute la logique des robots est testable de façon
unitaire et déterministe, indépendamment de la couche concurrente.
