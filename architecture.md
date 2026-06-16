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

### Contrat P2 ↔ P3

- **P2 décide** : `scout_decide` (et plus tard `collector_decide`) renvoient une
  `Action` à partir d'une vue en lecture du monde. Aucune mutation, aucune
  concurrence.
- **P3 applique** : le moteur de threads consomme chaque `Action` et fait
  évoluer le `World` partagé (déplacement, décrément de ressource, dépôt à la
  base, diffusion des `Report` aux collecteurs…).

Cette séparation garantit que toute la logique des robots est testable de façon
unitaire et déterministe, indépendamment de la couche concurrente.
