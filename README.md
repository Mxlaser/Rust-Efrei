# Simulation de collecte de ressources

**Carte & structures de données** : la fondation du projet. Ce module fournit
les types partagés et la génération procédurale de la carte (bruit de Perlin)
sur lesquels s'appuieront les Personnes 2, 3 et 4.

## Lancer la démo

```bash
cargo run            # carte par défaut (graine 42)
cargo run -- 123     # carte avec la graine 123
cargo test           # lance les tests unitaires
```

La démo affiche la carte en ASCII (`O` obstacle, `E` énergie, `C` cristal,
`#` base, `.` libre) puis un récapitulatif chiffré. Pas encore de Ratatui ni
de threads : c'est volontaire (version mono-thread vérifiable d'abord).

## Structure des fichiers

| Fichier        | Rôle                                                        |
|----------------|-------------------------------------------------------------|
| `src/types.rs` | Types partagés : `Position`, `Tile`, `ResourceKind`, `Resource`, `World` |
| `src/map.rs`   | Génération Perlin + placement base/ressources (`MapConfig`, `generate`) |
| `src/main.rs`  | Démo ASCII (sera remplacée par l'UI Ratatui de la Personne 4) |
| `src/lib.rs`   | Expose les modules à tout le groupe                         |

## API pour les autres membres

```rust
use resource_collection_sim::map::{self, MapConfig};
use resource_collection_sim::types::{Position, Tile, Resource, ResourceKind, World};

let world = map::generate(MapConfig::default());

world.base;                        // Position de la base (point de départ des robots)
world.width; world.height;         // dimensions
world.tile(pos);                   // Option<Tile> (None si hors limites)
world.is_walkable(pos);            // bool : dans la carte ET non-obstacle
world.is_obstacle(pos);            // bool
world.resource_at(pos);            // Option<&Resource>
world.walkable_neighbors(pos);     // Vec<Position> (utile au pathfinding, Personne 2)
world.resources;                   // HashMap<Position, Resource>

pos.manhattan_distance(&other);    // heuristique pour A* (Personne 2)
pos.neighbors(width, height);      // voisins 4-connexes dans les limites
```

### Notes de conception

- Les **ressources ne sont pas dans la grille de tuiles** mais dans
  `world.resources : HashMap<Position, Resource>`, car elles ont une quantité
  et peuvent s'épuiser. Une case avec ressource a la tuile `Tile::Empty`.
- `Resource::take_one()` retire une unité (renvoie `false` si épuisé) —
  prévu pour les collecteurs (Personne 2/3).
- La génération est **déterministe pour une graine donnée** (debug facile).
- La zone autour de la base est toujours dégagée (les robots peuvent sortir).

### Intégration concurrente (Personne 3)

`World` est `Clone + Debug` et sans état caché : il s'encapsule directement
dans un `Arc<Mutex<World>>`. Modules à ajouter ensuite : `robots`, `comm`, `ui`.

## Dépendances

`noise = "0.9"` (Perlin) et `rand = "0.8"`. `ratatui` / `crossterm` seront
ajoutés par la Personne 4.
