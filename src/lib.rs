//! Bibliothèque de la simulation de collecte de ressources.
//!
//! Les modules sont exposés ici pour que tous les membres du groupe puissent
//! les réutiliser depuis le binaire (`main.rs`) comme depuis les tests.
//!
//! Découpage prévu :
//!   * [`types`] — structures partagées (Personne 1) ;
//!   * [`map`]   — génération procédurale de la carte (Personne 1) ;
//!   * `robots`  — comportements éclaireur/collecteur (Personne 2, à venir) ;
//!   * `comm`    — channels & threads (Personne 3, à venir) ;
//!   * `ui`      — rendu Ratatui (Personne 4, à venir).

pub mod map;
pub mod robots;
pub mod types;
