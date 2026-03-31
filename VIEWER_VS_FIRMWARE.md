# Viewer vs Firmware — Comparaison moteur de rendu NAV

## Viewport

| | Viewer | Firmware |
|---|---|---|
| Taille | 768×768 px | 600×520 px (sprite PSRAM) |
| Tiles chargées | ~3×3 | ~3×3 |
| Tile size | 256×256 px | 256×256 px |

Les comptages de features du viewer (ex: 68k à Z7, 51k à Z8) sont **représentatifs** de ce que le firmware traite — même grille de tiles.

## Format des données

| | Viewer | Firmware |
|---|---|---|
| Format tile | NAV1 binaire | NAV1 binaire |
| Pack files | NPK3 (+ fallback fichiers individuels) | NPK3 (+ fallback fichiers individuels) |
| Coordonnées | VarInt + ZigZag | VarInt + ZigZag |
| Projection | Web Mercator | Web Mercator |
| Unités internes | 4096 par tile | 4096 par tile |
| Priority byte | nibble haut = min_zoom, bas = priority 0-15 | idem |

## Pipeline de rendu

| Pass | Viewer | Firmware |
|---|---|---|
| Tri | `features.sort(key=priority)` global | `globalLayers[16]` dispatch par priority |
| Pass 1 | Polygones + lignes non-bridge | Layers 0-15 (non-texte) |
| Pass 2 | Casings routiers (fond) | Abandonné (LovyanGFX ne permet pas casing propre) |
| Pass 3 | Cores routiers (avant) | — |
| Pass texte | Labels après toute la géométrie | `textRefs` pass séparée après globalLayers |

## Rendu géométrique

| | Viewer | Firmware |
|---|---|---|
| Polygones (remplissage) | `pygame.draw.polygon()` | `fillTriangle` / `fillRect` LovyanGFX |
| Polygones avec trous | Oui (alpha BLEND_RGBA_SUB) | Non implémenté |
| Outlines polygones | Seulement si `hasOutline` (fp[4] & 0x80) | Idem |
| Lignes larges | Rectangles orientés, caps carrés, pas de joints | `drawWideLine` LovyanGFX |
| Lignes fines (≤1px) | `pygame.draw.lines()` | `drawLine` LovyanGFX |
| Casings routiers | Pass 2/3 séparées | Abandonné |

## Labels texte

| | Viewer | Firmware |
|---|---|---|
| Collision detection | Non (inutile) | Code présent mais inutile |
| Pourquoi inutile | Labels filtrés par `resolve_text_labels()` dans le générateur | Idem |
| Font | `pygame.font.SysFont` | VLW Unicode chargée depuis SD en PSRAM |
| Tailles | 3 tailles fixes (15/17/20 px) | Configurable via fichier `.vlw` |
| Multilingue / accents | Limité (SysFont système) | Oui (VLW Unicode) |

## Dataset France-Sud — statistiques pack NPK3

| Zoom | Tiles total | Y range | Taille pack | Features/tile (moy.) | Features viewport (~9 tiles) |
|------|-------------|---------|-------------|----------------------|------------------------------|
| Z6  | 18          | 21-25   | 1.9 MB      | —                    | ~10k                         |
| Z7  | 38          | 44-50   | 6.3 MB      | ~7.5k                | ~68k                         |
| Z8  | 110         | 88-101  | 7.8 MB      | ~620                 | ~5.6k                        |
| Z9  | 355         | 177-202 | 20.8 MB     | ~5.7k                | ~51k                         |
| Z10 | 1 273       | 355-405 | 37.0 MB     | —                    | —                            |
| Z11 | 4 702       | 711-811 | 53.8 MB     | —                    | —                            |
| Z12 | 18 149      | 1423-1622 | 113.7 MB  | —                    | —                            |
| Z13 | 71 020      | 2847-3244 | 278.7 MB  | —                    | —                            |
| Z14 | 281 947     | 5695-6489 | 747.5 MB  | —                    | —                            |
| Z15 | 1 123 136   | 11390-12979 | 1133.9 MB | —                  | —                            |
| Z16 | 4 480 551   | 22781-25958 | 2128.3 MB | —                  | —                            |
| Z17 | 17 898 421  | 45562-51917 | 4275.9 MB | —                  | —                            |

**Z17 split FAT32 :**
- `Z17_0.nav` : 15 974 103 tiles, Y 45562-51206, **4096.3 MB** (juste sous la limite FAT32)
- `Z17_1.nav` :  1 924 318 tiles, Y 51207-51917, **179.6 MB**

## Limites par zoom

**Composition des features à Z7 (viewer) :** 68 497 features = 68 313 lignes + 99 polygones + 44 textes → **99.7% de segments de lignes.**

La simplification géométrique réduit les vertices par segment mais pas le nombre de segments (les intersections sont préservées). Ce n'est pas la densité visuelle mais le volume de segments qui pose problème.

**Réévaluation des zooms raster/NAV :**

| Zoom | Features viewport | Firmware actuel | À réévaluer |
|------|-------------------|-----------------|-------------|
| Z6  | ~10k              | Raster          | Probablement NAV faisable |
| Z7  | ~68k              | Raster forcé    | À tester sur matériel (comparable à Z9) |
| Z8  | ~5.6k             | Raster forcé    | **Probablement NAV faisable** (moins que Z9) |
| Z9  | ~51k              | NAV vectoriel ✓ | Référence |

Z8 avec ~5.6k features/viewport est bien en dessous de Z9 (51k) qui fonctionne bien. La décision de forcer raster pour Z6-Z8 mérite d'être revisitée sur le matériel.

## Code à nettoyer (firmware)

- `map_engine.cpp` lignes ~1724-1743 et ~1807-1814 : collision detection labels — code mort, à supprimer.
