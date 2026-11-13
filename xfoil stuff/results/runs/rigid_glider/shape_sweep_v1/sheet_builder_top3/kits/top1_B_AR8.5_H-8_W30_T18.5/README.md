# top1_B_AR8.5_H-8_W30_T18.5 - sheet-builder kit

Generated 2026-10-21T18:33:12 by `rigid_glider/sheet_builder.py` from `top1_B_AR8.5_H-8_W30_T18.5.json` (builtin outline: svg). Nothing here is flight-validated; every number is tagged MODEL / ASSUMED / CALCULATED / GEOMETRY / KAI.

## What is in the folder

- `parts/*.stl` - print files (already rotated onto the bed; 250 mm bed). `dxf/*.dxf` - 2D outlines in mm for an Onshape sketch import. `foam/*.svg` - 1:1 white print templates. `summary.json`, `index.html` (dark previews).

## Settings used

| item | value | tag |
|---|---|---|
| full span | 400 mm | KAI |
| membrane | 2 x 0.10 mm = 0.20 mm | KAI (parameter by size) |
| dihedral (rail faces) | 0.00 deg per half | KAI |
| tail bend (rail face) | 5.81 deg | KAI |
| root strip | 7.0 mm wide x 1.70 mm thick | ASSUMED |
| LE bead | 3.0 -> 1.2 mm wide, 1.50 -> 0.60 mm high above the membrane, stops at 90 % of the half span (rectangular section) | ASSUMED |
| jigsaw root keys | 7 keys, head r 1.60, neck 1.20 x 1.60, post 0.80 mm high; socket = key + 0.15 mm on every edge, 0.15 mm deeper | ASSUMED |
| root gap between halves | 0.40 mm | ASSUMED |
| PLA density / lead density | 1.24 / 11.3 g/cm3, pocket fill margin 25 % | ASSUMED |
| bed | 250 x 250 mm (design); rotation search every 0.5 deg | KAI |

Every setting is described in `rigid_glider/sheet_builder_configs/README.md`.

## Design

**Wing halves.** Two mirrored halves, each printed with its FLAT face on the bed (this face is the top of the wing). The leading-edge bead, the root strip and the jigsaw posts stand up from the flat face on the print and hang BELOW the wing in flight, toward the rail. The printed right-wing half therefore has the mirrored planform. One unioned closed shell per part (raised regions are extra height of the same extrusion, vertical walls between regions).

**Jigsaw key.** A post (0.80 mm high) with a round head and a straight neck stands on the underside of the root strip. The rail has a matching pocket in each saddle face: the pocket outline is the key outline moved outward by **0.15 mm on every edge** (ASSUMED clearance) and is 0.15 mm deeper than the post. The pocket walls fix the half in x (fore-aft) and y; the strip's bottom face sits on the saddle face, which fixes z and the dihedral. Keys alternate neck direction (+x / -x) so a half cannot be turned end for end.

**Rail.** A thin web (2.0 mm, 4.0 mm in the ballast block) carrying four face plates (wing and tail, right and left, 1.50 mm thick, lightened between the keys). The wing plates are tilted +/- 0.00 deg about x (dihedral); the tail plates are rotated by 5.81 deg about the fold line x = 57.5 mm so the tail bend is a printed face, not a hand bend. The plates overlap the web top by 0.5 mm (overlapping closed shells in one STL, volume computed as the union).

**Rail print orientation: `side`.** The rail lies on its side (profile plane on the bed, the flight y axis becomes build height). The two dihedral faces and the tail face are then near-vertical walls made by perimeters (the dihedral only leans a wall by 0.00 deg, a lateral shift of 0.0000 mm per 0.1 mm layer), so their position and angle depend on XY accuracy (about 0.05 mm) instead of on layer stair-stepping (an upright print would stair-step a 0.0 deg face with 0.1 mm steps and must bridge the underside of the plates). Weak point: the bed contact is the edge of one plate (1.5 mm wide) - use a brim and a clean plate; if it lifts, set `rail.print_orientation` to `upright` and accept stair-stepped faces.

## Mass, CG and ballast (CALCULATED from the STL volumes)

| item | mass g | x mm aft of root LE |
|---|---|---|
| wing piece 1 (R+L) | 10.511 | 37.82 |
| rail | 3.843 | 36.26 |
| centre fin | 0.278 | 115.09 |
| root seam glue | 0.300 | 36.26 |
| fin glue | 0.050 | 115.09 |
| **bare total / CG** | **14.981** | **39.08** |
| ballast (lead) at x = -27.4 mm | 3.026 | -27.4 |
| **all-up / target CG** | **18.007** | **27.90** (63.9 mm from the nose tip) |

Wing loading 12.30 N/m2 on a wing area of 14362 mm2 (planform ahead of the hinge, both halves); ballast pocket 335 mm3 for 268 mm3 of lead (fill margin 25 %); window 13.9 x 6.0 mm through the 4.0 mm ballast block, centre x = -27.4 mm.

## Bed fit

| part | footprint after rotation | rotation | fits 250 mm |
|---|---|---|---|
| wing_R | 144.5 x 200.0 | 0.0 deg | yes |
| wing_L | 144.5 x 200.0 | 0.0 deg | yes |
| rail | 143.3 x 14.6 | 0.0 deg | yes |
| fin_centre | 20.9 x 22.5 | 0.0 deg | yes |

## Flags

- FLAG: trim tab cannot be made at this size: tab chord 5.9 mm leaves only 1.9 mm of tilted tab after the 4.0 mm key landing (need >= 3 mm); no tab family written

Notes:
- root strip thickness raised from 1.00 to 1.70 mm so it is not lower than the tallest bead (+ membrane)

## Assembly

1. Print the wings flat-face-down (bead up) in PLA, the rail as stated above, fins and tabs flat. Do not scale in the slicer. Brim for the rail.
2. Dry fit: each half must drop onto its rail plate with every post in its pocket, the strip flat on the plate. Do not file the pockets; if too tight, re-print with a larger `keys.clearance_mm`.
3. Glue each half to the rail with thin CA (or hot glue) on the strip/plate contact faces and rib walls; press flat on a table for the wings (the table, not your hand, sets the sheet).
4. Tail: press the tail fan down onto the tail plates (the thin hinge zone bends to the printed face at 5.8 deg) and glue it; do not hand-bend to a gauge.
5. Ballast: fit lead into the rail window (3.0 g), seal both sides with tape or CA, weigh, and compare the CG with the target mark on the DXF / foam template.
6. Fin: centre fin: slide the tongue into the rail's aft slot and glue.

## ASSUMED / not validated

- PLA 1.24 g/cm3, lead 11.3 g/cm3, glue masses and the pocket fill margin are ASSUMED; weigh the first printed half (CURRENT.md next action 2).
- Key/socket clearance 0.15 mm is ASSUMED, never printed. Seams are zero-gap butt joints.
- The bead is a rectangular step section, not a half round (the options study used half-round beads of the same width and height; areas differ).
- Tail halves are bent about slanted fold lines when dihedral and tail bend are both set; the root gap is enlarged automatically to stay clear (reported above).
- CG uses planform x of the sheet masses (bent-tail x shortening of 0.5 % ignored).
- Fin position (centre fin on the rail's aft end; tip fins on the tip tabs) is a choice made here, not the position in fin_kit_v4.
- No aerodynamic model is run on this geometry (AVL deck export not done).
