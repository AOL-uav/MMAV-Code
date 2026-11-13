# Article 1: P07 modular test article, sheet flat-plate build first

2026-10-11. Decisions by Kai on this date:

1. **Planform: P07** (span 400 mm, AR 4.5, taper 0.5, quarter-chord sweep
   25 deg). Chosen as a test article, not as a winner: the model does not
   separate P07, P09 and P12 (`ARCH_COMPARISON.md`).
2. **Fin for the clean wing: tip fins first**, centre fin on a short boom as
   the backup if the wing wanders in yaw.
3. **Build order: a sheet flat-plate wing with a printed centre piece
   first**, then a printed NACA 0006 wing of the same planform.

The tail is a module, not part of the baseline airframe. Test order is
clean wing, then lip, then tail.

**Nothing here is validated.** No mass is measured. No flat-plate prediction
exists: the XFOIL plate attempt disagrees with published measurements
(`results/runs/rigid_glider/plate/summary.txt`,
`plate_te_diag/diag_summary.txt`). The CG starting points below are the
NACA 0006 model values, used for the plate only as places to begin.

Drawings and tables: `results/runs/rigid_glider/article1/` (open
`index.html`). Regenerate with `py -B rigid_glider/article1_drawings.py`.

CAD package (DXF planform and sections, 3D loft curves, station table,
dimensioned drawing) for 400 mm and a geometrically scaled 350 mm:
`results/runs/rigid_glider/article1_cad/index.html`, from
`py -B rigid_glider/article1_cad.py [span_mm ...]`. Only 400 mm was
modelled; span has not been changed from 400 mm.

## Geometry (both wings)

| Item | Value |
|---|---|
| Span / area | 400 mm / 355.6 cm² |
| Root / tip chord | 118.5 / 59.3 mm |
| Mean aerodynamic chord | 92.2 mm, leading edge 48.0 mm aft of root LE, 88.9 mm out |
| Tip leading edge | 108.1 mm aft of root LE (quarter-chord sweep 25 deg) |
| Tip trailing edge | 167.3 mm aft of root LE |
| Dihedral | 3 deg per panel; tip rise 10.5 mm; centre joint included angle 174 deg |
| Twist | none |
| Tip fins | two, each 59.3 mm chord x 24 mm tall (4% of wing area each), 12 mm above and 12 mm below the wing, vertical |
| AVL neutral point | 70.7 mm aft of root LE with centre fin; 71.1 mm with tip fins; 82.5 mm with the 10% tail |

Tip fins are the "tip fin 4% sym" case of
`arch_compare_v2/item9_tipfins.csv`: Cn_beta 0.0082 per rad at CL 0.3,
against 0.0016 bare and 0.017 for the 6% centre fin. Tip fins have only
been modelled on the clean wing, not with the lip or tail fitted.

## Article 1a: sheet flat plate

| Part | Make from | Size | Qty |
|---|---|---|---|
| Wing panel | 3 mm foam sheet (preferred) or 1.5 mm balsa, grain spanwise | `panel_template_1to1.svg` | 2 per configuration (one template, flipped) |
| Tip fin | same sheet | 59.3 x 24 mm | 2 |
| Centre piece | printed | see below | 1 |
| Tail | same sheet | 140 x 25 mm (alternate 140 x 20) | 1 |
| Tail boom | 2–3 mm carbon rod or tube | reaches 256 mm aft of root LE | 1 |
| Spare centre fin | same sheet | 36 chord x 59 tall | 1 |
| Ballast | steel or tungsten, in the centre-piece channel | see table | as needed |

3 mm sheet is 3.3% of the mean chord, inside the 2.6–4.3% range of the
published plate tests. 1.5 mm balsa is 1.6%.

**Edges.** Round the leading edge to a half-ellipse about 7 mm long. Taper
the trailing edge over about 15 mm to roughly 0.5 mm. Make both panels the
same; use one sanding template for both. Left/right mismatch is the most
likely cause of unrepeatable flights.

**Configurations.** Sheet panels are cheap, so cut a panel pair per
configuration instead of one adjustable pair:

| Pair | Trailing edge outboard of 80 mm |
|---|---|
| A | clean |
| B | cut along the 80% chord line, tape-hinged, set to 10 deg up (rise 3.3 mm inboard end, 2.1 mm at tip) |
| C | cut along the 85% chord line, tape-hinged, set to 8 deg up (rise 2.0 mm inboard end, 1.2 mm at tip) |

Set lip angles with a printed wedge gauge at both ends of each strip and
check the rise with calipers on both sides. Roll from a lip-angle mismatch
is about 0.0024 per degree in the model.

**Centre piece (printed; not yet drawn in CAD).**

- Two 3.2 mm slots, each 12 mm deep, holding the panel roots at 3 deg up
  (174 deg included). Slots run the full 118.5 mm root chord.
- Ballast channel along the centreline from 30 mm ahead of the root LE to
  95 mm aft, with a mm scale. The ballast slug is locked with tape or a
  screw.
- CG scale on the underside from 40 to 85 mm aft of root LE in 1 mm ticks,
  for use on a knife-edge balance stand.
- Socket for the tail boom on the centreline, and a slot for the spare
  centre fin with its quarter chord 178 mm aft.
- Tail saddle setting incidence at −3 deg, with −2 and −4 alternates; tail
  quarter chord 237 mm aft of root LE; provision to raise it 6–12 mm.
- Mass target 5 g or less (assumed in the ballast table).

**Mass and ballast (assumed densities, not measured).** All-up mass is set
by ballast. At CL 0.3:

| Wing loading | All-up mass | Speed | Reynolds number, mean chord / tip |
|---:|---:|---:|---|
| 6 N/m² | 21.8 g | 5.7 m/s | 35,000 / 23,000 |
| 8 N/m² | 29.0 g | 6.6 m/s | 41,000 / 26,000 |
| 12 N/m² | 43.5 g | 8.1 m/s | 50,000 / 32,000 |

| Wing sheet | Wing mass | Ballast at 8 N/m² | Ballast station for CG 54 / 64 / 78 mm |
|---|---:|---:|---|
| 3 mm foam (40 kg/m³) | 4.3 g | 19.4 g | 49 / 64 / 85 mm |
| 1.5 mm balsa (160 kg/m³) | 8.5 g | 14.8 g | 33 / 53 / 81 mm |
| 3 mm balsa (120 kg/m³) | 12.8 g | 10.2 g | 5 / 33 / 73 mm |

Full table: `article1/ballast_table.csv`. Weigh every part and replace
these numbers with measurements before flying. With a foam wing most of
the mass is ballast near the CG, so pitch inertia is much lower than on
the printed wing; that affects the motion after release, not the static
trim.

## Article 1b: printed NACA 0006

Same planform, tip fins and centre-piece interfaces. Thickness 7.1 mm at
the root and 3.6 mm at the tip. Removable aft-20% strips outboard of
80 mm (neutral, 15% / 8 deg, 20% / 10 deg). Mass target 24–28 g all-up
(`ARCH_COMPARISON.md` section 5); the pure-wing window is gone on P07 by
16 N/m².

**2026-10-12, Kai's slicer figures (standard PLA, one-wall vase mode):**
16 g per panel at 0.42 mm wall, 13 g per panel at 0.28–0.3 mm. A 26 g wing
gives roughly 40 g all-up once balanced (hand estimate: shell CG about
94 mm aft of root LE, nose ballast at −30 mm, 5 g centre piece), about
11 N/m². The 24–28 g target is therefore not reachable this way, and
foaming PLA is not planned. Article 1b is the heavy case; the model's
pure-wing window there is about 1 mm. Shell mass per unit area is fixed by
the wall, so a smaller span does not lower the wing loading. Shell
stiffness, print warp and skin denting are not modelled. If the tip prints badly, change to NACA 0008 and record it as a
manufacturing-driven change. Build after 1a has flown.

## Test card

**Before flying, every configuration**

1. Weigh the article. Record all-up mass and wing loading.
2. Measure the CG on the balance stand, to 1 mm.
3. Check symmetry: panel twist by sighting along the span, tip fin
   alignment, lip rise on both sides, tail incidence.
4. Photograph the article from above and from the front.

**Launch.** Use a ramp or fixed-stretch rubber catapult giving the same
speed and attitude each time, aimed at the predicted glide speed for the
wing loading (table above). Indoors or still air. Film side-on against a
marked background.

**Sequence, starting at 8 N/m²**

| Step | Configuration | First CG | Step | Direction |
|---|---|---:|---:|---|
| 1 | Clean wing, tip fins | 64 mm | 1 mm | aft, to 72 mm or until pitch-up |
| 2 | Lip 20% / 10 deg (pair B) | 54 mm | 2 mm | aft, to 64 mm |
| 3 | Lip 15% / 8 deg (pair C) | 56 mm | 2 mm | aft, to 66 mm |
| 4 | Tail 10%, −3 deg, clean pair A | 62 mm | 2 mm | aft, to 76 mm |

Steps 1, 2 and 4 start points are from `ARCH_COMPARISON.md` section 6
(NACA 0006). The step 3 start point and the end points are judgement, not
model output. Start forward, on the dive side. A dive or fast flat glide:
move the CG aft. A pitch-up or stall: move it forward.

- 5 releases at each CG setting; 10 at any setting that glides.
- If the clean wing wanders in yaw at every setting, fit the centre fin
  and repeat step 1 before moving on.
- Repeat the best setting of each configuration at 6 and 12 N/m².
- Then repeat the whole card on Article 1b.

**Log for every release:** configuration, mass, CG, launch setting, and
the outcome as one of: steady glide, dive, pitch-up or stall, roll-off
(left or right), yaw wander, oscillation. For steady glides also record
glide distance and time from the video.

**What counts as repeatable (proposed, to be agreed):** at least 8 of 10
releases at one setting end in a steady glide with the same heading
tendency, and the neighbouring CG settings 1 mm either side also glide.
The width of that run of settings is the measured CG window, which is the
number the model tried to predict.

## Open

- Printer bed size and cutting tools are not yet known; the centre piece
  has no CAD.
- Sheet stock is foamboard (Kai, 2026-10-12), not the 3 mm foam or balsa
  assumed in the Article 1a tables. Thickness, brand and areal mass are
  not yet known; at 5 mm it is 5% of the mean chord and 8% at the tip,
  thicker than the published plate tests. The 1a mass and ballast tables
  need redoing once the board is weighed.
- Tip fins with the lip or tail fitted have not been modelled.
- The 20 cm benchmark gliders have not been weighed; no mass in this
  document is measured.
