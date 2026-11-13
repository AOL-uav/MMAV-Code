# Tail-branch test article (fallback)

**Status as of 2026-10-11 (Kai): this is a developed fallback branch, not
the selected aircraft.** No architecture is selected; the pure flying wing
and the minimal-reflex wing are examined first, at a much smaller scale than
this 1.2 m article. "First build" and "first flight" below are meant within
the tail branch only.

A research prototype for getting a simple adjustable rigid glider flying. It
is not the final MMAV and none of these values is optimized. Every predicted
number comes from the existing inviscid AVL screens
(`results/runs/rigid_glider/prototype/tail_map.csv`, `fin_map.csv`,
`tolerances.csv`; background in `PROTOTYPE_REPORT.md` and
`REFLEX_RESULTS.md`). No new aerodynamic case was run for the original
document. On 2026-10-11 the 10% tail configuration was run directly
(`prototype_tail10.py`, outputs in
`results/runs/rigid_glider/prototype_tail10/`); it reproduced the existing
rows exactly and the values marked "10% tail run" below come from it.

All x positions are measured **aft from the root leading edge**, along the
centreline. y is spanwise from the centreline.

## 1. Main wing (frozen for the first build)

| Item | Value |
|---|---|
| Span | 1200 mm |
| Root chord | 207.8 mm |
| Tip chord | 135.1 mm (taper 0.65) |
| Quarter-chord sweep | 25.0 deg |
| Leading-edge sweep | 26.4 deg (tip LE is 298 mm aft of root LE) |
| Trailing-edge sweep | 20.6 deg (root TE at x = 208 mm, tip TE at x = 433 mm) |
| Area | 0.2057 m² (aspect ratio 7) |
| MAC | 174.0 mm, at y = 279 mm, its LE at x = 138.5 mm |
| Section | NACA 0008 throughout (16.6 mm thick at root, 10.8 mm at tip) |
| Twist, washout, dihedral | zero |

The planform is a straight-tapered trapezoid per side: LE, TE and every
constant-chord-fraction line are straight.

## 2. CG

| Item | Value |
|---|---|
| Initial CG, first flights | x = 171 mm (18.7% MAC) |
| Model nominal CG, 10% static margin | x = 176 mm (21.6% MAC) |
| Adjustment range to provide | x = 150 to 200 mm, readable to 1 mm |

The range covers the tail configurations (about 165 to 185 mm) and the later
tail-off reflex tests, which need about 152 to 165 mm because the bare
wing's neutral point is at x = 178 mm.

Hardware note, my inference and not a model result: a uniformly dense wing
of this planform balances near x = 210 mm, and the boom, tail and fin move
it further aft. Expect to need ballast well forward, so give the centre
body a nose that extends ahead of the root LE and can carry the ballast
rail.

## 3. Horizontal tail (removable)

| Item | First-flight tail | Smaller alternate |
|---|---|---|
| Area | 205.7 cm² (10% of wing) | 164.6 cm² (8% of wing) |
| Span | 420 mm | 420 mm |
| Chord | 49.0 mm, constant | 39.2 mm, constant |
| Quarter-chord station | x = 440 mm | x = 440 mm |
| LE / TE station | x = 428 / 477 mm | x = 430 / 469 mm |
| Arm, CG to tail quarter chord | 269 mm at CG 171; 264 mm at CG 176 | 266 mm at CG 174 |
| Incidence relative to wing chord | −3.0 deg (tail LE down) | −3.0 deg |
| Sweep, dihedral | none | none |
| Height | in the wing chord plane, as modelled | same |
| Model nominal CG for 10% margin | x = 176 mm | x = 174 mm |

- The 10% tail is chosen first for its larger chord: tail Reynolds number is
  about 20k against 18k for the alternate, and both are below any polar data
  in this project. That is the main aerodynamic unknown of the tail branch.
- Both tails mount at the same station, so the swap needs no boom change.
  Reset the CG when swapping.
- The model's tail section is NACA 0008. A flat plate of similar thickness
  with a rounded leading edge is a practical substitute but was not
  modelled.
- At the tail tips (y = 210 mm) the tail leading edge is about 140 mm
  behind the wing trailing edge; on the centreline it is 220 mm behind.

## 4. Centre fin (removable)

| Item | First fin | Larger alternate |
|---|---|---|
| Area | 61.7 cm² (3% of wing) | 82.3 cm² (4% of wing) |
| Chord | 87 mm, constant | 87 mm |
| Height | 71 mm | 95 mm |
| Quarter-chord station | x = 480 mm | x = 480 mm |
| LE / TE station | x = 458 / 545 mm | same |
| Modelled Cn_beta (with the 8% tail) | +0.012 /rad | +0.019 /rad |

Without a fin the model gives Cn_beta of about +0.002 /rad, effectively no
directional stability. The table above is from the fin map run with the 8%
tail. With the 10% tail (10% tail run, `fin_map.csv`), Cn_beta in /rad is:

| Fin | CG 171 mm | CG 176 mm |
|---|---:|---:|
| none | +0.0018 | +0.0028 |
| 3% | +0.0118 | +0.0127 |
| 4% | +0.0192 | +0.0199 |

Cn_beta rises as the CG moves aft. The fin does not change trim CL or
static margin. The boom therefore runs
from the wing to about x = 550 mm. Boom mass, stiffness and drag are not
modelled.

## 5. Trailing-edge strip provision (for later reflex tests)

Do not build a reflex shape now. Build the wing so the aft part can be
changed without reprinting it:

- End the main wing body at the **80% chord line** and make the last 20% of
  chord a separate strip. That line is straight on each side: x = 166 mm at
  the root, x = 406 mm at the tip, swept 21.8 deg.
- The section is 3.5% of chord thick at that line: 7.3 mm at the root,
  4.7 mm at the tip. That is the depth available for the joint (a lap or
  tongue joint with small screws or pins).
- First strip set: **neutral**, completing the plain NACA 0008. All first
  flights use it.
- Split each side's strip at **30% semispan (y = 180 mm)**, and preferably
  again at 55% (y = 330 mm). The existing analysis found an inboard 0–30%
  lip does nothing useful and the outboard part does the work, so the
  outboard pieces are the ones to swap.
- 20% of chord covers the largest lip analysed (20% chord, 10 deg). Smaller
  lips are made as strips that are neutral forward of their bend.
- The joint must locate the strip repeatably in angle. A 1 deg error in lip
  angle or a 1 deg left/right mismatch is a measurable effect in the model.

## 6. Predicted first-flight condition

10% tail at x = 440 mm, −3 deg, 3% fin, neutral strips. Inviscid AVL.

| CG | Trim CL | Trim alpha | Static margin |
|---|---:|---:|---:|
| x = 171 mm (initial) | 0.262 | 3.6 deg | 12.9% MAC |
| x = 176 mm (model nominal) | 0.337 | 4.6 deg | 10.1% MAC |
| x = 181 mm | 0.471 | 6.2 deg | 7.2% MAC |

Neutral point with this tail: x = 193.5 mm.

Glide speed, from V = sqrt(2 W / (rho S CL)) at sea-level density. AVL's
trim CL does not depend on mass:

| All-up mass | Wing loading | V at CG 171 (CL 0.262) | V at CG 176 (CL 0.337) |
|---:|---:|---:|---:|
| 120 g | 5.7 N/m² | 6.0 m/s | 5.3 m/s |
| 168 g | 8.0 N/m² | 7.1 m/s | 6.2 m/s |
| 220 g | 10.5 N/m² | 8.1 m/s | 7.1 m/s |
| 300 g | 14.3 N/m² | 9.4 m/s | 8.3 m/s |
| 400 g | 19.1 N/m² | 10.9 m/s | 9.6 m/s |

168 g is the 8 N/m² placeholder used in the studies, not a mass estimate.
Weigh the built article and read the speed from its row.

Sensitivities:

- **CG:** about 0.015 CL per mm forward of nominal and 0.027 per mm aft.
  Static margin changes 0.57% MAC per mm. Treat CG as a 1 mm measurement.
- **Tail incidence (10% tail run):** 0.087 CL per degree at CG 171 mm,
  0.111 at 176 mm and 0.154 at 181 mm, so it nearly doubles across the CG
  range. At CG 171 mm, −4 deg gives CL 0.348 and −2 deg gives 0.174; at
  176 mm, 0.447 and 0.224. Incidence barely moves static margin (about
  0.04% MAC per degree). At −1 deg the trim CL is only 0.09 to 0.16.
- **Left/right wing incidence mismatch (10% tail run):** 0.0065
  rolling-moment coefficient per degree at both CGs, the largest unbalanced
  moment in the model. Measure it and hold it well under 0.5 deg.
- **Fin yaw misalignment (10% tail run):** 2 deg is equivalent to 1.7 deg
  of sideslip offset at CG 171 mm and 1.5 deg at 176 mm.
- **Stage 4 CGs (10% tail run, no longer interpolated):** trim CL 0.294 at
  173.5 mm and 0.392 at 178.5 mm.

What the model does not cover: all viscous effects on lift and moment, the
tail at Re 20k, boom and fin drag, glide ratio, stall, and any dynamics. The
section analysis in `REFLEX_RESULTS.md` suggests the bare 0008 wing's
moment and aerodynamic centre shift at these Reynolds numbers by an amount
comparable to the static margin, which is why the first CG is forward of
nominal.

## 7. First releases (18)

Before any release: weigh the article; measure CG x; measure tail incidence
and left/right wing incidence; check fin alignment. Record all of it. Use
the same release method throughout, near the predicted speed for the
measured mass, and film from the side and from behind or above.

Starting configuration: 10% tail at −3 deg, 3% fin, neutral strips, CG
171 mm.

| Stage | Releases | Configuration | Question |
|---|---:|---|---|
| 1 | 3 | Starting configuration, level release | Does it glide at all? |
| 2 | 4 | Same; 2 released about 5 deg nose-up, 2 about 5 deg nose-down | Does it converge in pitch? |
| 3 | 2 | Same; 1 released about 5 deg banked left, 1 right | Severe roll or yaw divergence? |
| 4 | 6 | CG 173.5, 176 and 178.5 mm, 2 level releases each | Which CG gives the most repeatable glide? |
| 5 | 3 | Best CG from stage 4, fin removed | Is the fin necessary? |

Predicted trim CL for stage 4 is 0.29, 0.34 and 0.39 (10% tail run).

Rules between stages:

- Stage 1 trims first. If it dives, change tail incidence to −4 deg or move
  the CG 2 mm aft. If it pitches up or stalls, change tail incidence to
  −2 deg or move the CG 3 mm forward. Change one thing at a time and
  re-measure. Do not go on until 2 of 3 releases glide.
- Roll and yaw behaviour is also watched in stages 1 and 2; stage 3 only
  adds a deliberate disturbance. A consistent turn in one direction points
  first at wing incidence mismatch or fin alignment, so re-measure before
  changing the design.
- Do not take the CG aft of 181 mm in this plan (model margin about 7%).
- Stop and inspect after any hard landing; re-measure CG and incidences.
- Stage 5 is skipped if stages 1 to 3 already show roll or yaw divergence
  with the fin on. Try the 4% fin instead.

The 8% tail, the 4% fin, other tail stations and all reflex strips are left
for after these 18.

## 8. Keep adjustable

| Feature | Range or options |
|---|---|
| CG (ballast rail) | x = 150 to 200 mm |
| Ballast mass | enough to span the mass you want to test |
| Tail incidence | −1 to −5 deg in 0.5 deg steps, measurable |
| Tail planform | swappable: 10% and 8% |
| Tail station | provision for quarter chord at x = 380 to 500 mm if cheap; 440 mm first |
| Fin | removable; 3% and 4%; yaw alignment measurable |
| Trailing-edge strips | removable aft 20% of chord, split at 30% (and 55%) semispan |

Fixed: wing planform, section, zero twist, zero dihedral.

Datums worth building in: a flat reference on the centre body parallel to
the wing chord plane for incidence measurements, a marked x = 0 at the root
leading edge, and marked CG stations along the rail.
