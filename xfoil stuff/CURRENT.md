# CURRENT - MAV-2026 Codes frontier (checkpoint 2026-10-21)

This file is the frontier. `README.md` describes layout and the older study chain
(direction as of 2026-10-11); where they disagree, this file is newer. Not a git
repo; OneDrive-synced; study scripts overwrite their own output folders in place.

## Goal

A very small unpowered glider (about 0.15-0.50 m span) that pulls out of a
nose-down drop from the carrier quad and flies straight, stable and smooth with no
active control. Architecture preference (Kai, 2026-10-11): pure flying wing, then
flying wing with a small reflex/lip, then small tail as fallback.

## Where it stands

Flight phase. Several airframes glide from the drop; the problem is now flight
quality (pitch oscillation, steady turn), not "does it fly".

Flight evidence (all REPORTED by Kai, qualitative, nothing measured):
`results/runs/flight_day_2026-10-20/FLIGHT_LOG.md`

| airframe | result |
|---|---|
| B45 foamboard flat wing (400 mm, P07 outline, tip fins, two ~10x50 mm centre flaps ~10 deg up) | best: glides, "semi heavy" oscillation, good distance. Lawn-darts with flaps flat (2026-10-19). |
| Swift A foamboard 400, tail bent ~6 deg, CG ~73.0 mm from nose tip | second: glides, heavy oscillation; small CG moves did not help; less bend gave a nose dive |
| Swift A foamboard 250 | "kinda unstable" |
| Swift A printed 250 | shredded by props at release (tether too short; not a design issue) |
| Zanonia printed replica (155 mm) | not printed, not flown |

Common: most gliders settle into a steady turn (probably same direction each
time; foamboard imperfection suspected). Kai reads the swoops as stalls that do not
die out (stall porpoise). Swoops shrank but were still present after ~10 cycles (Kai: phugoid early, stall swoops late). Both large foam gliders flew best at the model's estimated balance point. All video is from beneath, so no period can be measured.

## Model status (MODEL; none of it is flight-validated)

- **Swift A oscillation sim** - `rigid_glider/swiftA_oscillation_study.py` ->
  `results/runs/rigid_glider/swiftA_oscillation/README.md`. Fails 2 of 3 pre-stated
  checks. At 73 mm / 6 deg it predicts a lightly damped phugoid (~3.1 s, decays,
  one stall on the first swoop), not persistent stalls; it still glides at 3-4 deg
  bend where Kai saw a dive. Trim Re ~17,400 is 3.5x below the 60,000 plate data.
  **Do not use its smooth region (forward CG, small bend) as settings.**
- **B45 centre flaps AVL** - `results/runs/rigid_glider/b45_centre_flaps/README.md`:
  weak agree, only if CG is ~63-68 mm behind root LE (Xnp 70.24 mm, MAC 92.2 mm).
- **Zanonia known-good check** - `research/known_good/zanonia/RESULTS.md`: rule
  verdict DISAGREE (SM 1.0 % MAC vs 3 % floor), but the miss is ~0.3 mm, inside
  drawing resolution. Test card: `research/known_good/zanonia/TEST_CARD.md`.
- **Tumble model** (Li et al. 2022 falling plate) - `research/tumble_model/README.md`,
  `results/runs/tumble_sim/`: step A validation passed its pre-written rule.
- **Printed Swift A 400 options** (mass/balance only, no CAD/STL) -
  `results/runs/rigid_glider/swiftA_printed400_options/README.md`. Solid PLA rows
  are 1.4-2.3x the foam 400's wing loading (foam 400 = 9.76 g). Only LW-PLA + LE
  spar (D-h3.0, ~10.3 g) matches the foam, and it rests on ASSUMED density and on
  LW-PLA printing a 0.3-0.4 mm membrane (unverified).

Standing rule for this project: AVL and XFOIL are not ground truth; agreement
checks are written before the run and not edited after.

## Blocker

No measured flight data. Every model-vs-flight comparison is against words.

## Next actions (in order)

1. **Measured test flights** (Kai): per flight record mass, balance CG (mm from
   nose tip), tail bend / flap angle gauged before each flight, drop height, flight
   time, turn direction, and **side-view video** (for the swoop period; model
   predicts ~3.1-4.4 s if it is a phugoid). Longer tether.
2. **Print one 0.4 mm Swift A 400 half and weigh it** - this decides whether any
   PLA option in the options table is real (if it is heavier than area x 0.4 mm x
   1.24 g/cm3, every PLA row is low).
3. Only after 1: revisit the oscillation model (low-Re section data, flap/bend
   effectiveness) against measured numbers.

Open decision (Kai): which airframe to develop after measured flights - B45
(flat wing + centre reflex, option 2 in the order) or Swift A (bent tail).

## Reproduce / continue

Python is stdlib-only, run with `py` from this folder. AVL 3.52:
`tools/avl/avl352.exe`; XFOIL 6.99: `tools/xfoil/xfoil.exe`.

```powershell
py -B rigid_glider/swiftA_oscillation_study.py --out <new folder>   # default --out overwrites results/runs/rigid_glider/swiftA_oscillation
py -B rigid_glider/swiftA_printed400_options.py                     # overwrites results/runs/rigid_glider/swiftA_printed400_options
py -B rigid_glider/b45_centre_flaps.py
```

New work goes in new scripts and new output folders; do not re-run an existing
study over its published results without saying so. Preserve `data/polars/*.dat`,
`results/legacy/`, `archive/`.

## Still unknown / not captured

- Turn direction and circle size per glider; oscillation growth/decay; any
  measured CG or mass from any flight.
- Kai's vase-mode data (16 g per 400 mm panel at 0.42 mm wall) has no recorded
  planform or section, so it is not used as an input.
