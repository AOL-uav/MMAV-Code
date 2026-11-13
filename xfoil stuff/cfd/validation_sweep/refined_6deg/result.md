# Refined laminar 6 deg run: result against the fixed criteria

Verdict: **PARTIAL** (as defined in `README.md`, written before the run). Approved on review 2026-10-14.

Case `/home/cfd/run/vp_ref_a06` (4,421,076 cells, laminar, 12 ranks) reached t = 20 chord passages: 27,799 steps, mean dt 0.00072, maximum Courant 1.50, wall time 128,267 s (35.6 h), peak memory 7.8 GB. Read-only post-processing; scripts in `scripts/` (`analyze_ref.py`, `extract_ref.py`, `bubble_ref.py`); data in `time_binned_ref.csv`, `surface_profiles_ref.csv`, `bubble_extent_ref.csv`.

## Averaging window and validity
The force coefficients settle by about t = 3 (1-chord bins in `time_binned_ref.csv`). Window used: **t = 8 to 20 (12 chord passages)**, as in the README plan. Halves [8,14] and [14,20]: CL 0.3339 and 0.3347 (0.26 % apart, limit 2 %), CM +0.0117 and +0.0114 (0.0003 apart, limit 0.002): the validity conditions are met. Window t = 10-20 gives the same numbers (CL 0.3345, CM +0.0115, x_cp 0.2159). Fluctuation (sd) in the window: CL 0.0013, CM 0.0005.
y+ on the plate patch at t = 20 (all faces, upper and lower): min 0.033, mean 1.21, max 17.9 (the maximum is not located; upper and lower surfaces are not separated in the solver's summary). Cell size in the shear layer: 0.00195 c (level 7) over x/c -0.04 to 0.20 and 0.0039 c (level 6) over x/c -0.07 to 0.85, up to about 0.02 c and 0.05 c above the upper surface; first layer 0.0004 c and 0.0008 c.

## Averaged coefficients
| | CL | CD | CM (c/4, nose-up +) | x_cp = 0.25 - CM/CN |
|---|---|---|---|---|
| refined laminar, t = 8-20 | 0.3343 | 0.0388 | +0.0115 | 0.2157 |
| refined laminar, t = 10-20 | 0.3345 | 0.0388 | +0.0115 | 0.2159 |
| coarse laminar (sweep, t = 8-20) | 0.3486 | 0.0483 | -0.0040 | 0.2613 |
| measured at 6.0 deg, increasing-alpha leg | 0.3709 | 0.0462 | +0.0155 | |
| measured at 6.0 deg, decreasing-alpha leg | 0.3814 | 0.0435 | +0.0145 | |
| measured, mean of legs | 0.3761 | 0.0448 | +0.0150 | 0.2104 |
| AVL (inviscid, linear) | 0.3276 | | | |

## Criteria, applied as written in `README.md`
| Criterion | Value | Result |
|---|---|---|
| CM between +0.010 and +0.020 | +0.0115 | met |
| x_cp between 0.195 c and 0.225 c | 0.2157 | met |
| CL at or above 0.357 | 0.3343 | not met |
PASS requires all three, so the result is not PASS. PARTIAL is "CM positive but outside the band, or CM closes more than half the gap from -0.004 without meeting the pass": CM closes 82 % of the gap without the pass being met. FAIL does not apply (CM is positive and closes far more than a quarter of the gap).
**Consequence, as written in the README before the run:** PARTIAL means resolution matters and nothing is validated. Stop. CFD is recorded as not usable for moment; lift is a rough check with a stated bias; no further CFD without Kai asking.

The README's one allowed substitution ("CL within 3 % of AVL") applies only if AVL is near the coarse CFD lift. At 6 deg, the angle of this run, AVL (0.3276) is 6.4 % below the coarse CFD lift (0.3486), so it was not invoked when the criteria were written and is not invoked here. The CL criterion stays as written. The refined CL is 2.1 % above AVL; that and the measured-above-AVL gap are recorded as open items, not as part of the verdict.

## Facts
- CM +0.0115 (band +0.010 to +0.020); x_cp 0.2157 (band 0.195-0.225).
- Reversed flow reattaches by about 0.33 c at all three span stations (see below).
- **Not shown to be mesh-independent.** One refinement (coarse to refined) moved CM by 0.0155 (-0.0040 to +0.0115) and CD by 20 % (0.0483 to 0.0388). CD is 13.4 % below the measured mean (0.0388 against 0.0448; the coarse value was 7.8 % above). Drag is not judged.

## Gap closed (coarse laminar to refined, against the measurement)
| | gap, coarse to measured | gap, refined to measured | fraction of the gap closed |
|---|---|---|---|
| CM (leg mean +0.0150) | +0.0190 | +0.0035 | 82 % (up leg 79 %, down leg 84 %) |
| x_cp (0.2104) | -0.0509 (coarse aft) | -0.0053 | 90 % |
| CL (leg mean 0.3761) | +0.0275 | +0.0418 | -52 % (the gap widened; up leg -64 %, down leg -44 %) |
| CD (leg mean 0.0448) | -0.0035 (coarse above) | +0.0060 (refined below) | not a closing: coarse +7.8 %, refined -13.4 % against measured |

## The lift gap: unresolved
Refined CL 0.3343 is 11.1 % below the measured mean (coarse: 7.3 % below), so refining moved lift away from the measurement. Two explanations remain open and neither is established here:
- **(a) A tunnel or correction offset** between the measurement and a free-air calculation.
- **(b) The real wing at this Reynolds number lifts more than the CFD captures.**
Evidence that bears against a reading that is only a rig effect:
- The tunnel's lift-slope excess over AVL rises with Reynolds number. Using the fitted slopes in `research/plate_data/README.md` (0.0607 per deg at Re 60k, 0.0682 per deg at Re 120k) against AVL's secant slope at 4 deg (0.0548 per deg), the excess is about +11 % at Re 60k and +24 % at Re 120k (against AVL's tangent slope 0.0533 per deg: +14 % and +28 %).
- A rough wall-correction size of about 1 % of angle at 4 deg. This is an unverified reviewer estimate; the thesis pages on its corrections (PDF pp. 57-59) give equations but no sizes.
- Other tunnels, from `research/bird_research_notes.md` section 6.3 (secondary sources, abstracts only): Laitone found lift slopes over 20 % above lifting-line theory at Re 40,000; Pelletier & Mueller are summarised there as finding lifting-line theory adequate at Re 60,000-200,000, and McArthur found no significant disagreement at Re 10,000-20,000. So the literature is not uniform.
AVL agreement with the CFD (equal at 4 deg, 2.1 % apart for the refined run at 6 deg) is two models agreeing, not validation.

## Reversed flow on the upper surface (same method as the sweep; `bubble_extent_ref.csv`)
Zones of reversed flow (x/c). Refined run: time average over t = 8-20 (the run's `wallShearStressMean`) and the t = 20 snapshot; coarse laminar: mean of its two snapshots (t = 15, 20) in `../bubble_extent.csv`.
| span | refined, time average 8-20 | refined, t = 20 snapshot | coarse laminar |
|---|---|---|---|
| 25 % | 0.03-0.33 | 0.03-0.15; 0.19-0.34 | 0.05-0.28; 0.36-0.75 |
| 50 % | 0.04-0.13; 0.16-0.32 | 0.04-0.13; 0.14-0.29 | 0.05-0.58 |
| 75 % | 0.06-0.27 | 0.06-0.26 | 0.06-0.29 |
Reversed fraction of the upper chord: refined 0.31 / 0.27 / 0.22 (time average) against coarse 0.65 / 0.55 / 0.24. The lower surface is forward-flowing at all three stations (sign check). The post-processing used a copy of the t = 20 fields in `/home/cfd/run/diag_ref` and wrote nothing into the case.

## Rule unchanged
No CFD is run on the fabricated wings, and CFD is not used for balance or pull-out.

## Limits
- The y+ maximum (17.9) is from the solver's patch summary and is not located.
- The time-averaged reversed-flow map is the single mean field at t = 20 (average over 8-20), not separately averaged sub-windows.
- One mesh refinement at one angle.
