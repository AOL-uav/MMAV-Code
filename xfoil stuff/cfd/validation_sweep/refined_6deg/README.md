# Refined laminar run at 6 degrees (reviewer's last CFD run) - criteria fixed BEFORE the run

STATUS: finished 2026-10-14. Verdict PARTIAL; see `## Result` at the end and `result.md`.

Written 2026-10-13, before the case was meshed or started. These criteria are unchanged by anything that follows.

## Reference values (recomputed from `../sweep_results.csv`)
Measured at 6 deg (Ananda 2012 tables, Re 60k, interpolated): CL 0.3761, CD 0.0449, CM +0.0150, centre of pressure x_cp = 0.25 - CM/CN = 0.2104 c (CN = CL cos(alpha) + CD sin(alpha) = 0.3787).
Coarse laminar CFD at 6 deg (window 8-20): CL 0.3486, CD 0.0483, CM -0.0040, x_cp 0.2613 c.
AVL (inviscid, linear, `../setup_checks/`): CL 0.3276 at 6 deg.

## Validity of the averaged result
- Average over at least 10 chord passages after the flow has settled.
- The two halves of the averaging window agree within 2 % in CL and within 0.002 in CM.
- Report y+ and the cell size in the shear layer (leading edge to 0.75 c) alongside.

## Outcome bands (applied to the averaged CL, CD, CM and x_cp of the refined run)
- **PASS (all three):** CM between +0.010 and +0.020; x_cp between 0.195 c and 0.225 c; CL at or above 0.357.
- **PARTIAL:** CM positive but outside the +0.010 to +0.020 band, or CM closes more than half of the gap from -0.004 to the measured +0.015 (that is CM above +0.0055) without meeting PASS. Means resolution matters; nothing is validated.
- **FAIL:** CM at or below zero, or CM closes less than a quarter of the gap (CM below +0.0007).
- **The one allowed substitution (CL criterion -> "CL within 3 % of AVL")** applies if the setup checks show AVL near the coarse CFD lift. The check found AVL equal to the coarse CFD at 4 deg (0.2190 against 0.2189) but 6.4 % below it at 6 deg (0.3276 against 0.3486), the angle of this run, so AVL is **not** near the coarse CFD lift at 6 deg and the substitution is **not** invoked here: the CL criterion stays "CL at or above 0.357". The refined CL is also reported against AVL (0.3276 +/- 3 % = 0.318-0.337), and the measured-above-inviscid gap (measured 14.8 % above AVL at 6 deg) is recorded as a separate open item, not as part of PASS/FAIL.

## Consequences (set by the reviewer)
- PASS means the 6 deg moment is reproduced, nothing more. The rule on balance, pull-out and Kai's wings stays until 8 deg is also reproduced (a new run beyond the cap, Kai's decision).
- PARTIAL or FAIL: stop. CFD is recorded as not usable for moment; lift is a rough check with a stated bias; no further CFD without Kai asking.

## Run plan (details and measured numbers are added below once meshed, without changing the above)
Refined laminar (no turbulence model) at 6 deg, case `/home/cfd/run/vp_ref_a06`, all 12 cores, detached with status and stop scripts. Fallback to 4 deg only if the 6 deg case will not fit in memory or cannot reach 10 averaged chord passages in roughly 24-36 h; any fallback is stated here and is the weaker test.

## Run plan with measured numbers (added 2026-10-13 09:45 after meshing and the first minutes of the run; the criteria above are untouched)
- Case `/home/cfd/run/vp_ref_a06` (laminar pimpleFoam, no turbulence model, same numerics as the sweep: backward in time, limitedLinearV 1, Courant 1.5), alpha 6, scripts in `scripts/` (copies of `/home/cfd/run/ref/`). All 12 cores.
- **No fallback to 4 deg was needed**: the 6 deg case fits in memory (peak 7.6 GB of the 15.5 GB the distro sees) and the run time is about 33 h.
- Mesh (snappyHexMesh, same background mesh and recipe as the coarse case plus local refinement): **4,421,076 cells** (the coarse mesh is 1.0 M, the medium 5.76 M). Level-5 plate surface (0.0078 c) as coarse; chord-aligned refinement boxes on the upper side: level 7 (0.00195 c) for x/c -0.04 to 0.20 up to about 0.02 c above the upper surface (the leading-edge separation onset), level 6 (0.0039 c) for x/c -0.07 to 0.85 up to about 0.05 c above the surface (the coarse reversed zone reaches 0.58-0.75 c at 6 deg). Six near-wall layers, expansion 1.2, final layer 0.5 of the local cell: first layer 0.0004 c at level 7 and 0.0008 c at level 6 (mean first-layer thickness 0.00071 c, total layer thickness 0.0071 c, 99 % of faces extruded), which puts the first cell centre at y+ of about 1 on the upper surface; the measured y+ is added to this file after the first written time. The lower surface and the trailing edge region stay at coarse resolution (level 5) with y+ of about 3 as in the coarse run.
- First attempt at the mesh (10 absolute layers, 6.06 M cells) failed layer addition (65 % extruded, negative-volume cells) and was discarded; the second (above) is the one used. checkMesh (default): Mesh OK, max non-orthogonality 64.9, max skewness 2.70, max aspect ratio 8.0 (`-allGeometry` flags 54,875 concave cells as in the coarse case).
- Start: the coarse laminar 6 deg solution (t = 20 of `vp_sweep_a06`, mapped with cellPointInterpolate; read only) so that the settling stage is short. Time step about 0.0007 (Courant 1.5, set by the level-7 cells), about 1430 steps per chord passage.
- Measured speed (steady, from the first minutes): 4.2 s per step, **about 5900 s (1.6 h) per chord passage**. Run to t = 20 (about 33 h, finish about 2026-10-14 18:40 Windows time). A first start with endTime 22 was aborted after 0.1 chord and restarted with endTime 20 to stay inside the 24-36 h budget; same mesh and settings.
- Averaging: `fieldAverage` of U, p and wallShearStress from t = 8 (written once per chord passage, all times kept), plus force coefficients every step; the averaging window for the criteria is chosen from the force histories after the run (at least 10 chord passages after settling, so t = 8-20 if settled by 8; if the flow has not settled by then the window is shortened and this is stated, not hidden).
- Status: `wsl -d MMAV-CFD -u cfd -- bash /home/cfd/run/ref/ref_status.sh`; stop: `... bash /home/cfd/run/ref/ref_stop.sh`.

## Result (2026-10-14; the criteria and outcome bands above are unchanged)
STATUS: finished. The run reached t = 20 (27,799 steps, 35.6 h, peak memory 7.8 GB); the averaging window is t = 8-20 (12 chord passages); the halves agree (CL 0.26 %, CM 0.0003), so the validity conditions are met. Mean y+ on the plate is 1.21 (maximum 17.9, location not identified). Full numbers, tables and limits: `result.md`.

**Verdict: PARTIAL.**
| Criterion | Value | Result |
|---|---|---|
| CM between +0.010 and +0.020 | +0.0115 | met |
| x_cp between 0.195 c and 0.225 c | 0.2157 | met |
| CL at or above 0.357 | 0.3343 | not met |
PASS needs all three, so it is not PASS; CM closes 82 % of the gap from -0.004 to the measured +0.0150 without the pass being met, which is PARTIAL. The "CL within 3 % of AVL" substitution was not invoked (AVL is 6.4 % below the coarse CFD lift at 6 deg), so the CL criterion stays as written.

**Consequence (as written above, unchanged):** resolution matters, nothing is validated. CFD is recorded as **not usable for moment**; lift is a rough check with a stated bias; **no further CFD unless Kai asks**. The rule on balance, pull-out and Kai's wings is unchanged: no CFD on the fabricated wings, and CFD is not used for balance or pull-out.

**Facts.** CM +0.0115; x_cp 0.2157; the reversed zone ends (reattachment) by about 0.33 c at all three span stations (time-average zones end at 0.33, 0.32 and 0.27 c; the coarse run was reversed out to 0.75 c at 25 % span and 0.58 c at 50 %). One refinement moved CM by 0.0155 and CD by 20 % (0.0483 to 0.0388), so the result is **not shown to be mesh-independent**. CD is 13.4 % below the measured mean (0.0388 against 0.0448); drag is not judged.

**Lift gap: unresolved.** CL 0.3343 is 11.1 % below the measured mean (the coarse run was 7.3 % below). Explanations still open: (a) a tunnel or correction offset; (b) the real wing at this Reynolds number lifts more than the CFD captures. Evidence bearing against a rig-only reading: the tunnel's lift-slope excess over AVL rises from about +11 % at Re 60k to about +24 % at Re 120k (`research/plate_data/README.md` fits, against AVL's secant slope at 4 deg); a rough wall-correction size of about 1 % of angle at 4 deg (an unverified reviewer estimate); and Laitone's report of lift slopes more than 20 % above lifting-line theory at Re 40,000 (`research/bird_research_notes.md` section 6.3, secondary; the same section summarises Pelletier & Mueller as finding lifting-line theory adequate, so the literature is not uniform). AVL agreeing with the CFD is two models agreeing, not validation.

**Post-processing copies.** `/home/cfd/run/diag_ref` is a copy of the t = 20 fields (mesh, p, pMean, wallShearStress, wallShearStressMean) used for the time-averaged shear post-processing. It was kept, together with the earlier copies `/home/cfd/run/diag`, `diag_lm` and `diag_a06src`; nothing was deleted.
