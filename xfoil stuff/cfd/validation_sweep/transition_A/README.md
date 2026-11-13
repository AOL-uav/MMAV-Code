# Transition model test (option A): kOmegaSSTLM, Ananda AR 3 plate, Re 60k, alpha 4 and 8

STATUS: results written 2026-10-13. Both kOmegaSSTLM runs finished at t = 30 chord passages. This is a discriminating test on the existing coarse mesh; it is descriptive only, with no judgement on usability.


## Wording correction (reviewer revision, 2026-10-13)
There are TWO separate discrepancies between the CFD and the measurement, not one:
- **(a) A low-angle lift and moment gap in fully attached flow.** At 2 deg the laminar run has no reversed flow at any of the three span stations, yet CL is 11.6 % below measured (0.1047 against 0.1184) and the centre of pressure is about 0.019 c aft of measured (x_cp 0.207 against 0.188, recomputed from `sweep_results.csv` with x_cp = 0.25 - CM/CN). The coarse and medium meshes gave the same CL at 4 deg (0.2189 and 0.2190), so resolution has not been shown to explain it. **Cause unexplained.**
- **(b) A premature aft shift of the centre of pressure from 6 deg upward** (CFD 0.261 at 6 deg and 0.311 at 8 deg against measured 0.210 and 0.223), which coincides with long reversed zones on the upper surface. **Cause unverified.**

The kOmegaSSTLM run below is **not a fair test of the transition model**, so it must not be read as the model failing, for three reasons: (1) the "bounding k", "bounding omega" and "Number of lambda iterations exceeds maxLambdaIter(10)" messages appear on every time step to the end of both runs; (2) the mesh gives y+ mean 2.8-2.9 (maximum about 19) with wall functions, where the model wants about 1; (3) at 4 deg the model never switched on (nut/nu stayed below 0.31) although the laminar solution has reversed flow over about 38 % of the chord. What the run does show is described in sections 1-5 below, and no more.

## What was run
Cases in MMAV-CFD: `/home/cfd/run/vp_lm_a04` (mesh and setup from `vp_a4_coarse`) and `/home/cfd/run/vp_lm_a08` (from `vp_sweep_a08`); the laminar cases were not modified. Same mesh, U and p boundary conditions, force coefficients (CofR 0.25 0 0, pitch axis +y, Aref 1.5), time scheme, momentum scheme and Courant limit 1.5 as the laminar sweep. Changes: RAS `kOmegaSSTLM` (unsteady pimpleFoam); fields k, omega, nut, ReThetat, gammaInt; `div(phi,k|omega|gammaInt|ReThetat) Gauss limitedLinear 1`; 6 MPI ranks per case (both at once); endTime 30 (sweep: 20). Nothing was changed after the first attempt and nothing was tuned.
Inlet (c = 1, U = 1, nu = 1/60000): Tu = 0.1 % (ASSUMED: the Ananda paper says the tunnel turbulence is "less than 0.1%"; no value is stated in `research/plate_data` or `VALIDATION_SCOPING.md`), nut/nu = 10 (ASSUMED), so k = 1.5e-6, omega = 0.009, nut = 1.667e-4, ReThetat = 1136.5 (the model's own correlation at Tu = 0.1 %, F_lambda = 1), gammaInt = 1. Wall: k = 0, omegaWallFunction, nutkWallFunction (as in the T3A tutorial).
Files: `setup_a08/` (dictionaries), `scripts/` (build, run, status, `extract_lm.py`, `analyze_lm.py`, `analyze_lm_output.txt` = printed tables), `transition_results.csv`, `bubble_extent_lm.csv`, `transition_profile.csv`, `surface_profiles_lm.csv`, `near_surface_lm.csv`, `plots/`.

## 1. Forces (window t = 8-20, the laminar sweep window; mean, then sd of fluctuation in brackets)
| alpha | source | CL | CD | CM (c/4, nose-up +) | x_cp |
|---|---|---|---|---|---|
| 4 | measured (Ananda, interpolated) | 0.2465 | 0.0282 | +0.0122 | 0.201 |
| 4 | laminar CFD | 0.2189 (0.0015) | 0.0285 (0.00008) | +0.0075 (0.0005) | 0.216 |
| 4 | kOmegaSSTLM | 0.2181 (0.0012) | 0.0284 (0.00007) | +0.0076 (0.0004) | 0.215 |
| 8 | measured | 0.5023 | 0.0746 | +0.0137 | 0.223 |
| 8 | laminar CFD | 0.4713 (0.0181) | 0.0786 (0.0017) | -0.0290 (0.0071) | 0.311 |
| 8 | kOmegaSSTLM | 0.4381 (0.0004) | 0.0704 (0.00006) | -0.0036 (0.0002) | 0.258 |

x_cp = 0.25 - CM/CN, CN = CL cos(alpha) + CD sin(alpha), as in `../README.md`.
Drift of the kOmegaSSTLM runs, t = 10-20 against t = 20-30: alpha 4: CL 0.2185 / 0.2177 (-0.4 %), CD 0.02843 / 0.02839, CM +0.0075 / +0.0078 (+0.0003). Alpha 8: CL 0.4382 / 0.4379 (-0.1 %), CD 0.07042 / 0.07038, CM -0.00358 / -0.00342 (+0.0002). Means over t = 10-30: alpha 4 CL 0.2181, CD 0.0284, CM +0.0077; alpha 8 CL 0.4380, CD 0.0704, CM -0.0035. Both runs are stationary to better than 0.5 % in CL and CD; the alpha-8 force history is almost steady (sd of CL 0.0004 against 0.018 in the laminar run).

## 2. Against the laminar run (what changed)
- **Alpha 4: essentially unchanged.** CL, CD, CM and x_cp differ from laminar by less than 0.001 in the coefficients; the reversed zone is the same (below).
- **Alpha 8: CM much closer to measured** (-0.0036 against measured +0.0137; laminar was -0.0290), and x_cp 0.258 against measured 0.223 (laminar 0.311). The CM sign still differs from the measurement.
- **Alpha 8: CL further from measured** (0.438 against 0.502, -12.8 %; laminar 0.471, -6.2 %). **CD moved from 5.4 % above to 5.6 % below measured** (0.0704 against 0.0746; laminar 0.0786).
- **Alpha 8: unsteadiness removed** (CL sd 0.0004 against 0.018; CM sd 0.0002 against 0.0071).
- Unchanged: the CFD CL remains below the measurement at both angles (alpha 4: -11.5 %).

## 3. Reversed flow on the upper surface (same method as `../bubble_extent.csv`: `bubble_extent_lm.csv`)
Zones of reversed flow, x/c, from the mean of the two saved snapshots (t = 25 and 30; the laminar numbers come from t = 15 and 20). Only two snapshots per case exist (`purgeWrite 2`), so this is not a long time average. The t = 25 and t = 30 snapshots are identical at alpha 8 (the flow is steady), and differ by 0.01-0.04 chord at alpha 4.
| alpha | span | laminar | kOmegaSSTLM |
|---|---|---|---|
| 4 | 25 % | 0.07-0.45 | 0.07-0.46; 0.93-TE |
| 4 | 50 % | 0.08-0.44 | 0.08-0.44 |
| 4 | 75 % | 0.10-0.38 | 0.10-0.39 |
| 8 | 25 % | 0.05-0.24; 0.30-0.47; 0.54-TE | 0.05-0.78 |
| 8 | 50 % | 0.05-0.23; 0.39-0.79 | 0.05-0.64 |
| 8 | 75 % | 0.05-0.16; 0.27-0.45 | 0.05-0.35 |
At alpha 8 the broken, multi-zone pattern of the laminar run becomes one reversed zone per station, which ends (reattaches) before the trailing edge at all three stations (0.78, 0.64, 0.35 chord); at 25 % span the laminar run was still reversed at the trailing edge. At alpha 4 the reversed zone is unchanged (the extra 0.93-TE piece at 25 % span is a short stretch at the trailing edge). The zone end is longer than the laminar first zone at alpha 8 (the laminar zones were shorter but fragmented), so the total reversed fraction is about 0.76 / 0.62 / 0.31 against 0.84 / 0.60 / 0.31.

## 4. Was the transition model active? (`transition_profile.csv`, `near_surface_lm.csv`, plots `transition_alpha*.svg`)
Near-surface band (up to 0.03 c above the upper surface, mid-span, mean of t = 25 and 30):
- **Alpha 4: it never triggered.** The largest nut/nu in the band is 0.08-0.31 (never above 1) along the whole chord, k stays below 7e-4, gammaInt is about 0.02-0.1 (mean 0.49 only in the first 0.1 c). The solution behaves as laminar, which is consistent with the unchanged forces. This is a different finding from a model that triggers and does not help.
- **Alpha 8: it triggered.** Max nut/nu in the band rises from 3.5 (x/c 0-0.1) to 31 (0.1-0.2) and to 52-67 over x/c 0.2-1.0; max k reaches about 0.05 (Tu about 18 % in the shear layer); mean gammaInt rises from 0.35 to 0.65-0.8. So the model switches on in the shear layer between about 0.05 and 0.2 chord and stays on to the trailing edge.

## 5. Run diagnostics
- y+ on the plate, written at t = 30: alpha 4 min 0.21, mean 2.77, max 18.8; alpha 8 min 0.18, mean 2.94, max 19.5. The model wants y+ of about 1; this coarse mesh is above that over most of the plate (mean 2.8-2.9, maximum near 19 at the leading edge) with wall functions in use. This is a limitation of the test, not re-meshed.
- Freestream turbulence at the plate (probe at x = -0.3, y = 0.75, z = 0.5): k = 1.504e-6 (alpha 4) and 1.510e-6 (alpha 8) against 1.5e-6 at the inlet, i.e. Tu 0.100 % at the plate: essentially no decay. The probe at the inlet end (x = -9.5) stays at 1.499e-6. The nut at the x = -0.3 probe is 4.2e-6 (alpha 4) and 2.5e-6 (alpha 8), i.e. nut/nu about 0.25 and 0.15, far below the inlet value 10, so the eddy-viscosity ratio does not stay at its inlet value near the plate.
- Time step and Courant: maximum Courant 1.50 throughout; mean dt 0.00355 (alpha 4, 8450 steps) and 0.00225 (alpha 8, 13,345 steps). No solver instability or crash in either run.
- Wall time (6 ranks each, run concurrently): alpha 4 22,757 s (6.3 h), alpha 8 28,518 s (7.9 h). Memory: both runs together peaked at about 5.0 GB.
- **Start-up warnings persisted for the whole run.** In every third of the run (t 0-10, 10-20, 20-30) there is one "Number of lambda iterations exceeds maxLambdaIter(10)" warning, one "bounding k" line and one "bounding omega" line on every time step (alpha 4: 2813 of 2813 steps in the last third; alpha 8: 4465 of 4465). They did not die out after the start-up.

## 6. Plots (dark SVG, `plots/`)
`Cp_alpha4_lam_vs_LM.svg`, `Cp_alpha8_lam_vs_LM.svg` (mid-span Cp, upper solid and lower dashed); `shear_alpha4_lam_vs_LM.svg`, `shear_alpha8_lam_vs_LM.svg` (upper-surface shear); `CL_history_alpha4.svg`, `CL_history_alpha8.svg`, `CM_history_alpha4.svg`, `CM_history_alpha8.svg`; `transition_alpha4.svg`, `transition_alpha8.svg` (log10 of max nut/nu and three times min gammaInt against chord).

## Not verified
- The Tu and nut/nu inlet values are assumptions; no other values were tried.
- Only two saved snapshots per case; shear sign convention as in `../README.md` (lower surface attached); zone ends are good to several percent chord on this mesh.
- The SVG plots were checked only for valid XML, not looked at.
