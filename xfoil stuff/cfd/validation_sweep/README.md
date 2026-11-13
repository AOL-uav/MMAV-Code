# Overnight AoA sweep, Ananda AR 3 flat plate, Re 60k (CFD)

Cases `vp_sweep_aXX` in distro MMAV-CFD under `/home/cfd/run/` (case data stays there, not on OneDrive). `vp_a4_coarse` and `vp_a4_medium` are untouched.

## Method (same as the alpha-4 coarse case)
- The angle is set by **rotating the plate geometry** (STL, nose-up about the quarter-chord point); the inflow stays along +x. So lift = +z, drag = +x, pitch axis = +y, CofR = (0.25 0 0) in `forceCoeffs` do not change with angle. Checked on alpha 2: STL chord line is 2.000 deg, controlDict directions identical to alpha 4.
- Everything else is identical to the alpha-4 coarse recipe: `scripts/mkcase.sh` regenerates the same dictionaries (the alpha-4 controlDict content was diffed against a regenerated one: same), same blockMesh + snappyHexMesh dictionaries and refinement boxes, laminar pimpleFoam, backward in time, `limitedLinearV 1`, max Courant 1.5, 12 MPI ranks. Because the plate is re-meshed per angle, cell counts differ slightly from 1,003,245 (alpha 2: 992,227). The refinement boxes are fixed in space; at 12 deg the trailing-edge tip dips to z = -0.156 c, just below the tip box (z = -0.12), so there the tip region is covered only by the plate-distance shell.
- Queue, in order: 2, 6, 8, 0 deg (20 chord passages each, average t = 8-20, as for alpha 4), then 10 and 12 deg (40 chord passages, average and fluctuation over the last half, t = 20-40).
- A case that fails (mesh not OK, solver crash) gets a `FAILED` file in its case folder and the queue continues. A finished case gets `DONE`.

## Commands (run in PowerShell; from Git Bash prefix with MSYS_NO_PATHCONV=1)
- Progress, one command: `wsl -d MMAV-CFD -u cfd -- bash /home/cfd/run/sweep/sweep_status.sh`
- Stop, one command: `wsl -d MMAV-CFD -u cfd -- bash /home/cfd/run/sweep/sweep_stop.sh`
- Log: `/home/cfd/run/sweep/driver.log` (from Windows: `\\wsl.localhost\MMAV-CFD\home\cfd\run\sweep\driver.log`). Per-case logs: `/home/cfd/run/sweep/vp_sweep_aXX.{mkcase,mesh,solve}.log` and `log.pimpleFoam` in each case.
- Results table: `python collect_sweep.py` in this folder; writes `sweep_results.csv` (CFD mean and fluctuation, measured values interpolated to the same angle on the up and down legs and their mean, and the differences). Re-run it at any time; unfinished cases show as `partial`.
- Restart after an interruption: `powershell Start-Process -WindowStyle Hidden wsl.exe -ArgumentList '-d','MMAV-CFD','-u','cfd','--','bash','/home/cfd/run/sweep/sweep_launch.sh'`. Finished cases are skipped; the interrupted case restarts from scratch.

## What keeps the distro alive
The driver runs under `setsid nohup` inside MMAV-CFD, so closing a terminal does not stop it. WSL can still shut a distro down when no Windows-side `wsl.exe` is attached. The launch command therefore runs `sweep_launch.sh` in a hidden Windows `wsl.exe` process, which waits for the driver to exit and so keeps the distro attached. If that Windows process is killed, or Windows is logged off, restarted or put to sleep, the distro can stop and the job dies. No change to `.wslconfig` or any global WSL setting was made. Do not run `wsl --shutdown` while it is running.

## Comparison caveats
- Measured data (`research/plate_data/ananda2012_flatplate_AR3_taper1.csv`, run 0383ar, Re 59999.4) is tunnel-corrected, from a tip-sting-mounted full-span wing, with measured angles landing at values like 3.48 and 4.54 deg. CFD is a free-air half model with square wing tip (tip shape not stated in the source), exact alpha, CM about c/4 nose-up positive as in the table.
- The coarse mesh is a resolution-limited laminar solution. **Drag is not mesh-converged:** at alpha 4 the medium mesh gave CD 0.0242 against 0.0285 on the coarse mesh (15% lower, still falling when stopped), see `../VALIDATION_SCOPING.md`. So the CD agreement in the table (within about 1-8%) must not be read as validation.
- **CM:** the earlier statement that CM is 'only good to about 0.001-0.002' was wrong as a description of the sweep. CFD minus measured CM is -0.003 (2 deg), -0.005 (4), -0.019 (6), -0.043 (8), -0.035 (10), -0.031 (12): errors of 0.02-0.04 from 6 degrees up, and CFD CM is negative at 6 and 8 degrees where the measurement is still positive. (The 0.001-0.002 figure was only the run-to-run drift between coarse and medium at alpha 4.)
- **Lift:** CFD CL is below the measurement at every angle, by -11.6% (2), -11.2% (4), -7.3% (6), -6.2% (8), -8.5% (10), -9.1% (12). This is a consistent bias of unknown cause. At alpha 4 the coarse and medium meshes gave the same CL (0.2189 and 0.2190; the medium run was unsettled), so mesh resolution alone has not been shown to explain it, but nothing in these runs identifies the cause either.
- Measured legs (up and down) differ by up to about 0.004 in CD near 4 deg; that is a fair repeatability indicator.

## Files
`collect_sweep.py`, `sweep_results.csv` (regenerated each time the script runs), `scripts/` (copies of the files in `/home/cfd/run/sweep/`: `sweep_driver.sh`, `sweep_launch.sh`, `sweep_stop.sh`, `sweep_status.sh`, `mkcase.sh`, `mesh.sh`, `solve.sh`).

## Diagnostics added after the sweep (existing fields only, no new flow solves)

Symptoms and causes are kept apart below: these files describe what the CFD fields show. None of them establishes why the CFD differs from the measurement.

### 1. Separation / reattachment extent (alpha 2, 4, 6, 8)
Files: `bubble_extent.csv` (one row per angle, span station and data basis), `surface_profiles.csv` (the sampled Cp and wall shear), `plots/Cp_span25.svg`, `plots/Cp_span50.svg`, `plots/Cp_span75.svg`, `plots/shear_upper_span25.svg` (and 50, 75), scripts in `scripts/` (`extract_surface.py` for ParaView `pvbatch`, `bubble_analysis.py`).
- **What data exists:** only instantaneous snapshots. There are no time-averaged fields (no fieldAverage). Because of `purgeWrite 2`, only t = 15 and t = 20 survive for the 20-chord cases (alpha 10 and 12: t = 35 and 40, not used). So the numbers below average TWO snapshots (`mean2`), and the t = 15 and t = 20 values are listed separately in the CSV to show the snapshot-to-snapshot spread. They are not converged time averages.
- The two alpha-8 pictures in `cfd/learn/img/` (`a08_slice_speed.png`, `a08_plate_pressure_top.png`) are single snapshots at t = 20, not averages.
- Wall shear came from `pimpleFoam -postProcess -func wallShearStress` run on copies of the t = 15 and 20 fields in `/home/cfd/run/diag/` (nothing was written into the `vp_*` cases). The sign was fixed by the lower surface, which is attached in all four cases: OpenFOAM's `wallShearStress` is negative along the flow there, so on the upper surface a positive value means reversed flow. Cp = 2 p (kinematic p, free-stream 0, U = 1).
- Method: upper-surface shear along the chord at 25, 50 and 75 % of the semi-span, moving-averaged over +-0.03 c, ignoring the nose (x/c < 0.03); a reversed zone is a stretch of at least 0.05 c with reversed flow; separation = start of the first zone, reattachment = its end. The mesh is 0.008 c near the plate, so a bubble of a few percent chord is under-resolved, and zone ends are good to several percent chord at best.

| alpha | span | separation x/c | reattachment x/c (first zone) | reversed fraction of upper chord | reversed zones, all (x/c) |
|---|---|---|---|---|---|
| 2 | 25 / 50 / 75 % | none | none | 0.00 | none (attached) |
| 4 | 25 % | 0.07 | 0.45 | 0.39 | 0.07-0.45 |
| 4 | 50 % | 0.08 | 0.44 | 0.37 | 0.08-0.44 |
| 4 | 75 % | 0.10 | 0.38 | 0.30 | 0.10-0.38 |
| 6 | 25 % | 0.05 | 0.28 | 0.65 | 0.05-0.28; 0.36-0.75 |
| 6 | 50 % | 0.05 | 0.58 | 0.55 | 0.05-0.58 |
| 6 | 75 % | 0.06 | 0.29 | 0.24 | 0.06-0.29 |
| 8 | 25 % | 0.05 | 0.24 | 0.84 | 0.05-0.24; 0.30-0.47; 0.54-trailing edge |
| 8 | 50 % | 0.05 | 0.23 | 0.60 | 0.05-0.23; 0.39-0.79 |
| 8 | 75 % | 0.05 | 0.16 | 0.31 | 0.05-0.16; 0.27-0.45 |

Reading it (symptoms only): at alpha 2 the upper surface is attached. At alpha 4 there is one reversed zone from just behind the nose to about 0.4 c, shorter toward the tip. From alpha 6 the reversed region is longer near the root and mid-span and breaks into several zones, and at alpha 8 at 25 % span it is still reversed at the trailing edge in one snapshot and in the two-snapshot mean. The t = 15 and t = 20 values for alpha 6 mid-span differ (first zone ends at 0.24 and 0.56), so the two-snapshot averages at alpha 6 and 8 are not stable.

### 2. Stationarity of the force histories
File: `window_means.csv` (script `scripts/window_means.py`). Each averaging window is split into four equal time intervals (t = 8-20 for alpha 0-8, t = 20-40 for alpha 10 and 12).

| alpha | CL Q1, Q2, Q3, Q4 | CD Q1..Q4 | CM Q1..Q4 | statement |
|---|---|---|---|---|
| 6 | 0.3488, 0.3451, 0.3554, 0.3453 | 0.0483, 0.0480, 0.0488, 0.0482 | -0.0039, -0.0031, -0.0060, -0.0029 | no trend; CM moves by about 0.003 between quarters |
| 8 | 0.4753, 0.4763, 0.4739, 0.4596 | 0.0789, 0.0787, 0.0792, 0.0777 | -0.0290, -0.0289, -0.0299, -0.0280 | CL falls 3.4% from Q1 to Q4, mostly in the last quarter |
| 10 | 0.5795, 0.5675, 0.5579, 0.5462 | 0.1166, 0.1157, 0.1158, 0.1134 | -0.0512, -0.0526, -0.0539, -0.0498 | CL drifts steadily down, -5.9% Q1 to Q4: not stationary |
| 12 | 0.6101, 0.5554, 0.5874, 0.6268 | 0.1565, 0.1482, 0.1553, 0.1596 | -0.0799, -0.0666, -0.0771, -0.0802 | no trend, but the quarters differ by up to 12% in CL (unsteady) |

Drift statement: alpha 6 and 8 are acceptable for the means quoted (quarter spread of CL within about 3%); alpha 10 has a clear downward CL trend over the window, so its mean depends on the window chosen; alpha 12 fluctuates strongly with no trend and needs a longer window than 20 chord passages for a mean good to better than a few percent. These are properties of the CFD histories, not comparisons with experiment.

### 3. Centre of pressure, CFD against measured
x_cp = 0.25 - CM/CN from the quarter-chord moment, with CN = CL cos(alpha) + CD sin(alpha); measured values are the interpolated values in `sweep_results.csv` (tunnel-corrected angle).

| alpha | CFD CN | CFD x_cp | measured CN | measured x_cp |
|---|---|---|---|---|
| 4 | 0.220 | 0.216 | 0.248 | 0.201 |
| 6 | 0.352 | 0.261 | 0.379 | 0.210 |
| 8 | 0.478 | 0.311 | 0.508 | 0.223 |
| 10 | 0.574 | 0.340 | 0.626 | 0.277 |
| 12 | 0.614 | 0.374 | 0.673 | 0.317 |

The CFD centre of pressure moves aft faster than the measured one from 6 degrees up (0.31 against 0.22 chord at 8 degrees). That is a description of the symptom; the cause is not established. The separated-flow extent in item 1 (a longer and more broken reversed region at 6 and 8 degrees) is a candidate to examine, not an explanation.

### 4. What the Ananda thesis says about the flow state (searched, `Ananda_2012_MS_thesis_UIUC.pdf`; page numbers are PDF pages, printed page = PDF - 16)
- Oil-flow visualisation in the thesis covers only three models, none of them this plate at Re 60k (p. 52, printed 36): Wortmann FX 63-137 A4 at Re 90,000; flat plate A2 taper 1 at Re 140,000; flat plate A2 taper 0.5 at Re 120,000. No bubble length is given for the AR 3 plate. The oil-flow pictures of the Wortmann wing show a long laminar bubble that shortens with angle (pp. 118-120).
- Flat-plate stall (pp. 122-123, printed 106-107): "Stall for the flat-plate wings occurs as a result of laminar separation"; "The lack of a critical Reynolds number suggests that the laminar boundary (shear) layer that separates from the central region of the flat-plate wing was unable to reattach resulting in no laminar separation bubble formation" (this is the thesis' reading of the force data near stall, not an observation of a bubble).
- No linear-range statement or stall angle for the AR 3 plate at Re 60k was found as text; lift slopes are fitted to "the linear trendline" (p. 122).
- The journal paper (Aerosp. Sci. Technol. 42 (2015), p. 397, Sec. 3.1, Fig. 11) says differently from the thesis list: oil flow on the AR 3 plate at Re 60,000 at 5, 7 and 10 degrees shows a laminar separation bubble at the leading edge of the centre section, "the laminar boundary layer ... separates immediately ... then transitions to turbulent flow and reattaches itself some distance aft of the leading edge", the reattachment line moves forward as angle rises, and "at stall (alpha about 10 deg), flow over the center section of the wing separates completely". No bubble length is given. The paper and thesis therefore disagree on whether the AR 3 plate was oil-flowed; the paper's account is the only one that speaks to this case.


## Note 2026-10-13: two discrepancies, and what the transition test does and does not show
(a) **Attached-flow gap.** At 2 deg the laminar CFD has no reversed flow at any span station, yet CL is 11.6 % below measured (0.1047 against 0.1184) and x_cp is about 0.019 c aft of measured (0.207 against 0.188; x_cp = 0.25 - CM/CN from `sweep_results.csv`). The medium mesh gave the same CL at 4 deg as the coarse one. Cause **unexplained**.
(b) **Aft shift from 6 deg.** CFD x_cp 0.261 (6 deg), 0.311 (8) against measured 0.210, 0.223, coinciding with long reversed zones. Cause **unverified**.
The kOmegaSSTLM run in `transition_A/` was **not a fair test of the model**: the bounding-k, bounding-omega and lambda-iteration messages occur on every step to the end of both runs; y+ is 2.8-2.9 on average (wall functions) against the model's roughly 1; at 4 deg it never switched on although the laminar solution has reversed flow over about 38 % of the chord. It is not evidence that the model fails.

## Setup checks (step 1 of the reviewer revision), 2026-10-13: results in `setup_checks/`
Written before any new CFD was started. No setup error was found.

### 1. AVL on the rectangular AR 3 flat planform (`setup_checks/avl_check.py`, `avl_results.csv`, `avl/*.avl`, `avl/*.out`)
Chord 1, span 3, no sweep or taper, zero thickness, Mach 0, moment reference (0.25, 0, 0), AVL 3.52. Two panelings (8 chordwise x 12 spanwise per half and 24 x 40 per half, cosine spacing) agree to 0.00002 in CL.
| alpha | AVL CL | CFD (coarse laminar) | CFD vs AVL | measured | measured vs AVL |
|---|---|---|---|---|---|
| 2 | 0.1097 | 0.1047 | -4.6 % | 0.1184 | +7.9 % |
| 4 | 0.2190 | 0.2189 | -0.1 % | 0.2465 | +12.5 % |
| 6 | 0.3276 | 0.3486 | +6.4 % | 0.3761 | +14.8 % |
| 8 | 0.4349 | 0.4713 | +8.4 % | 0.5023 | +15.5 % |
AVL lift slope 3.054 /rad (0.0533 /deg); measured slope (fit over -6..6 deg in `../research/plate_data/README.md`) 0.0607 /deg, 14 % higher. AVL neutral point 0.2249 c (inviscid, linear). AVL x_cp at all four angles equals 0.2246 to 0.2249, as expected for a linear flat plate; CFD x_cp is 0.207 (2 deg) and 0.216 (4 deg), measured 0.188 and 0.201.
Finding: **the coarse CFD reproduces linear inviscid theory at 4 deg exactly, and the measured wing lifts about 8-15 % above it** (rising with angle). This describes the symptom; it does not say why the tunnel wing lifts more, and it does not make the CFD lift deficit a CFD error. Above 4 deg the CFD itself rises above AVL (+6.4 % at 6 deg, +8.4 % at 8 deg).

### 2. Geometry as meshed, measured on the STL files (`setup_checks/stl_check.py`, `stl_check_output.txt`; vp_a4_coarse and vp_sweep_a08)
Chord coordinate 0.0000 to 1.0000; span coordinate -0.100 to 1.500 (the 0.1 c beyond the root pokes out through the symmetry plane and is outside the domain); maximum thickness 0.04286 c (Ananda: 0.15 in / 3.5 in = 0.04286, the 4.3 %); symmetric (camber 0). Leading-edge and trailing-edge thickness profiles lie within 0.0006 c of the 5:1 (semi-major 0.1071 c) and 10:1 (semi-major 0.2143 c) ellipses (the flat-section ends measured at 0.1025 and 0.795 c are the 0.1 % thickness threshold of those ellipses, not a mismatch). Tip shape (not stated in the sources): a flat square cut.

### 3. Force setup (`vp_a4_coarse/system/controlDict`; `setup_checks/patch_check.py`, `patch_check_output.txt`)
forceCoeffs: patch `plate`, rhoInf 1, magUInf 1 (free stream along +x, speed 1, kinematic pressure), liftDir (0 0 1), dragDir (1 0 0), CofR (0.25 0 0), pitchAxis (0 1 0), lRef 1, Aref 1.5 (half-wing planform, = chord x half span). All as intended. The integrated patch (alpha 4 mesh) has bounds x 0.0008-0.998, y 0-1.5, z -0.063 to 0.033, 50,384 faces and total area 3.061; it **includes the tip face** (656 faces, area 0.038, normals along -y out of the fluid, i.e. the flat tip cap at y = 1.5) and has no root face (the root is the symmetry plane). So tip-cap pressure is in CL, CD and CM, as for the real wing.

### 4. Blockage
Domain cross-section for the half model: 7.5 c x 16 c = 120 c^2. The reviewer's ratio 1.5/120 = 1.25 % is the planform area (half wing 1.5 c^2) over that cross-section; the physical blockage is the frontal area: (sin(alpha) + 0.0429 cos(alpha)) x 1.5 c^2 = 0.169 c^2 (4 deg), 0.221 (6 deg), 0.272 (8 deg), i.e. 0.14 %, 0.18 %, 0.23 % of 120 c^2 (the thickness alone, 0.064 c^2, is 0.05 %). The CFD has slip walls at 8 c above and below and 6 c beyond the tip. The tunnel: 2.8 x 4.0 ft = 0.853 x 1.219 m = 1.040 m^2 = 131.6 c^2 (c = 0.0889 m); the full wing planform (3 c^2) is 2.3 % of that.

### 5. What the thesis says about its corrections (`Ananda_2012_MS_thesis_UIUC.pdf`, PDF pp. 57-59 = printed 41-43, Sec. 3.8)
Four effects are named: buoyancy (neglected), solid blockage, wake blockage (Maskell) and streamline curvature. Quoted equations: "q_inf,c = q_inf,u (1 + eps_T)^2" and "V_inf,c = V_inf,u (1 + eps_T)" with eps_T = eps_sb + eps_wb (Eq. 3.23-3.25); streamline curvature (p. 59): "alpha_c = alpha_u + delta_SC (1 + tau_2) C_Lc" (3.26), "C_Dc = D_u/(q_inf,c S) + delta_SC C_Lc^2" (3.27), "C_Mc = M_u/(q_inf,c S c) + 0.125 tau_2 delta_SC (dC_Lc/dalpha_c)" (3.28). These pages give no numerical sizes for eps_sb, eps_wb, delta_SC or tau_2 and no statement of their net effect on lift slope or moment, so the size and sign of the effect on CL and CM at 2-8 deg is **not established here**. The equations show only that the corrected angle is shifted by a term proportional to C_L, a term proportional to dC_L/dalpha is added to the moment, and the dynamic pressure is raised by the blockage factor.

### 6. Consequences for the reviewer's decision rule
No setup error was found, so no laminar 4 deg repeat was run. The attached-flow lift gap is not a CFD setup mismatch against linear theory (CFD = AVL at 4 deg); whether it is a reference/correction mismatch cannot be settled from the thesis text. The moment problem remains (CFD CM -0.004 at 6 deg, measured +0.015), so step 2 proceeds.

## Early-stop rule for future runs (added 2026-10-16)
**Applies to a run only when it is written into that run's README before launch.** Proposed by the reviewer after the refined 8 deg run. That run was stopped early on Kai's authorisation, and this rule was applied to it after the fact (`refined_8deg/README.md`, "Result"; margin 0.0021 against 3 x standard error 0.0009 and halves difference 0.0008).
1. **Settled from t_s:** the first time after which every one-passage bin of CL, CM and the graded quantity stays within its band of the remaining bins (CL 2 %, CM 0.002, x_cp 0.003 c).
2. **The run may stop at the end of a bin only when all of these hold:**
   - at least 5 one-passage bins after t_s;
   - the halves test passes on those bins (CL 2 %, CM 0.002);
   - the distance from the window mean to the nearest band edge is at least 3 x (bin standard deviation / sqrt(number of bins)) and at least the halves difference.
3. **If any condition fails,** continue to the end time fixed in advance.
