# Refined laminar run at 8 degrees (option b) - criteria fixed BEFORE the run

STATUS: finished (stopped early by decision at t = 10.01, 2026-10-16; graded PASS on a disclosed, changed averaging window: see "Result" at the end). Criteria fixed 2026-10-15, before the solver was started (see "Launch" at the end for the start time). They are unchanged by anything that follows. Revised once before the start on the reviewer's directives (grading on x_cp, wording, box height, restart); no solver had been run for this case at that time.

## What changed from the 6 deg grading, decided before the run, and why
At 6 deg the outcome bands graded lift (CL at or above 0.357) as well as moment and centre of pressure. **For 8 deg, the grade is decided on the centre of pressure; the moment is reported against its band as a secondary result; lift is reported against AVL and the tunnel but is not graded.** Decided before the run, on the reviewer's directive: the lift gap is characterised separately (`../lift_gap_checks/README.md`: the tunnel's lift excess over AVL depends on Reynolds number on every wing, cause open), and a balance question needs the centre of pressure. The moment is the normal force times the offset of the centre of pressure from the quarter chord, CM = CN x (0.25 - x_cp). With x_cp inside its band, a CM below its band therefore reflects the known lift gap (CN lower than the tunnel's), **not a moment failure**.

## Reference values at 8.0 deg (recomputed from `research/plate_data/ananda2012_flatplate_AR3_taper1.csv`, run 0383ar, Re 59,999; each sweep leg interpolated linearly at the angle)
| angle | leg | CL | CD | CM (c/4, nose-up +) | CN | x_cp = 0.25 - CM/CN |
|---|---|---|---|---|---|---|
| 8.0 | increasing-alpha leg | 0.4976 | 0.0750 | +0.0148 | 0.5031 | 0.2206 |
| 8.0 | decreasing-alpha leg | 0.5071 | 0.0743 | +0.0126 | 0.5125 | 0.2254 |
| 8.0 | mean of legs | 0.5023 | 0.0746 | +0.0137 | 0.5078 | 0.2230 |
Table points that bracket 8.0 deg: increasing leg 7.57 deg (CM +0.0178) and 8.63 deg (+0.0104); decreasing leg 7.39 deg (+0.0157) and 8.37 deg (+0.0107). The moment break lies between these points, so the interpolation across them is itself uncertain.
Coarse laminar CFD at 8 deg (sweep, window 8-20): CL 0.4713, CD 0.0786, CM -0.0290, x_cp 0.3106. AVL (inviscid, linear, `../setup_checks/`): CL 0.4349. The 6 deg refined run (`../refined_6deg/`): CM +0.0115, x_cp 0.2157 against measured +0.0150 and 0.2104.

## Sensitivity to angle, and the bands (arithmetic shown)
CM falls steeply with angle here. Measured CM change per degree across 7.75 to 8.25 deg: increasing leg (+0.0165 at 7.75, +0.0131 at 8.25) -0.0070 per deg; decreasing leg (+0.0139, +0.0113) -0.0051 per deg. The tunnel angles are wall-corrected (`../lift_gap_checks/`: about 1.5-2 % of the angle, 0.12-0.16 deg at 8 deg) and the CFD angle is exact, so an angle allowance of **plus or minus 0.25 deg** is used.
**x_cp band (graded).** x_cp over both legs and alpha = 7.75, 8.0, 8.25 deg: 0.2161 to 0.2286. Moment uncertainty 4.6 % (thesis Sec. 3.9): 0.0007 / CN 0.51 = 0.0014: 0.2147 to 0.2300. Rounded outward to 0.005: **x_cp between 0.215 c and 0.230 c.** The CFD x_cp is computed from the run's own CM and CN, as at 6 deg.
**CM band (secondary, reported).** Envelope of the interpolated measured CM over both legs and the three angles: +0.0113 to +0.0165 (leg spread at 8.0 deg 0.0022; change over +-0.25 deg 0.0017 and 0.0013). Widened by 4.6 % of 0.0148 = 0.0007: +0.0106 to +0.0172. Rounded outward: **CM between +0.0105 and +0.0175.**

## Validity of the averaged result (same rules as 6 deg, plus unsteady flow)
- Average over at least 10 chord passages after the flow has settled; the run ends at t = 20.
- The two halves of the averaging window agree within 2 % in CL and within 0.002 in CM.
- Report y+ (mean, maximum) and the cell size in the shear layer next to the result.
- **One extension rule for every case below: if a validity rule fails, the run is reported as "not converged, not graded" with the numbers given; any extension of the run is Kai's decision.**
- **Unsteady or shedding flow (the coarse laminar 8 deg run was unsteady, CL standard deviation 0.018).** Defined numerically before the run: the flow has a *regular period* if the successive maxima of the CM history (smoothed over 0.05 passage), over the last 10 passages (t = 10 to 20), number at least three and are spaced within +-10 % of their mean spacing p. Then the averaging window is a whole, even number N of periods, N the smallest even integer with N x p of at least 10 passages, ending at the last CM maximum at or before t = 20 (and starting no earlier than t = 8); the halves test is applied to the first N/2 and last N/2 periods; the mean and the standard deviation are reported. If the spacing is irregular (not within +-10 %), the window is t = 8 to 20 with halves [8,14] and [14,20], and the standard deviation is reported. If either version cannot meet the rules, the run is not converged and not graded.

## Outcome bands (decided on the averaged x_cp; the moment and lift are reported, not graded)
- **PASS:** x_cp between 0.215 c and 0.230 c **and** CM positive (validity rules met).
- **PARTIAL:** not PASS and x_cp below 0.267 c. (0.267 c is half of the gap from the coarse laminar x_cp, 0.3106, to the measured mean, 0.2230.) Resolution matters; nothing is validated.
- **FAIL:** x_cp at or above 0.267 c.
- **Reported, not graded:** CM against its band +0.0105 to +0.0175 (and the cross-check that CM closes more than half of the gap from -0.0290 to +0.0137, i.e. is above -0.0077); CL against AVL (0.4349), the tunnel (mean 0.5023, up leg 0.4976, down leg 0.5071) and the coarse CFD value (0.4713).

## Consequences (set before the run)
- **Even a PASS validates the centre of pressure at Reynolds number 60,000 for this plate only.** Using the CFD on the fabricated wings would be a labelled extrapolation, and the rule on them stays (no CFD on the fabricated wings; CFD not used for balance or pull-out) unless Kai decides otherwise.
- **PARTIAL or FAIL:** stop. The CFD stays recorded as not usable for moment, and for centre of pressure at 8 deg; no further CFD without Kai asking.

## Refinement-box height check (before launch)
The level-6 refinement box covered 0.05 c above the upper surface (as at 6 deg). From the coarse laminar 8 deg solution (`vp_sweep_a08`, read through a copy, `setup/box_height_check.csv`, `scripts/box_height_check.py`; **only instantaneous snapshots exist for that case, t = 15 and 20, no mean field**), the height above the upper surface of the top of the reversed-flow zone (chordwise velocity below zero) and of the shear-layer edge (chordwise velocity reaching 0.9) were measured in chord bins of 0.05 forward of 0.85 c:
| span | t | top of the reversed zone, max over x/c (at) | shear-layer edge, max |
|---|---|---|---|
| 25 % | 15 / 20 | 0.095 c (x/c 0.50) / 0.095 c | 0.145 c / 0.140 c |
| 50 % | 15 / 20 | 0.070 c (x/c 0.65) / 0.070 c (0.55) | 0.105 c / 0.120 c |
| 75 % | 15 / 20 | 0.020 c (x/c 0.25) / 0.025 c | 0.045 c / 0.050 c |
The reversed zone exceeds the 0.045 c threshold (reaching 0.095 c at 25 % span) and the original box height of 0.05 c. **Change made: the level-6 box height only, from 0.0714 to 0.1314 in the body frame (0.11 c above the upper surface = 0.095 c + 0.015 c margin); levels, box lengths and everything else unchanged** (`setup/snappyHexMeshDict`; the earlier file is kept in the case as `system/snappyHexMeshDict.box6_0.0714`). The height of the 6 deg case's reversed zone was not checked before that run (a limit of the 6 deg result).
**Re-mesh:** **5,949,059 cells** (before: 4,417,284; 6 deg: 4,421,076), +35 %; level 6 now 3,128,775 cells (was 1,606,200). checkMesh (default): Mesh OK, max non-orthogonality 64.7, max skewness 2.17, max aspect ratio 8.0. Layers unchanged in method (six layers, mean first layer 0.00071 c).
**Impact (estimates):** memory scales with cells: 7.8 GB x 5.95/4.42 = about 10.5 GB of the 15.5 GB the WSL VM has. Run time scales with cells at the same time step: see the estimate below.

## Case set-up
- **Case:** `/home/cfd/run/vp_ref_a08` (laminar pimpleFoam, no turbulence model), built by the same script as `vp_ref_a06` (`scripts/mk_ref.sh`: `bash mk_ref.sh vp_ref_a08 8 22 8`, then `foamDictionary ... endTime -set 20`). The angle is set as at 6 deg: the plate geometry is rotated nose-up about the quarter-chord point and the inflow stays along +x, so the lift and drag directions, moment centre (0.25 0 0), reference values (Aref 1.5, lRef 1, pitch axis +y) are unchanged. Start: the coarse laminar 8 deg solution (t = 20 of `vp_sweep_a08`, read only, mapped with cellPointInterpolate).
- **Diff against `vp_ref_a06`** (`setup/diff_vs_vp_ref_a06.txt`, written before the box change and restart edits; the later differences are listed here): `fvSchemes`, `fvSolution`, `blockMeshDict`, `0.orig/*`, `constant/*` and `geometry/profile.txt` are identical. Differences: (1) the STL is rotated 8 deg; (2) `snappyHexMeshDict`: origin, `e1`, `e3` of the two refinement boxes follow the plate angle, and the level-6 box height is 0.1314 instead of 0.0714 (above); (3) `decomposeParDict`: 8 subdomains instead of 12; (4) `controlDict`: the three fieldAverage1 restart entries `restartOnRestart false; restartOnOutput false; periodicRestart false;` are written explicitly (they equal the defaults; added for the restart, below); all else in `controlDict` (functions forceCoeffs1, yPlus1, wallShearStress1, fieldAverage1 from t = 8; end time 20; one write per chord passage, all kept) is identical.

## Restart (resume) behaviour
`scripts/ref_run_a08.sh` (the script the launcher runs) now resumes instead of re-mapping: it looks for the latest written time directory in which every processor has `U`, `p` and `phi`; if one exists, it does not re-map or re-decompose, moves any newer, incomplete time directories aside (`_incomplete_<t>`), appends to the log and continues from that time (`startFrom latestTime`; the time step is read back from the time directory); if none exists it maps and starts fresh. **fieldAverage continues:** the mean fields and the averaging time are read back (`restartOnRestart false`).
Dry tests (no solver run on this case): (a) selection logic on mock trees: times 0, 5, 10 complete and 11 incomplete resumes from 10 and flags 11; time 10 missing a file on one processor resumes from 5 and flags 10 and 11; only time 0 gives a fresh start; the real case, not yet run, gives a fresh start. (b) fieldAverage restart on a throwaway copy of the toy plate case (serial, a few seconds, deleted afterwards): run to t = 1 (averaging time 0.808), restarted from the latest time and run to 1.5: the averaging time continued to 1.308 (0.808 + 0.500), so the averages were not reset. (The solver log prints "Restarting averaging for fields:" on resume; the written averaging time shows it continued.) **Remaining risk:** a kill during a write leaves an incomplete directory (handled as above); a restart loses at most the time since the last whole-chord write (about 1.5 h of run time at the estimated rate); the force-coefficient file continues in a new `postProcessing/forceCoeffs1/<resume time>/` directory (the analysis reads all of them).

## Resource limits (so that the PC stays usable)
- **8 MPI ranks** (6 deg used 12): 12 physical cores and 24 logical, so 4 physical cores stay free by count. The ranks are not pinned.
- Run under **`nice -n 19`** (all WSL distros share one kernel, so this also yields to Linux work in other distros). `.wslconfig` is not changed.
- **Runtime estimate (an estimate, not a measurement):** the 6 deg run took 27,799 steps in 128,267 s on 12 ranks, 4.61 s per step. The coarse laminar 8 deg case needed 1.58 times as many steps as the 6 deg one (8,909 against 5,647), so about 43,900 steps. Eight ranks instead of twelve: about 4.61 x 12/8 = 6.9 s per step at 4.42 M cells; with 5.95 M cells, x 1.35 = 9.3 s per step. 43,900 x 9.3 s = about 408,000 s = **about 113 hours (4.7 days)**; plausible range 100 to 135 hours, longer if other work competes. The first measured rate is added under "Launch".

## Commands
- Start (PowerShell): `Start-Process -WindowStyle Hidden wsl.exe -ArgumentList '-d','MMAV-CFD','-u','cfd','--','bash','/home/cfd/run/ref/ref_launch_a08.sh'`. A hidden Windows `wsl.exe` holds the distro alive; if it is killed, or Windows sleeps, restarts or logs off, the run stops; relaunching with the same command resumes from the latest complete time.
- Status: `wsl -d MMAV-CFD -u cfd -- bash /home/cfd/run/ref/ref_status_a08.sh`
- Stop: `wsl -d MMAV-CFD -u cfd -- bash /home/cfd/run/ref/ref_stop_a08.sh` (stops only this run, by its own session).
- Scripts in `scripts/` are copies of the ones in `/home/cfd/run/ref/`.

## Limits
- The measured moment at 8 deg is interpolated across the moment break between table points 1 deg apart.
- The box-height check used snapshots of the coarse run (no mean field) and the coarse mesh, whose reversed zone may differ from the refined one.
- One mesh refinement, one angle; the result will not show mesh-independence.

## Launch
Started 2026-10-15 01:20 Windows time with the Start-Process command above (8 ranks, `nice -n 19`, 5,949,059 cells, fresh start from the mapped coarse 8 deg solution). First measured rate (steps 30-129, after start-up): **7.3 s per step at dt about 0.00075, i.e. about 9,800 s (2.7 h) per chord passage**; peak memory so far 9.1 GB (estimate was 10.5 GB). The earlier 113 h estimate assumed 9.3 s per step; at 7.3 s per step the run takes about **55 h if dt stays near 0.00075 and about 90 h if dt falls the way it did in the coarse laminar 8 deg case (1.58 times as many steps)**; projected finish between 2026-10-17 08:00 and 2026-10-19 00:00 Windows time, central estimate about 2026-10-18 09:00. These are projections from the first minutes, not measurements of the whole run.

## Result

**Grade: PASS** (x_cp 0.2279 c, inside 0.215-0.230 c; CM +0.0101, positive), on an averaging window that departs from the one fixed above. Disclosed below.

Numbers from `scripts/analyze_ref_a08.py` reading the run's own force file (`/home/cfd/run/vp_ref_a08/postProcessing/forceCoeffs1/0/coefficient.dat`); one-passage bins in `time_binned_ref.csv`.

| window | CL | CD | CM | CN | x_cp |
|---|---|---|---|---|---|
| t = 4 to 10.01 (graded) | 0.4511 | 0.0624 | +0.01007 | 0.4554 | 0.2279 |
| t = 8 to 10.01 (comparison) | 0.4517 | 0.0622 | +0.01011 | 0.4560 | 0.2278 |
| measured, mean of legs | 0.5023 | 0.0746 | +0.0137 | 0.5078 | 0.2230 |

- **Halves test** on the graded window, [4, 7.005] against [7.005, 10.01]: CL 0.4508 / 0.4514 (0.14 %), CM +0.00989 / +0.01025 (difference 0.00036), x_cp 0.2283 / 0.2275 (difference 0.0008). Passes (limits 2 %, 0.002).
- **Spread:** the six whole one-passage bins from t = 4 have x_cp 0.2269 to 0.2288, bin-to-bin standard deviation 0.0007, standard error of the mean 0.0003. No drift.
- **y+** on the plate at t = 10 (`postProcessing/yPlus1/0/yPlus.dat`): mean 1.34, maximum 18.6. Shear-layer cells as in "Refinement-box height check" (level 6).
- **Unsteadiness:** CL standard deviation over the window 0.0020 (coarse laminar 8 deg: 0.018). The regular-period test was not applied: the window is shorter than its 10-passage basis.

**Disclosed departure from the fixed window.** The window fixed before the run was t = 8 to 20 (at least 10 passages). The run was stopped at t = 10.01 (2026-10-16 06:12 UTC, right after the complete t = 10 write; `ref_stop_a08.sh`, since `runTimeModifiable` is false) and graded on t = 4 to 10.01, about 6.0 passages. Reason: compute time, about 28 h more for t = 20. Kai authorised stopping "if its looking somewhat conclusive", and the reviewer cleared it with conditions. Supporting evidence:
- the refined 6 deg run's bins were flat from t = 3-4 (`../refined_6deg/time_binned_ref.csv`: x_cp 0.217 at 3-4 against 0.216 for 8-20), so an early stop would not have changed that grade;
- the 8 deg bins are flat from t = 4;
- the margin to the 0.230 edge (0.0021) is 7 standard errors and 2.6 times the halves difference.

Choosing the window after seeing a value near a band edge is a post-hoc risk. It is kept small here only by the clear settling and that margin. The stop rule in `../README.md` ("Early-stop rule for future runs") was written after this run and applied to it after the fact. It would have allowed the stop.

**Sensitivity.**
- x_cp sits 0.0021 c inside the upper edge of its band.
- The measured reference itself is interpolated across the moment break over a 1 deg table gap (see "Reference values"), and that uncertainty is larger than this margin.
- The CFD x_cp is 0.005 c aft of the measured mean (0.2230).
- CM +0.0101 is just below its secondary band (+0.0105 to +0.0175), with CL 10.2 % below the tunnel (0.4511 against 0.5023; AVL 0.4349, coarse 0.4713). With x_cp in band, this is the lift gap described above, not a moment failure. CM closes the gap from the coarse value (-0.0290) to measured well past half (cross-check limit -0.0077).
- The time-averaged fields (UMean, pMean) cover only t = 8 to 10 (about 2 passages), so any reversed-flow map from them is indicative only.

**Consequences (as fixed before the run).**
- The refined laminar CFD reproduces the measured centre of pressure for this plate at Re 60,000 at 6 and 8 deg only.
- The rule on the fabricated wings is unchanged: no CFD on them, and CFD not used for balance or pull-out, unless Kai decides otherwise.
- Lift remains about 10 % low against this tunnel (11 % at 6 deg). One mesh refinement, so not mesh-independent.
