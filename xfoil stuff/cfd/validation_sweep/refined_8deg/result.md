# Refined laminar 8 deg run: result against the criteria fixed in advance

Verdict: **PASS**, on an averaging window that departs from the one fixed in advance (disclosed below). Source of every number: `scripts/analyze_ref_a08.py` reading the run's own force file (`/home/cfd/run/vp_ref_a08/postProcessing/forceCoeffs1/0/coefficient.dat`); one-passage bins in `time_binned_ref.csv`; criteria and measured references in `README.md`.

Case `/home/cfd/run/vp_ref_a08` (5,949,059 cells, laminar, 8 ranks at nice 19) was **stopped early by decision at t = 10.01 chord passages** (2026-10-16 06:12 UTC, right after the complete t = 10 write, with `ref_stop_a08.sh`; `runTimeModifiable` is false, so `stopAt writeNow` was not available). The end time fixed in advance was t = 20.

## Averaging window and validity
Window used for the grade: **t = 4 to 10.01 (about 6.0 passages)**. The window fixed in advance was t = 8 to 20 (at least 10 passages). Halves [4, 7.005] and [7.005, 10.01]: CL 0.4508 and 0.4514 (0.14 % apart, limit 2 %), CM +0.00989 and +0.01025 (0.00036 apart, limit 0.002), x_cp 0.2283 and 0.2275 (0.0008 apart, limit 0.003): the halves test passes. The six whole one-passage bins from t = 4 have x_cp 0.2269 to 0.2288 (bin-to-bin standard deviation 0.0007, standard error of the mean 0.0003); no drift.
y+ on the plate at t = 10: mean 1.34, maximum 18.6. CL standard deviation over the window 0.0020 (coarse laminar 8 deg: 0.018). The regular-period test was not applied: the window is shorter than its 10-passage basis.

## Averaged coefficients
| | CL | CD | CM (c/4, nose-up +) | CN | x_cp = 0.25 - CM/CN |
|---|---|---|---|---|---|
| refined laminar, t = 4 to 10.01 (graded) | 0.4511 | 0.0624 | +0.01007 | 0.4554 | 0.2279 |
| refined laminar, t = 8 to 10.01 (comparison) | 0.4517 | 0.0622 | +0.01011 | 0.4560 | 0.2278 |
| measured at 8.0 deg, mean of legs | 0.5023 | 0.0746 | +0.0137 | 0.5078 | 0.2230 |
| coarse laminar (sweep, t = 8-20) | 0.4713 | 0.0786 | -0.0290 | | 0.3106 |
| AVL (inviscid, linear) | 0.4349 | | | | |

## Criteria, applied as written in `README.md`
| Criterion | Value | Result |
|---|---|---|
| x_cp between 0.215 c and 0.230 c (graded) | 0.2279 | met |
| CM positive (graded) | +0.01007 | met |
| CM against its secondary band +0.0105 to +0.0175 (reported, not graded) | +0.0101 | just below the band |
| CL against AVL, tunnel, coarse CFD (reported, not graded) | 0.4511 against 0.4349, 0.5023, 0.4713 | 10.2 % below the tunnel |
PASS = x_cp in 0.215-0.230 c and CM positive: **PASS**. The secondary CM cross-check (CM above -0.0077, more than half the gap from the coarse -0.0290 to the measured +0.0137) is also met.

## Disclosed departure from the fixed window
The window fixed before the run was t = 8 to 20, at least 10 passages. The run was stopped at t = 10.01 and graded on t = 4 to 10.01. Reason: compute time (about 28 h more for t = 20); Kai authorised stopping "if its looking somewhat conclusive", and the reviewer cleared it with conditions. Supporting evidence: the refined 6 deg run's bins were flat from t = 3-4 (`../refined_6deg/time_binned_ref.csv`), so an early stop would not have changed that grade; the 8 deg bins are flat from t = 4; the margin to the 0.230 edge (0.0021) is 7 standard errors and 2.6 times the halves difference. Choosing the window after seeing a value near a band edge is a post-hoc risk, kept small here only by the clear settling and that margin. The early-stop rule in `../README.md` was written after this run and applied to it after the fact; it would have allowed the stop.

## Sensitivity
- x_cp sits 0.0021 c inside the upper edge of its band.
- The measured reference is itself interpolated across the moment break over a 1 deg table gap, which is a larger uncertainty than the margin.
- The CFD x_cp is 0.005 c aft of the measured mean (0.2230).
- CM +0.0101 is just below its secondary band, with CL 10.2 % below the tunnel (0.4511 against 0.5023; AVL 0.4349, coarse 0.4713). With x_cp in band this is the lift-gap pattern described in `README.md`, not a moment failure.
- The time-averaged fields cover only t = 8 to 10 (about 2 passages), so any reversed-flow map from them is indicative only.

## Consequences (as fixed before the run)
- The refined laminar CFD reproduces the measured centre of pressure for this plate at Re 60,000 at 6 and 8 deg only.
- The rule on the fabricated wings is unchanged: no CFD on them, and CFD is not used for balance or pull-out, unless Kai decides otherwise.
- Lift remains about 10 % low against this tunnel (11 % at 6 deg). One mesh refinement, so not mesh-independent.
