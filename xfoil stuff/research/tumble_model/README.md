# Tumble model (Li et al. 2022 quasi-steady falling-plate model)

Status: STEP A pass rule written BEFORE the validation sweep was run (this section is frozen; results are appended below).

## Sources (downloaded, treated as data, nothing executed)
- `sources/li2022.html` : Li, Goodwill, Wang, Ristroph 2022, JFM 937 A6, doi 10.1017/jfm.2022.89 (Cambridge core-reader HTML; equations read from the LaTeX in the page).
- `sources/andersen2005a.pdf` : Andersen, Pesavento, Wang 2005a, JFM 541, 91-104 (analysis of transitions between fluttering, tumbling and steady descent). NOT used to build the model: Li et al. state the full model in their section 4 and defer to Andersen 2005b (not downloaded) only for the form of the added-mass terms and C_R; both are given explicitly in Li et al., so nothing was needed from 2005a.

## Model: Li et al. 2022 section 4-5, implemented exactly (rigid_glider/tumble_sim.py cites each equation by number)
- State (x, y, theta, v_x', v_y', omega), body (co-rotating) frame velocities, 2D, per unit span: eqs 4.4-4.9.
- Added mass m11 = 0, m22 = pi rho l^2/4; Ia = pi rho l^4 [1 + 8 (2 l_CM/l)^2]/128 (text after 4.9).
- Lift L = L_T + L_R with the rotational (Magnus-like) term, eqs 4.10-4.12; drag eq 4.13; translational torque tau_T eq 4.14; rotational damping torque tau_R eq 4.15 (about the CoM, plus sign for 2 l_CM/l <= 1); buoyancy torque tau_B (after 4.9).
- C_L, C_D, l_CP: eqs 5.1-5.7 with C_L1 5.2, C_L2 0.95, alpha0 14 deg, delta 6 deg, C_D0 0.1, C_D1 5.0, C_D(pi/2) 1.9, C_CP0 0.3, C_CP1 3.5, C_CP2 0.2, C_R = 1.1 (all CITED).
- Angle of attack alpha = atan2(v_y' - omega l_CM, v_x') over the full circle; all force laws are defined for alpha in [-pi, pi] by the symmetry rules 5.3, 5.5, 5.7. No clamping, no wrapping problems.
- l_CM is the CoM offset from the CoV (mid-chord), positive toward the leading edge (+x'). theta is the angle of the plate's x' axis from the horizontal (gravity terms -m'g sin(theta), -m'g cos(theta)).

## Step A: paper-plane validation case (Li section 6.3, fig 12)
Inputs: chord l = 5.1 cm (CITED), span 15 cm (CITED), bare paper mass 0.59 g (CITED), rho_f = 1.2e-3 g/cm^3 (CITED), thickness h/l = 0.002 (CITED, section 7), g = 981 cm/s^2.
Not published (ASSUMED): the added weight is a line mass at the leading edge (copper tape), mass chosen to give the target l_CM/l; paper uniform; side fins ignored; I about the CoM = paper 1/12 m_p l^2 + m_p l_CM^2 + m_w (l/2 - l_CM)^2. Sensitivity variant (also reported, not part of the pass rule): the paper's generic formula I = m (l^2/12 + l_CM^2) with the total mass.
Sweep: l_CM/l = 0.00 to 0.35 in steps of 0.01. Initial conditions per value: 8 attitudes (0, 45, ..., 315 deg) x 3 pitch rates (-10, 0, +10 rad/s), released at rest in translation (24 starts). T = 15 s, dt = 2 ms RK4 (JUDGEMENT; convergence check against dt = 1 ms reported).

### Classifier (applied to the last 5 s of each run; thresholds fixed here, not tuned)
- TUMBLE: |theta(end) - theta(start)| over the window >= 2 pi (at least one net rotation in the window).
- Otherwise STEADY if theta range < 10 deg and alpha range < 5 deg and speed range < 10 % of mean speed over the window.
  - STEADY and mean |alpha| < 2 deg: DIVE.
  - STEADY and mean |alpha| >= 2 deg: GLIDE.
- Otherwise BOUND/FLUTTER (non-steady, non-rotating oscillation: flutter, progressive flutter, bounding, still-settling).
- The cell mode for a given l_CM/l is the plurality mode of its 24 starts; a cell is "clear" if that mode has >= 80 % of the starts, otherwise "mixed".

### PASS RULE (all must hold)
1. l_CM/l = 0.00 and 0.08 are clear TUMBLE cells.
2. l_CM/l = 0.24 is a clear GLIDE cell.
3. l_CM/l = 0.31 is a clear DIVE cell.
4. Along the sweep, the plurality mode rank (TUMBLE 0 < BOUND/FLUTTER 1 < GLIDE 2 < DIVE 3) never decreases as l_CM/l rises (mixed cells are skipped for this test but must be reported). So the order is tumble, (bound), glide, dive.
5. The contiguous glide band contains 0.24.
If any rule fails: report FAIL and stop. No constants are tuned.

---
# RESULTS (written after the sweeps; everything above this line was frozen before they ran)

## Step A result: IMPLEMENTATION CHECK PASSED for the variant declared in advance (geometry-based inertia); the generic-inertia variant FAILS the same rule
This is an implementation/consistency check, NOT a validation of the model against experiment. The dive edge near l_CM/l = 0.30 follows directly from the cited centre-of-pressure coefficient (C_CP^0 = 0.3: the static centre of pressure at alpha = 0 is at 0.3 l), so reproducing it is a consistency check, not independent evidence.
Command: `py -B rigid_glider/tumble_sim.py stepA` (about 12 s). Files: `results/runs/tumble_sim/stepA_cells_geometry.csv`, `stepA_runs_geometry.csv`, `stepA_verdict.json`.
- Rule 1: 0.00 = TUMBLE 24/24; 0.08 = TUMBLE 21/24 (88 %, 3 BOUND/FLUTTER). ok
- Rule 2: 0.24 = GLIDE 24/24. ok
- Rule 3: 0.31 = DIVE 24/24. ok
- Rule 4: no decrease in plurality rank along the sweep (mixed cells 0.06, 0.07, 0.09-0.11, 0.16 skipped). ok
- Rule 5: clear glide cells 0.17 to 0.29 (contains 0.24). ok
- Plurality map (T tumble, B bound/flutter, G glide, D dive, ? = mixed cell): 0.00-0.05 T; 0.06 T?; 0.07 B?; 0.08 T; 0.09-0.10 T?; 0.11 B?; 0.12-0.15 B; 0.16 G?; 0.17-0.29 G; 0.30-0.35 D.
- Time-step check: 144 starts at dt = 1 ms against dt = 2 ms, identical modes in all 144.
- Compared with the paper's own numbers: paper-plane figure 12 gives tumble at 0 and 0.08, flip/tumble/dive mixes around 0.14, glide at 0.24, dive at 0.31. The model transitions here (tumble to bound near 0.1, bound to glide near 0.16, glide to dive at 0.30) are the model's, not tuned; the 0.30 dive boundary follows from l_CP(0) = 0.3 l (CITED C_CP^0). The paper's experiments put glide at about 0.22-0.29 (Li et al., section 7); that the dive edge here (0.30) agrees follows from the cited coefficient and is not independent confirmation.

### Step A caveats (stated, not hidden)
- The sensitivity variant with the paper's generic inertia formula I = m (l^2/12 + l_CM^2) does NOT pass the frozen rule (`stepA_cells_generic.csv`): 0.08 is 75 % tumble (needs 80 %), 0.24 is 79 % glide (5 of 24 starts still settling), and cells 0.27-0.28 are bound/flutter among glides, so rule 4 and 5 also fail. A diagnostic 40 s run of the same starts (not part of the rule) shows the five unsettled 0.24 starts are a long glide transient (24/24 GLIDE at 40 s), which matches the paper's remark that gliding has "a long transient before reaching steady state". The pass above is for the geometry-based variant, which was declared the primary one in advance; it is NOT insensitive to how the inertia of the added weight is modelled.
- At 0.08 about 1/8 of starts do not tumble (bound/flutter); at 40 s about 3/8 do not. The paper itself reports mixed flips/tumbles/dives near this value. In this sweep the end state is start-independent only well inside a band (0, 0.24, 0.31); near the boundaries (about 0.06 to 0.16) it depends on the start. The paper's statement that the modes are robust to initial conditions is NOT reproduced as a general property here, and for the gliders below it is not true near the mark or at +3 mm.
- Not checked against the paper: trajectories in figure 12 (images, not extracted), the tank experiments of figure 3 (only the paper-plane case was in scope).
- Items in the paper I could NOT extract: all figures (3, 8-12), the exact mass and inertia the authors used for the weighted paper plane (I used a leading-edge line mass and ignored the side fins, ASSUMED), their integrator tolerances and simulation length, and their initial conditions for figure 12.

## Step B (run only because Step A passed): gliders as 2-D strips
Command: `py -B rigid_glider/tumble_sim.py stepB` (42 120 runs, about 8 min). Files: `results/runs/tumble_sim/index.html` (dark-mode mode map), `stepB_map.csv` (shares per CG, all variants), `stepB_runs_primary.csv` (every run of the primary variant), `stepB_cells.json`.
Starts per CG: 36 attitudes (every 10 deg, all 360 deg) x 9 pitch rates (-30, -20, -10, -5, 0, 5, 10, 20, 30 rad/s) = 324, released with zero translational velocity (ASSUMED). 15 s runs, dt 1 ms RK4; terminal mode = Step A classifier on the last 5 s. A second classification uses the 1.0 to 2.5 s window (roughly a drop from under 10 m); in that window nearly every start is still "BOUND/FLUTTER" (not yet steady), so it only says how many are still rotating there.

Inputs (per unit span = value / span):
- Swift A foam 400 (CALCULATED from `bird_outline_v4_swift.build_geometry4(GLIDE)` and `bird_outline_v2.size_numbers`, imported not modified): all-up 9.76 g, ballast 1.20 g at the -12.5 mm nose hole, CG 37.0 mm aft of the root LE (reproduces the brief's 9.8 g / 1.2 g / 37.0 mm), MAC 47.41 mm with its LE at 27.45 mm so the MAC mid-chord is 51.16 mm, wing area / span = 43.87 mm (primary strip chord), sheet area / span = 55.43 mm. Pitch inertia about the CG from the sheet polygon (uniform sheet), the rail (uniform bar) and the ballast (point).
- F45_span250 (CALCULATED from `results/runs/rigid_glider/flat_test_v2/F45_span250/geometry.json`, 0.6 mm printed plate, read only): plate 10.33 g, rib 0.89 g, two fins 0.83 g, stick 2.53 g, ballast at the -22.5 mm nose hole sized by moment balance; mean chord 55.56 mm, MAC 57.61 mm with LE at 30.02 mm (mid-chord 58.83 mm). The kit has no single design mark: the reference CG is 38.0 mm (10 % MAC ahead of the AVL neutral point with fins, 43.9 mm; JUDGEMENT), map covers 30 to 44 mm.
- l_CM = (reference mid-chord) - CG; l_CM/l = l_CM / strip chord. Primary = mean chord and MAC mid-chord, as briefed. Variants (chord = MAC; chord = sheet area/span; reference = sheet centroid) are in `stepB_map.csv`.
- Air density 1.2e-3 g/cm^3, g = 981 cm/s^2, buoyancy neglected.
- The Li coefficient laws were fitted to a rectangular AR 8 plate in water at Re of order 1e3 to 1e4 and are applied here unchanged: ASSUMED carry-over to a swept AR 9 tailed wing at Re of order 1e4 in air. The tail bend / reflex is not in the Li model, no published form exists to add it, and it was NOT added (no bend result is claimed). The Swift tail area (about 18 % of the wing) is not represented as a separate surface in the strip.

### Mode shares at the design mark and +-3, +-6 mm (primary variant, terminal mode, 324 starts each; offset + = CG aft)
| glider | offset mm | CG mm | mass g | l_CM/l | tumble % | bound/flutter % | glide % | dive % |
|---|---|---|---|---|---|---|---|---|
| Swift A foam 400 | -6 | 31.0 | 11.12 | 0.459 | 0 | 0 | 0 | 100 |
| Swift A foam 400 | -3 | 34.0 | 10.40 | 0.391 | 0 | 0 | 0 | 100 |
| Swift A foam 400 | 0 (mark) | 37.0 | 9.76 | 0.323 | 3 | 0 | 0 | 97 |
| Swift A foam 400 | +3 | 40.0 | 9.22 | 0.254 | 50 | 16 | 35 | 0 |
| Swift A foam 400 | +6 | 43.0 | 8.70 | 0.186 | 58 | 0 | 42 | 0 |
| F45_span250 | -6 | 32.0 | 20.29 | 0.483 | 0 | 0 | 0 | 100 |
| F45_span250 | -3 | 35.0 | 19.23 | 0.429 | 0 | 0 | 0 | 100 |
| F45_span250 | 0 (ref) | 38.0 | 18.28 | 0.375 | 0 | 0 | 0 | 100 |
| F45_span250 | +3 | 41.0 | 17.41 | 0.321 | 0 | 0 | 0 | 100 |
| F45_span250 | +6 | 44.0 | 16.63 | 0.267 | 0 | 1 | 99 | 0 |

Whole Swift map (primary): dive for every CG forward of 36 mm (l_CM/l above 0.34), a transition at 37 to 38 mm (tumble 3 % at 37, 28 % at 38 with 63 % bound/flutter), and tumble 39 to 62 %, glide 31 to 46 % for 39 to 45 mm. The F45 kit range 30 to 42 mm is all dive in the model; glide appears from 43 mm. Swift aft offsets of +8 and +9 mm need aft ballast at a rail hole and the mass jumps to 12 and 21 g (the project's size_numbers rule), so those two columns are not comparable.
Note the glide shares are mixtures with tumble: at these l_CM/l the model has both attractors, so the outcome IS start-dependent near the mark (3 % tumble / 97 % dive) and at +3 mm (50 / 16 / 35), and it changes with the strip definition (see the bonus check). Which starts go where was not analysed. The tumble-glide-dive transition sits at the same l_CM/l (about 0.3) that Step A found, so what moves a glider between modes is entirely the l_CM/l this task assigns it.

### Bonus check: does it predict tumbling at the design-mark CG with no bend (observed 2026-10-17)?
No. The model gives 97 % dive and 3 % tumble for the Swift at the mark (primary); the F45 at its reference CG gives 100 % dive. It is also very sensitive to the strip definition at the mark: chord = MAC (l_CM/l 0.299) gives 77 % bound/flutter, 21 % dive, 2 % tumble; chord = sheet area/span (0.255) gives 68 % glide, 32 % bound/flutter; reference = sheet centroid (0.341) gives 100 % dive. So the Swift at the mark sits right on the model's glide/dive edge (about 0.30 in l_CM/l) and neither side is tumble. Tumbling in the model needs the CG about 3 mm or more aft (l_CM/l below about 0.28). No constants were changed to get this answer; it is not a validation of the glider results. Possible reasons the model and the drops disagree (not tested, speculation): the 2-D strip ignores 3-D flow, sweep, aspect ratio 9 and the fan tail; the wonky release gave velocities the sim does not start with; the real CG or reference point may differ from the 37.0 mm / MAC mid-chord numbers used.

---
# DROP-WINDOW SCORING (definitions written BEFORE the run; reviewer directive)
Why: the terminal mode (10 to 15 s) is the wrong window for the observed drops, which fell under 10 m in about 1.5 to 2 s. Script: `rigid_glider/tumble_drop_window.py` (new file; imports tumble_sim.py, nothing else changed). Same 324 starts per case (36 attitudes x 9 rates, at rest), same strips, dt 1 ms.
- Fall depth = the model's own vertical position y (state, cm), not free fall. t_H = first time y <= -H, H = 5 m and 10 m. If a start has not reached H within 15 s it is counted as "not reached" and excluded from that H's shares (the count is reported).
- ">= 1 rotation": the largest |theta(t) - theta0| over 0 <= t <= t_H reaches 2 pi (checked at every integration step).
- "Still rotating" at t_H: |omega(t_H)| >= 5 rad/s (threshold JUDGEMENT, set here) AND sign(omega(t_H)) = sign(theta(t_H) - theta0) (sign-consistent with the net rotation so far) AND |theta(t_H) - theta0| >= pi (at least half a turn made).
- Cases: Swift A foam 400 at the mark (CG 37.0 mm) with all four strip definitions (primary area/span chord 43.87 mm; chord = MAC; chord = sheet area/span; reference = sheet centroid), and at +3 mm (CG 40.0 mm) primary only. No CG, kit or build-file change.
- Breakdown by initial pitch rate (9 values); attitude: share by 90-degree attitude quadrant and the min/max over the 36 attitudes.

## Drop-window results (run after the definitions above; Swift A foam 400, 324 starts per case, all reached 5 m and 10 m within 15 s)
`py -B rigid_glider/tumble_drop_window.py` (about 1 min). Files: `results/runs/tumble_sim/dropwindow_by_rate.csv`, `dropwindow_summary.json`. "Rot" = share of starts with at least one full rotation (|dtheta| >= 360 deg) before falling H; "still" = share still rotating (rule above) at depth H. Depth is the model's own y.

| case (l_CM/l) | H | Rot % | still % | mean t_H s |
|---|---|---|---|---|
| mark, primary (0.323) | 5 m | 46 | 17 | 1.59 |
| mark, primary | 10 m | 47 | 2 | 2.36 |
| mark, chord = MAC (0.299) | 5 m | 44 | 15 | 1.63 |
| mark, chord = MAC | 10 m | 45 | 2 | 2.52 |
| mark, chord = sheet area/span (0.255) | 5 m | 43 | 4 | 2.52 |
| mark, chord = sheet area/span | 10 m | 43 | 0 | 6.52 |
| mark, reference = sheet centroid (0.341) | 5 m | 44 | 4 | 1.54 |
| mark, reference = sheet centroid | 10 m | 45 | 0 | 2.23 |
| +3 mm, primary (0.254) | 5 m | 53 | 50 | 2.12 |
| +3 mm, primary | 10 m | 53 | 50 | 4.86 |

By initial pitch rate, Rot % / still %, at 5 m then 10 m (the +w and -w values agree to within 1 point; the +w value is shown; full table in dropwindow_by_rate.csv):

| w0 rad/s | mark primary 5 m | 10 m | mark MAC 5 m | 10 m | mark sheet-chord 5 m | 10 m | mark centroid 5 m | 10 m | +3 mm 5 m | 10 m |
|---|---|---|---|---|---|---|---|---|---|---|
| 30 | 100 / 67 | 100 / 6 | 100 / 61 | 100 / 8 | 100 / 14 | 100 / 0 | 100 / 14 | 100 / 0 | 100 / 100 | 100 / 100 |
| 20 | 86 / 6 | 89 / 3 | 81 / 8 | 81 / 0 | 72 / 3 | 72 / 0 | 81 / 3 | 81 / 0 | 100 / 100 | 100 / 100 |
| 10 | 17 / 3 | 17 / 0 | 17 / 0 | 17 / 0 | 14 / 0 | 14 / 0 | 17 / 3 | 17 / 0 | 28 / 14 | 28 / 14 |
| 5 | 3 / 0 | 3 / 0 | 3 / 0 | 3 / 0 | 6 / 3 | 6 / 0 | 3 / 0 | 3 / 0 | 11 / 11 | 11 / 8 |
| 0 | 0 / 0 | 3 / 0 | 0 / 0 | 3 / 0 | 0 / 0 | 3 / 3 | 0 / 0 | 3 / 3 | 0 / 0 | 3 / 3 |

Reading it:
- Most of the "at least one rotation" share is the prescribed initial rate itself: at 30 rad/s one turn takes about 0.2 s, so every start at 30 rad/s turns once before 5 m under every definition. The informative starts are the low-rate ones: with w0 = 0 to 5 rad/s only 0 to 6 % turn before 5 m (11 % at 5 rad/s for +3 mm) and 3 to 6 % before 10 m.
- At the mark, spun-up starts mostly do not keep rotating: by 10 m only 0 to 2 % of all starts are still rotating (primary and MAC strips: 6 to 8 % of the 30 rad/s starts). At +3 mm (l_CM/l 0.254) every start with |w0| >= 20 rad/s, and 14 % of those at 10 rad/s, is still rotating at both depths: the model does keep tumbling there if released with enough spin.
- Does attitude matter? Yes at intermediate rates, no at the extremes. At |w0| = 0 and 5 rad/s almost no attitude completes a turn; at 30 rad/s all do; at 10 rad/s only 6 of the 36 attitudes at the mark (10 at +3 mm), a narrow band that depends on the spin sign, rotate once before 5 m, and at 20 rad/s the primary mark has 31 of 36 attitudes rotating (the ones near 70 to 120 deg do not). Quadrant shares of the full-rotation count are 36 to 49 %, because each attitude gets the same rate mix.
- The release spin needed in this model for end-over-end rotation to persist to 5 m at the mark is of order 30 rad/s; whether the real quad releases carried that much was not measured. That number depends on the strip chord and reference point as much as on the glider: with chord = sheet area/span the 10 m fall takes 6.5 s on average.

## Plain statement on use
THIS MODEL IS NOT USABLE FOR SETTING A CG. It is an uncalibrated 2-D plate model applied to a 3-D swept tailed wing by a strip. Its glide/dive edge is set by one cited coefficient (C_CP^0 = 0.3), the Swift mark sits on that edge, the verdict flips with the chord and reference-point definition, no tail bend is represented, and it does not reproduce the observed tumbling at the mark in the drop window or at the terminal window unless the release carries a large spin. Use it, at most, to see how a plate with a given l_CM/l can behave under large spin.
