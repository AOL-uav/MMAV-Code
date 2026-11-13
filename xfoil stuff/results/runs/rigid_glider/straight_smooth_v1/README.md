# Straight and smooth, phase 1 (lateral / turn side only): model study, nothing here is a flight result

## Question, method and pre-stated rule

Written 2026-10-21 15:20 (local), BEFORE any grid configuration was run. This section is not edited after the run; everything below the line "RESULTS" is filled in afterwards.

### Question

Kai's two 400 mm foam gliders (B45 flat wing with centre flaps; Swift A foam 400 with the tail bent 6 deg, CG 73.0 mm from the nose tip) glide and are stable but settle into a steady turn. Kai suspects build asymmetry (for example the tail bent slightly more on one side). Can a change of SHAPE (centre-joint dihedral, bigger fins, a centre fin) make the glider go straight, and what is left that only an adjustment (a pre-set trim tab) can remove?

### Hypothesis to test (not assumed)

With a fixed built-in asymmetry (extra roll-moment coefficient dCl, extra yaw-moment coefficient dCn, both about the CG, stability axes, positive = right wing down / nose right) and no control, the steady state has sideslip beta, yaw rate r and bank phi constant, roll rate p = 0. With r_hat = r b / (2 V) the steady moment balance (AVL stability derivatives are per rad, AVL non-dimensional rate convention p b/2V, r b/2V) is

    Clb*beta + Clr*r_hat + dCl = 0
    Cnb*beta + Cnr*r_hat + dCn = 0

Cramer's rule gives (derived here, signs checked)

    r_hat = (Cnb*dCl - Clb*dCn) / (Clb*Cnr - Clr*Cnb)

which equals Kai's expression (Clb*dCn - Cnb*dCl)/(Clr*Cnb - Clb*Cnr) with numerator and denominator both multiplied by -1. The denominator D = Clb*Cnr - Clr*Cnb is the spiral-stability criterion (spiral stable when D > 0 for the usual signs Clb < 0, Cnr < 0, Cnb > 0). So the claim to test is:
1. no shape sets r_hat to zero (the numerator is built from the same derivatives; shape only changes the size of r_hat through D and the derivatives), a shape change can only make the circle larger (more dihedral effect, more spiral stability);
2. the turn radius R = b / (2 r_hat) (horizontal, small glide angle) scales with span for the same ANGULAR asymmetry, because r_hat is scale-free (heading change over a path length s is 2 r_hat s / b radians, independent of speed);
3. a pre-set adjustable fin trim tab can null the turn.
Sign check, stated before the run: dCl > 0 (right wing down) with D > 0 and Cnb > 0 gives r_hat > 0 (turn to the right, steady bank phi = (V/g) r > 0 right wing down), and the mirror image (left wing down gives a left turn). This is tested in code, and again by integrating the AVL linear system in time.

### Models (MODEL = inviscid AVL 3.52, vortex lattice, flat plate, thickness ignored)

- B45 baseline: exactly the deck of `rigid_glider/b45_centre_flaps.py` (flat wing 400 mm, AR 4.5, taper 0.5, 25 deg quarter-chord sweep, tip fins 24 mm tall x tip chord, centre flaps 10 mm chord at y = 30..80 mm, nominal location, 10 deg trailing edge UP, no dihedral). CG 66.0 mm aft of the root leading edge (= nose tip). The task text said 65.5 mm "unless the code says otherwise": the code that marks the flown glider (`flight_day_cg_page.py`, `cg_t = 66.0`, inside the 63-68 mm agree band of the b45_centre_flaps README) says 66.0, so 66.0 is used. Mass: printed stick, foam plate, tip fins, nose ballast at the -37.5 mm hole sized for this CG (ASSUMED masses from `geometry.json` B45, 300 g/m2 foamboard).
- Swift A foam 400 baseline: Swift A GLIDE outline (`bird_outline_v4_swift.GLIDE`), AVL deck `bird_outline_v2.avl_deck2` with 10 x 80 panels (the deck the oscillation study and fin kit use), tail bent 6 deg UP, CG 73.0 mm from the nose tip = 37.0 mm aft of the root leading edge (nose tip is 36.0 mm ahead of the root LE at 400 mm).
- As-flown fins of Swift A foam 400 (the instruction asked this be found out and reported): none. The A_foam_span400 build files (`bird_outline_v4_swift/A_foam_span400/drawing.svg`, `parts.dxf`, `template_page1/2.svg`) contain no fin, `flight_day_cg_page.py` / `cg_marks.html` mark only CG, NP, nose hole and the 6 deg tail bend, and the oscillation study deck has Wing + Tail only. `fin_kit_v4` is a written FALLBACK kit (tip fins, centre fin, tail fin), not something that was built or flown. Therefore "as flown" Swift A = no fin; this is read from the files, Kai to confirm. Consequence: "fin area x1.5" is not defined for Swift A; its fin options are none and the `fin_kit_v4` centre fin ("centre_best", 2.0 % of S, foam 400 size from `chosen_sizes.csv`: chord 16.76 mm, height 20.95 mm, trailing edge 1 mm ahead of the tail hinge line, vertical on the centreline).
- B45 as flown has the two tip fins (24 mm tall, 59.3 mm chord, centred on the plate plane). Fin options: as flown, and fin area x1.5 (height x1.5 = 36 mm, same chord).
- Dihedral: both half wings rotated rigidly about the root (x-axis through the centreline): y -> y cos(G), z -> y sin(G) for G = 0, 3, 6, 9, 12 deg; reference span, area and chord stay those of the flat wing (so coefficients are comparable). Tip fins stay vertical, centred on the (raised) tip. Swift A tail fan stays on the centreline, flat, bent 6 deg (hinge at the same x).
- CG for variants: static margin in % MAC is held at the baseline's value (baseline SM = (Xnp - CG)/MAC at its own CG) by moving the CG to Xnp_variant - SM_base x MAC; the CG is reported; the ballast mass sits at the baseline's nose hole and its mass is solved to give that CG (ASSUMED).
- Trim: alpha such that Cm about the CG = 0 (linear fit of Cm(alpha) over alpha = 0, 4, 8 deg); trim CL, V = sqrt(2 m g / (rho S CL)).
- Lateral eigenvalues: the 4 x 4 lateral block (v, p, r, phi) of the system matrix AVL writes (state order u w q theta v p r phi ...), at theta0 = 0 in AVL (the AVL mode calculation; thrust balances drag is its premise), roots with the solver in `fin_kit_v4`. Mass and inertia (Ixx, Iyy, Izz about the CG, Ixz set to 0) are ASSUMED from a point-sampled plate model. Eigenvalues are dimensional (they depend on span and speed); the rule uses the 400 mm span. Dutch roll damping depends on the assumed inertia.

### Reference asymmetries (ASSUMED sizes, 1 deg each; sign conventions stated here)

AVL axes: x aft, y to starboard (the pilot's right looking forward), z up. Roll moment Cl' and yaw moment Cn' are the AVL stability-axis totals; positive Cl' = right wing down, positive Cn' = nose right.
(a) Differential deflection of the pitch-trim surface by 1 deg. B45: right flap 10.5 deg UP, left flap 9.5 deg UP. Swift A: right tail half bent 6.5 deg UP, left tail half 5.5 deg UP (the two tail halves modelled explicitly, no YDUPLICATE). Expected sign: more up on the right = less lift on the right = Cl' > 0.
(b) Wing-half incidence difference 1 deg: right half +0.5 deg (leading edge up), left half -0.5 deg, both halves modelled explicitly (no YDUPLICATE), rotation about the y axis at each section. Expected sign: Cl' < 0 (right half lifts more, right wing goes up).
(c) 1 deg toe of one fin (where fins exist): B45 the right tip fin only; Swift A the centre fin. Positive toe = trailing edge toward +y (starboard), which is aerodynamically the same as a positive sideslip angle of 1 deg on that fin (flow from the right); the AVL section-incidence sign needed for it is calibrated in the tests on an isolated fin against a 1 deg sideslip run, not assumed. Expected sign: Cy < 0, Cn' > 0 for the positive toe.
dCl and dCn of each asymmetry = (AVL totals with the asymmetry) minus (same deck, same alpha, no asymmetry). Mirror-image asymmetries give the mirror-image turn, so only magnitudes enter the rule. The three asymmetries are evaluated one at a time, not summed.

### Pre-stated rule (not to be changed after the run)

A configuration is STRAIGHT-CAPABLE if at its trim (CG as flown for the baselines; for variants the same static margin in % MAC as its baseline, CG reported) ALL of the following hold at 400 mm span:
1. Spiral mode: real eigenvalue < 0 (stable) or time to double > 15 s (eigenvalue < ln 2 / 15 = 0.0462 1/s).
2. Dutch roll damping ratio >= 0.10 AND the roll subsidence eigenvalue < 0.
3. For EACH reference asymmetry that exists for that configuration ((a), (b), and (c) where a fin exists), the heading change over 100 m of glide path is < 90 deg, i.e. 2 |r_hat| (100 m) / (0.4 m) < pi/2 rad, i.e. |r_hat| < 0.00314, i.e. radius R = b / (2 |r_hat|) >= 63.7 m (rule text says 64 m). r_hat from the closed form above using AVL Clb, Cnb, Clr, Cnr and the asymmetry-run dCl, dCn. The 250 mm values (same angles: same r_hat, R and heading change per 100 m scaled by 250/400 and 400/250) are REPORTED, not part of the pass/fail.
4. Static margin within 1 % MAC of the baseline's (met by construction, reported) AND trim CL within 0.02 of the baseline's.
The baseline is itself reported against the rule. Baseline consistency check (report only, no pass/fail): the baselines should come out spiral-neutral or weak and should turn noticeably for 1 deg of asymmetry, consistent with Kai's big circles.

### Cross-check (pre-stated)

For at least 3 configurations the linear lateral system (AVL system matrix A, plus a constant asymmetric input) is integrated in time (RK4, long enough to settle or to show the divergence) and the steady yaw rate is compared with the closed form; they must agree within 5 % where the system is stable. The constant input is taken from AVL's own control columns of the system matrix where a control exists (flap R / flap L, tail halves), so the closed-form and time-integration paths use different data plumbing. If the spiral is unstable (eigenvalue > 0) there is no steady state and the cross-check is only done for stable configurations.

### Grid (bounded, at most about 40 configurations)

B45: dihedral 0, 3, 6, 9, 12 deg x fins {as flown, area x1.5} = 10 configurations. Swift A: dihedral 0, 3, 6, 9, 12 deg x {no fin (as flown), centre fin from `chosen_sizes.csv`} = 10 configurations. 20 configurations in total. If an extension is needed the run stops and reports instead.
Trim tab: for the best two candidates (those with a fin; ranked by the largest heading change among their reference asymmetries, smallest first), a tab on one fin with its hinge at 70 % of the fin chord (ASSUMED; B45: the right tip fin; Swift A: the centre fin), deflection that makes r_hat = 0 for each reference asymmetry separately, using the tab's own dCl, dCn per degree from AVL (sign: positive = trailing edge toward +y), reported with the residual sideslip and bank.

### Limits (stated now)

AVL is inviscid; flat-plate thickness ignored (5 mm foam on a 59-118 mm chord is not thin); the asymmetry sizes are assumed; low-Reynolds-number fin and flap effectiveness will be lower than AVL's (Re of a few thousand to 30,000), so the dCl, dCn from fins and tabs are upper estimates; masses and inertias ASSUMED; Ixz set to 0 in the AVL mass file; AVL's mode calculation uses level flight (theta0 = 0) while the real glide is about 6.6 deg down, which changes the steady-turn balance (a glide-angle correction is reported as a sensitivity only); no flight evidence about turn direction or size exists, so nothing here can confirm Kai's explanation of the turn.

RESULTS
Filled in 2026-10-21 20:00 (local) by `rigid_glider/straight_smooth_v1.py`, REVISED pass after the reviewer's REVISE (8 directives); wall time of the full run 280 s. MODEL = inviscid AVL 3.52; ASSUMED = masses, inertias, asymmetry sizes, tab hinge, B45 glide angle and pull-out loss; KAI = Kai's statement.

### What changed vs the first pass

1. AVL modes, system matrices and RK4 now use the base-state elevation theta0 = alpha_trim + gamma (MODE parameter E; the first pass left it at 0, i.e. an implied glide angle of -alpha, 4-9 deg). gamma = -6.61 deg (MODEL for Swift A, ASSUMED for B45), sensitivity gamma = -5 and -8 deg.
2. Closed form for the steady turn now uses the roll rate of a turn about the VERTICAL axis during a glide, p' = r' tan(-gamma). The pre-stated p' = 0 form is physically wrong for a glide (it is the stability-axis roll rate of a turn about the velocity vector, which a descending turn is not). The first pass's 'body' form (tan alpha) was only right for theta0 = 0 and is superseded. RK4 vs closed form on the SAME AVL matrix checks algebra (the closed form is the steady state of that linear system), not physics (that the base state and the matrix are right).
3. The product of inertia Jxz = integral(x z dm) from the point-mass model (dihedral tips are up and aft) is now in the mode calculation; Delta zeta per configuration is reported, and the oscillatory mode is labelled by its shape (|beta|/|phi|, sign of Cnb).
4. Kai-facing text leads with the degrees of each asymmetry needed for R = 12-32 m; per-1-deg radii beyond bank 30 deg are not quoted (kept in the CSV, flagged out of linear range).
5. Roll trims (differential centre flap for B45; differential tail halves for Swift A) are computed next to the fin tab. Options, none chosen.
6. The trim-CL rise with dihedral at fixed static margin is diagnosed (below) before any re-trim statement.
7. Tests re-run (summary at the end, including the hash check of the pre-stated section above, which is unchanged).
8. Dihedral is framed honestly (summary below).

### Summary for Kai (read this first)

**Degrees of each asymmetry needed to give a circle of R = 12.4 to 31.7 m** (Kai's rough 1-2 turns per drop from 25-30 m; R from release height minus the 6.93 m pull-out loss, glide angle 6.61 deg, MODEL for Swift, ASSUMED for B45). Corrected steady-turn model (gamma = -6.61 deg, Ixz included), linear scaling from 1 deg:

| glider as flown | (a) differential trim surface (B45 flaps / Swift tail halves) | (b) wing-half incidence difference | (c) one-fin toe |
|---|---|---|---|
| B45 (tip fins, flaps 10 deg up) | 0.31-0.78 deg | 0.03-0.08 deg | 0.62-1.58 deg |
| Swift A foam 400 (no fin, tail bent 6 deg) | 0.87-2.23 deg | 0.04-0.11 deg | no fin |

**How far to trust these numbers.** AVL's Clb (hence the denominator D and r_hat) is known to be off for thin, low-aspect-ratio plates at Re of 10-30k: measured roll due to sideslip departs from vortex-lattice predictions because tip-vortex roll-up is destabilising in Clb (Linehan & Mohseni, cited in `research/lateral_robustness/reports/Glider robustness to build asymmetry.md`, section 'How far to trust AVL at Re 10-30k'), and CLa is uncertain by about +-30 %. These numbers therefore rank configurations and give an order of magnitude; they are not predicted radii or tolerances.

**Option for Kai (not a decision):** calibration drops with a B45 given a deliberate, measured wing-half twist of 0.5 deg and of 1.0 deg (one half only, gauged), with the circle radius measured on repeated drops. That would give the real turn per degree of twist to replace the model's order of magnitude.

Time-domain check for (a) from rest (linear, same corrected matrix, glide time 34-44 s): B45 needs 0.07-0.29 deg for 1-2 turns (its spiral is unstable, which amplifies a small input); Swift A needs 2.5-7.5 deg of tail-half difference.

What this says: asymmetries of a fraction of a degree to a couple of degrees (wing-half twist 0.1-0.3 deg, one-fin toe, a flap hinge that differs by 0.1-0.5 deg) are enough in the model to produce Kai's circles. For the Swift A foam, a tail-half bend difference alone would need a few degrees, so a small wing-half incidence difference (warp) is the likelier model route. For the Swift A the quasi-steady figure (0.9-2.2 deg for (a)) is smaller than the time-domain one because its spiral is stable with a half-life of about 13 s (at gamma = -6.61): the circle has not fully developed within a 28-36 s glide. This is a consistency statement, not proof.

**Dihedral, honestly:** its real benefit in this model is curing the B45 spiral (B45 baseline spiral doubles in 11 s; with the tip fins as flown it is stable from 3 deg of dihedral; with fins x1.5 it is still weakly unstable at 3 deg, doubling in 53.7 s, which passes part 1 only on the 15 s criterion, and stable from 6 deg). The Swift A foam's spiral is already stable at gamma = -6.61 (half-life 13 s; it only looked neutral in the first pass because AVL's base state was theta0 = 0). The apparent help of dihedral on asymmetries (a) and (c) is cancellation as Cnb falls toward zero (Swift A Cnb goes +0.0032 -> -0.0020 by 6 deg; B45 +0.0118 -> +0.0059 by 12 deg), which is a liability: the Swift A loses directional stability, the oscillation damping falls to about zero or below zero from 3-6 deg (more with Ixz), and asymmetry (b) (wing-half incidence) is not cured by any dihedral in the grid. No configuration in the grid is straight-capable.

**Glide angle caveat:** the glide angle is unmeasured. The sensitivity band used here (-5 and -8 deg) does not contain the roughly -11 deg (L/D about 6) given by `swoop_model_v1`. At -11 deg the B45 baseline spiral is stable, so the statements that the B45 spiral is unstable and that dihedral cures it depend on the assumed glide angle and are not findings until the glide angle is measured (see the post-hoc glide-angle section below and `results/runs/rigid_glider/shape_sweep_v1/gamma11_check.csv`).

**Stated plainly:** inviscid AVL (no viscous effects, flat plate, no thickness; fin and tab effectiveness are upper estimates); inertias ASSUMED (300 g/m2 foam, stick z ASSUMED); asymmetry sizes ASSUMED (1 deg each); no measured turn exists (Kai's 1-2 turns per drop is a rough count, no direction, no video from the side); the Swift A foam 400 as flown had no vertical element at all (KAI, 2026-10-21; it matches the build files, `flight_day_cg_page.py`, `swiftA_oscillation_study.py`).

### Verdict against the pre-stated rule (FIRST PASS: theta0 = 0, p' = 0 closed form, no Ixz)

**No configuration of the 20 is STRAIGHT-CAPABLE.** Parts passed: part 1 spiral 18/20; part 2 Dutch roll and roll 1/20; part 3 heading change 0/20; part 4 static margin and trim CL 4/20. Per-configuration pass/fail with the corrected frame is in the next section; the first-pass numbers are in `configs.csv`.

### Post-hoc corrected verdict (gamma = -6.61 deg, Ixz included; labelled post-hoc, the pre-stated rule above is unchanged)

| config | trim CL | spiral | oscillation zeta, no Ixz -> with Ixz (delta) | oscillation label (beta/phi ratio) | Jxz kg m2 | 1 spiral | 2 DR+roll | 3 heading (<90 deg all asym) | 4 SM+CL | pass |
|---|---|---|---|---|---|---|---|---|---|---|
| B45_fin1_d0 | 0.384 | doubles in 10.6 s | +0.064 -> +0.061 (-0.003) | Dutch roll (1.55) | +2.2e-06 | FAIL | FAIL | FAIL | pass | no |
| SwiftA_nofin_d0 | 0.296 | stable (time to half 13.4 s) | +0.039 -> +0.039 (+0.000) | Dutch roll (1.71) | +0.0e+00 | pass | FAIL | FAIL | pass | no |
| B45_fin1.5_d0 | 0.383 | doubles in 6.8 s | +0.102 -> +0.100 (-0.002) | Dutch roll (1.71) | +2.3e-06 | FAIL | pass | FAIL | pass | no |
| B45_fin1_d3 | 0.409 | stable (time to half 17.5 s) | +0.015 -> +0.005 (-0.010) | Dutch roll (1.00) | +4.9e-06 | pass | FAIL | FAIL | FAIL | no |
| B45_fin1.5_d3 | 0.409 | doubles in 53.7 s | +0.054 -> +0.046 (-0.009) | Dutch roll (1.07) | +5.3e-06 | pass | FAIL | FAIL | FAIL | no |
| B45_fin1_d6 | 0.434 | stable (time to half 5.0 s) | -0.024 -> -0.043 (-0.019) | Dutch roll (0.78) | +7.5e-06 | pass | FAIL | FAIL | FAIL | no |
| B45_fin1.5_d6 | 0.440 | stable (time to half 9.1 s) | +0.013 -> -0.004 (-0.017) | Dutch roll (0.83) | +8.3e-06 | pass | FAIL | FAIL | FAIL | no |
| B45_fin1_d9 | 0.463 | stable (time to half 2.9 s) | -0.058 -> -0.086 (-0.028) | Dutch roll (0.67) | +1.0e-05 | pass | FAIL | FAIL | FAIL | no |
| B45_fin1.5_d9 | 0.462 | stable (time to half 4.1 s) | -0.021 -> -0.048 (-0.027) | Dutch roll (0.69) | +1.1e-05 | pass | FAIL | FAIL | FAIL | no |
| B45_fin1_d12 | 0.486 | stable (time to half 2.0 s) | -0.086 -> -0.124 (-0.039) | Dutch roll (0.59) | +1.3e-05 | pass | FAIL | FAIL | FAIL | no |
| B45_fin1.5_d12 | 0.492 | stable (time to half 2.6 s) | -0.053 -> -0.090 (-0.037) | Dutch roll (0.61) | +1.4e-05 | pass | FAIL | FAIL | FAIL | no |
| SwiftA_centrefin_d0 | 0.296 | stable (time to half 18.0 s) | +0.071 -> +0.071 (-0.000) | Dutch roll (1.83) | +1.5e-08 | pass | FAIL | FAIL | pass | no |
| SwiftA_nofin_d3 | 0.317 | stable (time to half 4.6 s) | +0.004 -> -0.001 (-0.005) | Dutch roll (0.98) | +6.6e-07 | pass | FAIL | FAIL | FAIL | no |
| SwiftA_centrefin_d3 | 0.318 | stable (time to half 5.0 s) | +0.030 -> +0.025 (-0.005) | Dutch roll (1.01) | +6.7e-07 | pass | FAIL | FAIL | FAIL | no |
| SwiftA_nofin_d6 | 0.341 | stable (time to half 2.8 s) | -0.021 -> -0.034 (-0.013) | lateral oscillation (not a Dutch roll) (0.74) | +1.3e-06 | pass | FAIL | FAIL | FAIL | no |
| SwiftA_centrefin_d6 | 0.342 | stable (time to half 2.9 s) | +0.001 -> -0.012 (-0.013) | lateral oscillation (not a Dutch roll) (0.75) | +1.3e-06 | pass | FAIL | FAIL | FAIL | no |
| SwiftA_nofin_d9 | 0.369 | stable (time to half 2.0 s) | -0.041 -> -0.063 (-0.022) | lateral oscillation (not a Dutch roll) (0.62) | +1.9e-06 | pass | FAIL | FAIL | FAIL | no |
| SwiftA_centrefin_d9 | 0.369 | stable (time to half 2.0 s) | -0.023 -> -0.045 (-0.022) | lateral oscillation (not a Dutch roll) (0.62) | +1.9e-06 | pass | FAIL | FAIL | FAIL | no |
| SwiftA_nofin_d12 | 0.398 | stable (time to half 1.5 s) | -0.059 -> -0.091 (-0.033) | lateral oscillation (not a Dutch roll) (0.55) | +2.5e-06 | pass | FAIL | FAIL | FAIL | no |
| SwiftA_centrefin_d12 | 0.398 | stable (time to half 1.5 s) | -0.043 -> -0.076 (-0.033) | lateral oscillation (not a Dutch roll) (0.55) | +2.5e-06 | pass | FAIL | FAIL | FAIL | no |

Mode shapes: 6 of 20 configurations have an oscillatory lateral mode that fails the post-hoc Dutch-roll label (Cnb < 0 or |beta|/|phi| < 0.5): SwiftA_nofin_d6, SwiftA_centrefin_d6, SwiftA_nofin_d9, SwiftA_centrefin_d9, SwiftA_nofin_d12, SwiftA_centrefin_d12. These are listed as 'lateral oscillation (not a Dutch roll)' and are not credited with a Dutch roll damping ratio in part 2. Ixz changes zeta by -0.039 to +0.000 (per configuration above). Moving from theta0 = 0 (first pass) to theta0 = alpha + gamma changes the spiral eigenvalue of the B45 baseline from +0.0549 to +0.0655 1/s and of the Swift baseline from +0.0011 to -0.0517 1/s.

Glide-angle sensitivity (spiral eigenvalue, oscillation zeta, and the steady heading-rate r_hat for asymmetries (a) and (b), Ixz included; selected configurations, all 20 in the CSV; r_hat jumps by large factors where the denominator is near zero, e.g. a spiral eigenvalue near 0):

| config | gamma -5: spiral, zeta | gamma -6.61: spiral, zeta | gamma -8: spiral, zeta | r_hat (a) at -5 / -6.61 / -8 | r_hat (b) at -5 / -6.61 / -8 |
|---|---|---|---|---|---|
| B45_fin1_d0 | +0.1065 1/s, +0.066 | +0.0656 1/s, +0.061 | +0.0300 1/s, +0.057 | -0.01268 / -0.02067 / -0.04538 | +0.11651 / +0.18990 / +0.41691 |
| B45_fin1.5_d0 | +0.1460 1/s, +0.104 | +0.1017 1/s, +0.100 | +0.0631 1/s, +0.097 | -0.01142 / -0.01645 / -0.02658 | +0.11624 / +0.16747 / +0.27060 |
| B45_fin1_d3 | -0.0060 1/s, +0.008 | -0.0397 1/s, +0.005 | -0.0688 1/s, +0.002 | +0.09629 / +0.01458 / +0.00842 | -0.67690 / -0.10250 / -0.05918 |
| B45_fin1.5_d12 | -0.2444 1/s, -0.088 | -0.2707 1/s, -0.090 | -0.2935 1/s, -0.091 | -0.00230 / -0.00208 / -0.00193 | +0.01347 / +0.01219 / +0.01127 |
| SwiftA_nofin_d0 | -0.0138 1/s, +0.043 | -0.0517 1/s, +0.039 | -0.0846 1/s, +0.035 | +0.02711 / +0.00723 / +0.00443 | -0.53778 / -0.14335 / -0.08780 |
| SwiftA_centrefin_d0 | -0.0001 1/s, +0.075 | -0.0386 1/s, +0.071 | -0.0720 1/s, +0.067 | +10.36309 / +0.01090 / +0.00586 | -222.00762 / -0.23355 / -0.12544 |
| SwiftA_nofin_d3 | -0.1191 1/s, +0.002 | -0.1508 1/s, -0.001 | -0.1781 1/s, -0.004 | +0.00148 / +0.00117 / +0.00100 | +0.02723 / +0.02154 / +0.01826 |

All 20 configurations at the three glide angles are in `post_modes.csv` and `post_turns.csv`.

### Cross-check of the corrected closed form against the AVL system matrix (algebra, not physics)

Pre-stated requirement: closed form vs time integration within 5 % for at least 3 configurations. The pre-stated p' = 0 form was off by 40-2983 % in the first pass (theta0 = 0 matrix) and is physically wrong for a glide. With the corrected closed form (p' = r' tan(-gamma)) and the matrix at theta0 = alpha + gamma (Ixz included), the steady heading rate of the linear system (RK4 from rest where all modes are stable; algebraic equilibrium of A and its control columns otherwise) is compared with the closed form for the (a) differential. This verifies the algebra only; it cannot say whether AVL's base state or the 1 deg inputs describe the real glider.

| config | gamma | all modes stable | r_hat closed (corrected) | r_hat equilibrium | error % | r_hat RK4 | error % | num/unc |
|---|---|---|---|---|---|---|---|---|
| B45_fin1_d0 | g6.61 | no | -0.020672 | -0.020735 | +0.3 | - | - | 92.7 |
| SwiftA_nofin_d0 | g6.61 | yes | +0.004437 | +0.004588 | +3.4 | +0.004586 | +3.36 | 11.9 |
| SwiftA_nofin_d0 | g5 | yes | +0.016646 | +0.017187 | +3.3 | +0.017182 | +3.22 | 11.9 |
| SwiftA_nofin_d0 | g8 | yes | +0.002718 | +0.002809 | +3.4 | +0.002808 | +3.33 | 11.9 |
| B45_fin1.5_d0 | g6.61 | no | -0.016449 | -0.016397 | -0.3 | - | - | 143.1 |
| B45_fin1_d3 | g6.61 | yes | +0.014580 | +0.014835 | +1.7 | +0.014835 | +1.75 | 31.3 |
| B45_fin1_d3 | g5 | yes | +0.096286 | +0.097958 | +1.7 | +0.097925 | +1.70 | 31.3 |
| B45_fin1_d3 | g8 | yes | +0.008418 | +0.008566 | +1.8 | +0.008566 | +1.75 | 31.3 |
| B45_fin1.5_d3 | g6.61 | no | -0.079955 | -0.079490 | -0.6 | - | - | 68.4 |
| B45_fin1.5_d3 | g8 | yes | +0.051209 | +0.051092 | -0.2 | +0.051075 | -0.26 | 68.4 |
| B45_fin1_d6 | g6.61 | no | -0.000367 | -0.000387 | +5.3 | - | - | 2.5 |
| B45_fin1.5_d6 | g6.61 | no | +0.005913 | +0.005844 | -1.2 | - | - | 26.0 |
| B45_fin1_d9 | g6.61 | no | -0.002477 | -0.002519 | +1.7 | - | - | 27.0 |
| B45_fin1.5_d9 | g6.61 | no | -0.000389 | -0.000416 | +7.0 | - | - | 3.5 |
| B45_fin1_d12 | g6.61 | no | -0.003161 | -0.003161 | -0.0 | - | - | 47.4 |
| B45_fin1.5_d12 | g6.61 | no | -0.002083 | -0.002028 | -2.6 | - | - | 28.1 |
| SwiftA_centrefin_d0 | g6.61 | yes | +0.006945 | +0.006939 | -0.1 | +0.006937 | -0.12 | 15.1 |
| SwiftA_centrefin_d0 | g5 | yes | +6.601803 | +4.849343 | -26.5 | +0.738928 | -88.81 | 15.1 |
| SwiftA_centrefin_d0 | g8 | yes | +0.003730 | +0.003727 | -0.1 | +0.003726 | -0.12 | 15.1 |
| SwiftA_nofin_d3 | g6.61 | no | +0.000541 | +0.000626 | +15.8 | - | - | 2.9 |
| SwiftA_nofin_d3 | g5 | yes | +0.000684 | +0.000791 | +15.7 | +0.000791 | +15.73 | 2.9 |
| SwiftA_centrefin_d3 | g6.61 | yes | +0.000907 | +0.000884 | -2.5 | +0.000884 | -2.55 | 4.7 |
| SwiftA_centrefin_d3 | g5 | yes | +0.001181 | +0.001150 | -2.6 | +0.001150 | -2.63 | 4.7 |
| SwiftA_centrefin_d3 | g8 | yes | +0.000756 | +0.000737 | -2.6 | +0.000737 | -2.55 | 4.7 |
| SwiftA_nofin_d6 | g6.61 | no | -0.000009 | +0.000081 | -983.5 | - | - | 0.1 |
| SwiftA_centrefin_d6 | g6.61 | no | +0.000179 | +0.000181 | +1.3 | - | - | 1.3 |
| SwiftA_nofin_d9 | g6.61 | no | -0.000163 | -0.000098 | -40.2 | - | - | 1.5 |
| SwiftA_centrefin_d9 | g6.61 | no | -0.000036 | -0.000047 | +30.6 | - | - | 0.3 |
| SwiftA_nofin_d12 | g6.61 | no | -0.000204 | -0.000168 | -17.4 | - | - | 2.3 |
| SwiftA_centrefin_d12 | g6.61 | no | -0.000202 | -0.000139 | -31.4 | - | - | 2.3 |

At gamma = -6.61: 4 configurations have all modes stable; 4 of them agree with RK4 within 5 % (SwiftA_nofin_d0, B45_fin1_d3, SwiftA_centrefin_d0, SwiftA_centrefin_d3). Within 5 % at gamma = -5: 3, at -8: 5 (counts over all stable configurations). Where num/unc is below about 5, the numerator Cnb dCl - Clb dCn is at the print resolution of AVL's input run and the comparison is not informative.

### Turn sensitivity, corrected frame (1 deg of each asymmetry; gamma = -6.61, Ixz included)

R and heading change are quoted only where the steady bank stays below 30 deg (linear range); otherwise 'out of range' (raw numbers in `post_turns.csv`). Radius R = b/(2 r_hat) at 400 mm; 250 mm gives R x 0.625 and heading change x 1.6 at the same angles.

| config | asym | dCl | dCn | r_hat (heading rate) | R400 m | heading deg/100 m (400) | sideslip deg | bank deg | deg for R=31.7 m | deg for R=12.4 m |
|---|---|---|---|---|---|---|---|---|---|---|
| B45_fin1_d0 | a | +0.00044 | -0.00001 | -0.02067 | 9.7 | -592 | -1.03 | -12.4 | 0.305 | 0.780 |
| B45_fin1_d0 | b | -0.00510 | +0.00039 | +0.18990 | out of range | out of range | - | - | 0.033 | 0.085 |
| B45_fin1_d0 | c | +0.00000 | +0.00006 | -0.01020 | 19.6 | -292 | -0.81 | -6.2 | 0.619 | 1.579 |
| SwiftA_nofin_d0 | a | +0.00020 | +0.00001 | +0.00723 | 27.7 | +207 | +0.59 | +6.3 | 0.874 | 2.231 |
| SwiftA_nofin_d0 | b | -0.00683 | +0.00013 | -0.14335 | out of range | out of range | - | - | 0.044 | 0.112 |
| B45_fin1.5_d0 | a | +0.00044 | -0.00001 | -0.01645 | 12.2 | -471 | -0.68 | -10.4 | 0.384 | 0.980 |
| B45_fin1.5_d0 | b | -0.00512 | +0.00039 | +0.16747 | out of range | out of range | - | - | 0.038 | 0.096 |
| B45_fin1.5_d0 | c | +0.00001 | +0.00013 | -0.01055 | 18.9 | -302 | -0.83 | -6.7 | 0.598 | 1.527 |
| B45_fin1_d3 | a | +0.00044 | -0.00002 | +0.01458 | 13.7 | +418 | +1.14 | +8.3 | 0.433 | 1.105 |
| B45_fin1_d3 | b | -0.00506 | +0.00044 | -0.10250 | out of range | out of range | - | - | 0.062 | 0.157 |
| B45_fin1_d3 | c | -0.00001 | +0.00006 | +0.02351 | 8.5 | +674 | +1.29 | +13.2 | 0.268 | 0.686 |
| B45_fin1.5_d3 | a | +0.00044 | -0.00002 | -0.07996 | out of range | out of range | - | - | 0.079 | 0.202 |
| B45_fin1.5_d3 | b | -0.00509 | +0.00044 | +0.76432 | out of range | out of range | - | - | 0.008 | 0.021 |
| B45_fin1.5_d3 | c | -0.00001 | +0.00013 | -0.12366 | out of range | out of range | - | - | 0.051 | 0.130 |
| B45_fin1_d6 | a | +0.00043 | -0.00004 | -0.00037 | 544.6 | -11 | +0.21 | -0.2 | 17.189 | 43.890 |
| B45_fin1_d6 | b | -0.00501 | +0.00048 | +0.01169 | 17.1 | +335 | -2.09 | +6.2 | 0.540 | 1.378 |
| B45_fin1_d6 | c | -0.00001 | +0.00006 | +0.00811 | 24.7 | +232 | +0.37 | +4.3 | 0.778 | 1.987 |
| B45_fin1.5_d6 | a | +0.00043 | -0.00004 | +0.00591 | 33.8 | +169 | +0.49 | +3.2 | 1.067 | 2.726 |
| B45_fin1.5_d6 | b | -0.00503 | +0.00049 | -0.05717 | 3.5 | -1638 | -5.19 | -28.8 | 0.110 | 0.282 |
| B45_fin1.5_d6 | c | -0.00003 | +0.00014 | +0.02578 | 7.8 | +738 | +1.17 | +13.9 | 0.245 | 0.625 |
| B45_fin1_d9 | a | +0.00042 | -0.00005 | -0.00248 | 80.8 | -71 | +0.07 | -1.2 | 2.549 | 6.507 |
| B45_fin1_d9 | b | -0.00493 | +0.00053 | +0.02425 | 8.2 | +695 | -0.97 | +12.0 | 0.260 | 0.665 |
| B45_fin1_d9 | c | -0.00002 | +0.00006 | +0.00513 | 39.0 | +147 | +0.20 | +2.6 | 1.231 | 3.143 |
| B45_fin1.5_d9 | a | +0.00042 | -0.00005 | -0.00039 | 513.8 | -11 | +0.15 | -0.2 | 16.217 | 41.409 |
| B45_fin1.5_d9 | b | -0.00495 | +0.00053 | -0.00115 | 173.5 | -33 | -1.99 | -0.6 | 5.477 | 13.984 |
| B45_fin1.5_d9 | c | -0.00004 | +0.00014 | +0.01318 | 15.2 | +377 | +0.52 | +6.8 | 0.479 | 1.223 |
| B45_fin1_d12 | a | +0.00041 | -0.00006 | -0.00316 | 63.3 | -91 | +0.01 | -1.5 | 1.997 | 5.098 |
| B45_fin1_d12 | b | -0.00483 | +0.00057 | +0.02612 | 7.7 | +748 | -0.58 | +12.2 | 0.242 | 0.617 |
| B45_fin1_d12 | c | -0.00003 | +0.00006 | +0.00386 | 51.8 | +111 | +0.14 | +1.8 | 1.634 | 4.171 |
| B45_fin1.5_d12 | a | +0.00041 | -0.00006 | -0.00208 | 96.0 | -60 | +0.05 | -1.0 | 3.030 | 7.737 |
| B45_fin1.5_d12 | b | -0.00484 | +0.00057 | +0.01219 | 16.4 | +349 | -1.08 | +5.9 | 0.518 | 1.323 |
| B45_fin1.5_d12 | c | -0.00006 | +0.00014 | +0.00913 | 21.9 | +262 | +0.33 | +4.4 | 0.691 | 1.765 |
| SwiftA_centrefin_d0 | a | +0.00020 | +0.00001 | +0.01090 | 18.3 | +312 | +0.79 | +9.7 | 0.579 | 1.478 |
| SwiftA_centrefin_d0 | b | -0.00685 | +0.00012 | -0.23355 | out of range | out of range | - | - | 0.027 | 0.069 |
| SwiftA_centrefin_d0 | c | +0.00004 | +0.00001 | +0.00556 | 36.0 | +159 | +0.31 | +5.0 | 1.135 | 2.898 |
| SwiftA_nofin_d3 | a | +0.00020 | +0.00001 | +0.00117 | 170.3 | +34 | +0.17 | +1.0 | 5.374 | 13.721 |
| SwiftA_nofin_d3 | b | -0.00679 | +0.00016 | +0.02154 | 9.3 | +617 | -4.02 | +17.0 | 0.293 | 0.748 |
| SwiftA_centrefin_d3 | a | +0.00020 | +0.00001 | +0.00164 | 122.1 | +47 | +0.19 | +1.4 | 3.854 | 9.840 |
| SwiftA_centrefin_d3 | b | -0.00681 | +0.00016 | +0.01388 | 14.4 | +398 | -4.38 | +11.3 | 0.455 | 1.161 |
| SwiftA_centrefin_d3 | c | +0.00004 | +0.00001 | +0.00183 | 109.2 | +52 | +0.09 | +1.5 | 3.447 | 8.800 |
| SwiftA_nofin_d6 | a | +0.00020 | +0.00001 | +0.00027 | 743.5 | +8 | +0.10 | +0.2 | 23.465 | 59.915 |
| SwiftA_nofin_d6 | b | -0.00673 | +0.00020 | +0.04180 | 4.8 | +1198 | -2.00 | +28.6 | 0.151 | 0.386 |
| SwiftA_centrefin_d6 | a | +0.00020 | +0.00001 | +0.00049 | 411.4 | +14 | +0.10 | +0.4 | 12.984 | 33.153 |
| SwiftA_centrefin_d6 | b | -0.00676 | +0.00020 | +0.03943 | 5.1 | +1130 | -2.11 | +27.6 | 0.160 | 0.409 |
| SwiftA_centrefin_d6 | c | +0.00005 | +0.00001 | +0.00089 | 225.2 | +25 | +0.04 | +0.7 | 7.108 | 18.151 |
| SwiftA_nofin_d9 | a | +0.00019 | +0.00001 | -0.00002 | 12844.0 | -0 | +0.06 | -0.0 | 405.365 | 1035.055 |
| SwiftA_nofin_d9 | b | -0.00665 | +0.00024 | +0.04431 | 4.5 | +1269 | -1.23 | +27.9 | 0.142 | 0.364 |
| SwiftA_centrefin_d9 | a | +0.00019 | +0.00001 | +0.00012 | 1685.8 | +3 | +0.07 | +0.1 | 53.205 | 135.852 |
| SwiftA_centrefin_d9 | b | -0.00667 | +0.00023 | +0.04342 | 4.6 | +1244 | -1.28 | +27.8 | 0.145 | 0.371 |
| SwiftA_centrefin_d9 | c | +0.00005 | +0.00001 | +0.00051 | 392.7 | +15 | +0.03 | +0.4 | 12.395 | 31.648 |
| SwiftA_nofin_d12 | a | +0.00019 | +0.00001 | -0.00012 | 1654.1 | -3 | +0.05 | -0.1 | 52.203 | 133.295 |
| SwiftA_nofin_d12 | b | -0.00654 | +0.00027 | +0.04231 | 4.7 | +1212 | -0.85 | +24.9 | 0.149 | 0.381 |
| SwiftA_centrefin_d12 | a | +0.00019 | +0.00001 | -0.00012 | 1728.8 | -3 | +0.05 | -0.1 | 54.561 | 139.317 |
| SwiftA_centrefin_d12 | b | -0.00657 | +0.00026 | +0.04191 | 4.8 | +1201 | -0.88 | +25.1 | 0.151 | 0.385 |
| SwiftA_centrefin_d12 | c | +0.00005 | +0.00001 | +0.00028 | 707.2 | +8 | +0.02 | +0.2 | 22.319 | 56.989 |

### Trim options for the turn (options only, none chosen)

**Option 1: fin trim tab** (fin or rudder trim is speed-sensitive, which is known free-flight practice and matters for a nose-down drop whose speed changes a lot; hinge at 70 % fin chord, ASSUMED; right tip fin on B45, centre fin on Swift A; r_hat = 0 for each 1 deg asymmetry separately, leaving a steady sideslip). Tab angles from the first pass (the tab numerator condition does not depend on the closed-form frame):

| config | role | asym | tab deflection deg (TE toward +y) | residual sideslip deg | tab dCn per deg |
|---|---|---|---|---|---|
| B45_fin1.5_d12 | best (pre-stated ranking) | a | +0.18 | +0.14 | 1.7e-04 |
| B45_fin1.5_d12 | best (pre-stated ranking) | b | -1.05 | -1.53 | 1.7e-04 |
| B45_fin1.5_d12 | best (pre-stated ranking) | c | -0.79 | -0.01 | 1.7e-04 |
| B45_fin1_d12 | best (pre-stated ranking) | a | +0.57 | +0.13 | 8.6e-05 |
| B45_fin1_d12 | best (pre-stated ranking) | b | -4.75 | -1.56 | 8.6e-05 |
| B45_fin1_d12 | best (pre-stated ranking) | c | -0.70 | -0.01 | 8.6e-05 |
| B45_fin1_d6 | best (body-rhat ranking) | a | +0.03 | +0.25 | 8.5e-05 |
| B45_fin1_d6 | best (body-rhat ranking) | b | -1.00 | -2.55 | 8.5e-05 |
| B45_fin1_d6 | best (body-rhat ranking) | c | -0.70 | -0.05 | 8.5e-05 |
| B45_fin1_d0 | EXTRA (baseline with fin, not in the pre-stated list) | a | -1.37 | +0.61 | 8.5e-05 |
| B45_fin1_d0 | EXTRA (baseline with fin, not in the pre-stated list) | b | +12.56 | -7.06 | 8.5e-05 |
| B45_fin1_d0 | EXTRA (baseline with fin, not in the pre-stated list) | c | -0.67 | -0.01 | 8.5e-05 |
| SwiftA_centrefin_d0 | EXTRA (baseline with fin, not in the pre-stated list) | a | -1.56 | +0.24 | 1.4e-05 |
| SwiftA_centrefin_d0 | EXTRA (baseline with fin, not in the pre-stated list) | b | +33.44 | -9.18 | 1.4e-05 |
| SwiftA_centrefin_d0 | EXTRA (baseline with fin, not in the pre-stated list) | c | -0.80 | +0.03 | 1.4e-05 |

**Option 2: differential centre flap (B45) / differential tail halves (Swift A), a roll trim.** Differential angle (D1 - D2, so half on each side with opposite sign) that zeroes the r_hat numerator Cnb dCl - Clb dCn, from the control response already measured in the main run (flap R/L on B45, tail-half R/L on Swift A), and the residual steady sideslip. Flagged '>10' where the angle is beyond the range the existing runs support (+-5 deg each side). Low-Re flap and tail effectiveness is below AVL's, so these angles (for example 9.2 deg for B45 asymmetry (b)) are lower bounds. Wing-tip trailing-edge surfaces do not exist in the model, so a wing-tip number cannot be computed from the existing data.

| config | asym | differential deg | per side deg | residual sideslip deg | beyond 10 deg |
|---|---|---|---|---|---|
| B45_fin1_d0 | a | -1.00 | -0.50 | +0.00 | no |
| B45_fin1_d0 | b | +9.19 | +4.59 | -1.45 | no |
| B45_fin1_d0 | c | -0.49 | -0.25 | -0.31 | no |
| SwiftA_nofin_d0 | a | -1.63 | -0.81 | -0.03 | no |
| SwiftA_nofin_d0 | b | +32.31 | +16.15 | -3.42 | >10 |
| B45_fin1.5_d12 | a | -1.00 | -0.50 | +0.00 | no |
| B45_fin1.5_d12 | b | +5.85 | +2.93 | -0.78 | no |
| B45_fin1.5_d12 | c | +4.38 | +2.19 | +0.56 | no |
| B45_fin1_d12 | a | -1.00 | -0.50 | +0.00 | no |
| B45_fin1_d12 | b | +8.26 | +4.13 | -0.47 | no |
| B45_fin1_d12 | c | +1.22 | +0.61 | +0.15 | no |
| B45_fin1_d6 | a | -1.00 | -0.50 | +0.00 | no |
| B45_fin1_d6 | b | +31.85 | +15.92 | +4.60 | >10 |
| B45_fin1_d6 | c | +22.09 | +11.05 | +5.00 | >10 |

All configurations and asymmetries: `roll_trim.csv`. Both options null the numerator, not the sideslip; the glider then flies straight but slipping. Which to build is Kai's decision.

### Why does the trim CL rise with dihedral at constant static margin? (diagnosis)

| config | Xnp mm | CLa /rad | Cm about NP (z-ref 0) | SM % MAC | CL = CmNP/SM | CG height modelled mm | CL_trim (CG height modelled) | CL_trim (CG height 0) | CL_trim (baseline CG height) |
|---|---|---|---|---|---|---|---|---|---|
| B45_fin1_d0 | 70.30 | 3.773 | +0.01867 | 4.66 | 0.400 | -2.12 | 0.384 | 0.401 | 0.384 |
| B45_fin1_d6 | 71.12 | 3.742 | +0.01798 | 4.66 | 0.385 | +4.51 | 0.434 | 0.396 | 0.382 |
| B45_fin1_d12 | 71.90 | 3.642 | +0.01716 | 4.66 | 0.368 | +11.18 | 0.486 | 0.386 | 0.373 |
| SwiftA_nofin_d0 | 41.64 | 5.007 | +0.02895 | 9.80 | 0.296 | +0.00 | 0.296 | 0.296 | 0.296 |
| SwiftA_nofin_d6 | 42.72 | 4.993 | +0.02742 | 9.80 | 0.280 | +4.85 | 0.341 | 0.317 | 0.317 |
| SwiftA_nofin_d12 | 43.72 | 4.864 | +0.02622 | 9.80 | 0.268 | +9.83 | 0.398 | 0.338 | 0.338 |

Decomposition of the change in trim CL from 0 to 12 deg (AVL, from the table above):

- B45: total +0.102 = moment-about-the-NP term -0.032 (Cm about the NP falls -8 %, SM held) + change of the modelled CG height above the wing plane +0.118 (CG height -2.1 -> +11.2 mm) + remainder with the CG on the reference plane +0.017.
- Swift A: total +0.102 = moment-about-the-NP term -0.028 (Cm about the NP falls -9 %, SM held) + change of the modelled CG height above the wing plane +0.060 (CG height +0.0 -> +9.8 mm) + remainder with the CG on the reference plane +0.070.

So the rise is NOT a lift-tilt or static-margin effect (SM is held and the NP moment falls). For B45 the largest part is the CG height (for Swift A the CG height and the remainder are comparable): the dihedral lifts the wing-half masses, the point-mass CG moves above the wing plane, and at alpha of 5-9 deg the axial force acting at that height adds a nose-up moment the trim has to cancel. That part depends on the ASSUMED stick position (z = -6 mm) and the point-mass model, so its size is model-dependent. The remainder (non-zero even with the CG forced onto the reference plane, larger for the Swift A) is not isolated here; the likely cause is the same axial-force/height coupling acting at the raised wing halves, but that is an inference, not a test. A 're-trim needed' statement is therefore conditional: the real CG height and the real flap/tail effectiveness decide it, and the model alone does not settle it.

### Post-hoc comparison with Kai's reported turns, details

Glide height 18.1-23.1 m, path 156-199 m, R band 12.4-31.7 m (glide angle 6.61 deg). Time-domain turns per 1 deg of (a), corrected matrix:

| glider | release | glide time s | turns per 1 deg | bank at end per 1 deg | deg for 1 turn | deg for 2 turns |
|---|---|---|---|---|---|---|
| B45_fin1_d0 | 25 m | 34.1 | 6.951 | 104.5 | 0.144 | 0.288 |
| B45_fin1_d0 | 30 m | 43.5 | 14.848 | 204.7 | 0.067 | 0.135 |
| SwiftA_nofin_d0 | 25 m | 28.4 | 0.265 | 3.1 | 3.771 | 7.541 |
| SwiftA_nofin_d0 | 30 m | 36.2 | 0.395 | 3.4 | 2.533 | 5.066 |

Plausible for hand-built foamboard (fractions of a degree to a couple of degrees = plausible): wing-half incidence 0.1-0.3 deg yes; one-fin toe and B45 flap-hinge differences yes; Swift A tail-half bend difference of several degrees: borderline to unlikely for 'slightly more on one side'. The model has not been checked against any measured turn.

### Glide-angle check at -11 deg (post-hoc)

The glide angle of the real gliders has not been measured. The sensitivity band above (-5 and -8 deg) was chosen around the -6.61 deg of the oscillation model; the swoop model (`swoop_model_v1`) gives about -11 deg (L/D about 6), which lies outside that band. The same pipeline was rerun at -11 deg in `results/runs/rigid_glider/shape_sweep_v1/gamma11_check.csv` (values copied below):

| quantity | gamma -6.61 deg | gamma -11 deg |
|---|---|---|
| B45 as flown, spiral eigenvalue 1/s | +0.0648 (unstable) | -0.0482 (stable) |
| Swift A as flown, spiral eigenvalue 1/s | -0.0518 | -0.1556 |
| B45 as flown, oscillation zeta | +0.061 | +0.049 |
| Swift A as flown, oscillation zeta | +0.039 | +0.027 |
| B45 degrees for R = 12-32 m scale, (a) / (b) / (c), tw30 columns of the check file | 0.319 / 0.035 / 0.646 | 0.235 / 0.026 / 0.477 |
| Swift A same scale, (a) / (b) | 1.505 / 0.047 | 4.500 / 0.139 |

Consequences: (1) whether the B45 spiral is unstable, and therefore whether dihedral is needed to cure it, depends on the glide angle; it is a conditional statement, not a finding, until the glide angle is measured. (2) The degrees of asymmetry needed for a given circle change by a factor of about 1.4 (B45) to 3 (Swift A) between -6.61 and -11 deg, so the order-of-magnitude reading above stands but the individual numbers do not. (3) The sign of Cnb (loss of directional stability of the Swift A from 6 deg of dihedral) does not depend on the glide angle; the oscillation damping of the dihedral variants was not rerun at -11 deg.

### Limits

- AVL is inviscid, flat plate, no thickness; real fins and tabs at Re of a few thousand to 30,000 give less than AVL. Inertias and masses ASSUMED; asymmetry sizes ASSUMED; the results are linear in the asymmetries only up to bank about 30 deg.
- AVL's base state: constant speed, thrust balancing drag implicitly, no nonlinear effects (stall); the glide angle gamma is an input (MODEL for Swift, ASSUMED for B45), sensitivity -5 and -8 deg above.
- The product of inertia is from point masses, other products are 0; the stick height (z = -6 mm, ASSUMED) sets the CG height, which drives the trim-CL behaviour diagnosed above.
- Where the denominator D is near zero (an unstable or slow spiral: B45 baseline, B45 with fins x1.5 at 3 deg, centre-fin Swift at gamma = -5) the steady-turn r_hat is hypersensitive; the time-domain numbers are the better guide there.
- Phase 1 only: lateral side. No CAD, no STL, no pitch simulation.

### Files

`configs.csv`, `turn_sensitivity.csv` and `crosscheck.csv` (FIRST PASS: theta0 = 0, pre-stated and tan-alpha forms), `post_modes.csv`, `post_turns.csv`, `post_crosscheck.csv`, `roll_trim.csv`, `trim_tab.csv`, `trim_cl_diagnosis.csv`, `posthoc_turns.csv`, `runs.json`, `index.html`, `avl_work/`, `prestated_section.sha256`. Code: `rigid_glider/straight_smooth_v1.py`; tests `rigid_glider/test_straight_smooth_v1.py`.

### Tests (2026-10-21 20:01, py -B rigid_glider/test_straight_smooth_v1.py)

Ran 25 tests in 81.2 s: OK (25 passed, 0 failures). Includes the formula and sign tests, mass-model checks against geometry.json, baseline Xnp and static margin reproduction (B45 70.30 mm, Swift A 41.70 mm, both within 0.5 mm), the elevation / Ixz / mode-shape / corrected cross-check / roll-trim tests, and test_prestated_section_unchanged: the sha256 of the README bytes before the line RESULTS equals b70e80e8af1bab7996eb411ed1a468fbf3c4fe403e6e65a2e75b61773becc0b6 (prestated_section.sha256), so the pre-stated section is unchanged.
