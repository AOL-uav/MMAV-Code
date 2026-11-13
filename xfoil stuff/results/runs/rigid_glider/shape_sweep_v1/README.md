# Shape sweep v1: planform families and stability features, scored on lateral robustness and pitch margin (model study, nothing here is a flight result)

## Question, method and pre-stated rule

Written 2026-10-21 17:35 (local), BEFORE any grid configuration of the sweep was run. This section is not edited after the run; everything below the line "RESULTS" is filled in afterwards by `rigid_glider/shape_sweep_v1.py`. Before this text was written only plumbing was run: the B45 and Swift A as-flown references against `straight_smooth_v1` (reproduced, see Tests), one T sample, one B sample with a centre fin and the Zanonia reference, plus a geometry-only check (no AVL) that all 54 outlines of the two families build, mesh in `sheet_builder` and fit the bed. No result of those runs was used to choose a level or a threshold below, except two things stated openly: (i) the kink station of the dihedral panel is snapped to the nearest AVL section node for the B family and the Zanonia reference (AVL refused a sliver interval at exactly 0.6 of the semi-span), (ii) the drag-profile constant CD0 below was calibrated on Swift A as flown.

### Question

Kai's two 400 mm foamboard gliders (B45 flat wing, Swift A bent tail) glide but settle into a steady turn and show pitch swoops. The earlier phase (`straight_smooth_v1`) showed that tuning dihedral and fins of those two shapes does not make either straight-capable. Kai wants to see the FULL range of shapes, not tweaks of B45 and Swift A, and to end with good candidate shapes for straight, smooth passive flight that are robust to build error. This study is a bounded, structured sweep (NOT an optimiser) over two planform families plus a small set of stability features, scored on lateral robustness (how large a wing-half twist the shape tolerates before it circles tighter than R = 30 m, spiral mode, oscillation damping) and on pitch margin (trim angle of attack against the thin-plate lift-slope break). The question is whether any shape in the sweep passes the pre-stated candidate rule, which shapes come closest, and which features do the work.

### Models (MODEL = inviscid AVL 3.52, flat plate, thickness ignored; AVL serial)

All configurations are at 400 mm span. Everything reuses `straight_smooth_v1` (imported, not re-derived): explicit two-half decks with a shared COMPONENT index per physical surface, AVL controls reset between runs by always sending D1/D2/alpha, 10x amplified asymmetry runs (AMP = 10) to beat the 5-decimal print resolution, the corrected steady-turn closed form with p' = r' tan(-gamma), base-state elevation theta0 = alpha + gamma (MODE parameter E) and the product of inertia from point masses with AVL's IZX sign convention (IZX = minus the integral of x z dm).

- **Family T (trapezoid flying wing, 27 outlines):** `flat_test.wing_geometry(400, AR, taper, quarter-chord sweep)`; AR {3.5, 4.5, 6} x quarter-chord sweep {15, 25, 35} deg x taper {0.3, 0.5, 0.8}. (AR 4.5, 25 deg, 0.5 is the B45 / P07 outline.) B45-style tip fins in Stage 1 (24 mm tall, chord = tip chord, vertical, centred on the tip). Pitch trim by a reflex flap (AVL control, both halves) over 0.15..0.40 of the semi-span (B45: y = 30..80 mm), chord 15 % of the local chord (ASSUMED; B45 as flown has 10 mm, about 9 %), deflection TE up.
- **Family B (bird / Swift lineage with body-blended fan tail, 27 outlines):** `bird_outline_v4_swift.build_geometry4`. Four parameters, three levels each, from the Swift A GLIDE recipe: wing-alone aspect ratio ar_target {7.0, 8.5 (A), 10.0}; hand leading-edge sweep shift {-8, 0 (A), +8} deg added to the three hand knots; wrist position wrist_eta {0.05, 0.139 (A), 0.30} (hand-wing fraction 1 - wrist_eta = 0.95, 0.861, 0.70); fan-tail area as a fraction of the wing area {0.10, 0.185 (A), 0.30} (`with_tail_area`). The 27 outlines are the 3^(4-1) fractional factorial: all 27 combinations of the first three parameters and the tail level d = ((a + h + w + 1) mod 3) - 1 (levels coded -1, 0, 1), so that (0,0,0,0) is Swift A exactly (`s4.GLIDE` itself). Pitch trim by the tail bend (AVL control on both tail halves, full-chord rotation about the section leading edge, TE up, the same convention as `straight_smooth_v1`'s tail ainc). No fin in Stage 1.
- **References (3, fixed CG and setting, not targeted):** B45 as flown (CG 66.0 mm aft of root LE, centre flaps 10 mm chord at y = 30..80 mm 10 deg up, tip fins, foam mass model of `straight_smooth_v1`), Swift A as flown (CG 37.0 mm aft of root LE = 73.0 mm from the nose tip, tail ainc -6 deg, no fin, foam mass model), Zanonia scaled to 400 mm (outline from `research/known_good/zanonia`, 30 % outer curl with 2.2 deg tip incidence, CG at the marked cross, SM about 1 % MAC, plate 100 g/m2 ASSUMED). Their gamma is -6.61 deg (as in `straight_smooth_v1`). These three are reported against the rule but are NOT eligible as candidates (their CG / setting is not the sweep's SM 12 % / CL target).
- **Trim in the sweep:** static margin held at 12 % MAC (CG = Xnp - 0.12 MAC, solved with the mass model, CG height from the mass model); the trim CL target is reached by the reflex flap (T) or the tail bend (B): (alpha, delta) solved to CL = target and Cm = 0 about the CG (linear model from four AVL states, then verified and corrected by AVL runs until |CL - target| < 0.002, |Cm| < 1.5e-4, |SM - 12 %| < 0.05 %). A configuration whose delta exceeds 25 deg, or which does not converge, is called trim-INFEASIBLE and fails rule 4.
- **Glide angle:** gamma = -atan((CD0 + CD_AVL) / CL_trim), CD_AVL = AVL induced drag at trim, CD0 = 0.0307 (ASSUMED profile drag at Re of 1.2-1.5e4, calibrated so that Swift A as flown, CL 0.296 and CD_AVL 0.0036, gives the -6.61 deg of `swiftA_oscillation`). Reported per configuration.
- **Mass (printed versions):** sheet 2 layers x 0.1 mm = 0.2 mm PLA (1.24 g/cm3 ASSUMED), `sheet_builder` DEFAULT bead (3.0 -> 1.2 mm wide, 1.5 -> 0.6 mm tall, to 0.9 of the semi-span), DEFAULT root strip and rail, glue allowances: the bare-kit masses are CALCULATED from the sheet_builder STL volumes (`build_kit` per outline, dihedral 0, tail flat, no fins); fins are 0.6 mm PLA plates (T tip fins 24 mm x tip chord, B centre fin by area), lead ballast (11.3 g/cm3 ASSUMED) at the kit's nose station, sized for the CG. Mass distribution for the inertias is a point-mass model (ASSUMED). Wing loading = all-up weight / planform area S. Tags: CALCULATED, ASSUMED. The foam references use the `straight_smooth_v1` foam mass models (ASSUMED 300 g/m2).
- **Bed fit:** `sheet_builder` fit check of one wing half at 400 mm (`build_kit` -> `fit_unsplit`: the half fits a 250 x 250 mm bed in some rotation at 0.5 deg steps).

### Stage 1: outline screen (54 outlines + 3 references)

Flat (no dihedral), SM 12 % MAC, trim CL 0.30. T carries the tip fins (cant 0); B has no fin.

### Stage 2 selection rule (stated before the run)

Over the 54 Stage-1 outlines (references excluded), filter F = {trim feasible, Cnb > 0, fits the bed}. In each family separately, rank the members of F by twist tolerance (rule 3 quantity, larger better) and by pitch margin (8 deg minus trim alpha, larger better); score = sum of the two ranks (ties by larger twist tolerance); take the 5 best of each family, 10 outlines in all. If a family has fewer than 5 members in F it is filled by the same score from outside F (and flagged). Twist tolerance is taken as computed (the closed form), whatever the spiral eigenvalue.

### Stage 2 (features on the 10 selected outlines)

Tip dihedral on the outer 40 % of the semi-span (kink at 0.6 of the semi-span, outer panel rotated rigidly about the kink, inner part flat): {0, 10, 20} deg. Fins: T: none / tip fins cant 0 / tip fins canted 20 deg outward (top of the fin outboard, rotated about the fin centre); B: none / centre fin at the tail end sized 2 % S / 4 % S (`fin_kit_v4` 'tail' option, vertical, base on the wing plane). Trim CL {0.25, 0.35} with SM 12 % held. 3 x 3 x 2 = 18 per outline, 180 in all. Total AVL configurations 54 + 3 + 180 = 237; cap 300: if the grid would exceed 300 the run stops and reports. (Stage-1 flat / default-fin / CL 0.30 results are the dihedral 0 anchors of each outline; the CL 0.30 level of the feature combinations is not run.)

### Asymmetries and twist tolerance

Same definitions as `straight_smooth_v1`: (a) differential deflection of the trim surface by 1 deg (flap or tail halves, through AVL's control columns), (b) wing-half incidence difference of 1 deg (right half +0.5, left -0.5, explicit halves), (c) 1 deg toe of one fin (T: right tip fin; B: the centre fin), where a fin exists. dCl, dCn from AVL, r_hat from the corrected closed form at the configuration's gamma (heading rate). **Twist tolerance TW30** = the wing-half incidence difference (asymmetry b, degrees) that gives a steady circle of R = 30 m: TW30 = 1 deg x (b / (2 R)) / |r_hat per deg| with b = 0.4 m, i.e. (0.4 / 60) / |r_hat per deg|. Linear scaling from 1 deg (valid to bank about 30 deg; stated, not enforced). TW30 for (a) and (c) is reported the same way.

### Pre-stated candidate rule (not to be changed after the run)

A configuration is a CANDIDATE if ALL hold, at 400 mm, trimmed (SM 12 % MAC, CL target), theta0 = alpha + gamma, Ixz included:
1. **Spiral stable:** the spiral eigenvalue is negative (time to half, any) or the time to double is more than 30 s (eigenvalue < ln 2 / 30 = 0.0231 1/s).
2. **Oscillation:** the lateral oscillatory mode has damping ratio zeta >= 0.10 AND Cnb > 0 (an oscillation with Cnb < 0 is not credited) AND the roll-subsidence eigenvalue is negative. If the lateral block has no complex pair (overdamped) zeta counts as 1 and the Cnb > 0 condition still applies.
3. **Twist tolerance:** TW30 (asymmetry b) >= 3 x the TW30 of the B45 as-flown reference computed by the same script in the same frame (gamma -6.61, Ixz); `straight_smooth_v1`'s table gives 0.033 deg at R = 31.7 m, i.e. about 0.035 deg at R = 30 m, so the threshold is about 0.105 deg.
4. **Pitch:** static margin within 8-20 % MAC AND trim alpha <= 5.0 deg (at least 3 deg below the 8 deg break of the thin-plate lift slope, Okamoto & Jinba 2009, measured at Re about 1.2-1.5e4, as cited in `research/lateral_robustness/reports/Glider robustness to build asymmetry.md`) AND the trim is feasible and converged (delta <= 25 deg).
5. **Printable:** each wing half fits the 250 x 250 mm bed (`sheet_builder` fit check) at 400 mm.

References are reported against the same five parts but are not eligible.

### Scoring, Pareto, ranking and correlation (stated before the run)

- Pitch margin = 8 deg - trim alpha (deg). Pareto plot: x = TW30 (asymmetry b, log axis), y = pitch margin; the Pareto front is taken over trim-feasible configurations (non-dominated in both, larger better). Candidates are highlighted.
- **Top 3** = the candidates ranked by TW30 (larger first), at most one configuration per outline. If fewer than 3 configurations are candidates, the list is filled with NEAR-MISS configurations ranked by (number of the five parts passed, larger first), then by TW30; the same one-per-outline rule; they are labelled NEAR-MISS. The one-per-outline rule applies across the whole list (candidates and near-misses together); within an outline the configuration with the highest rank under the same ordering is the one listed.
- **Rawdon SS** = EDA (deg) x (lv / b) / CL_trim, EDA = outer-panel dihedral weighted by the share of integral(c y dy) outboard of the kink, lv = CG to quarter chord of the fin if present, else of the tail (B) or of the tip chord (T). Computed per configuration, not part of the rule. Spearman rank correlation (ties averaged) between SS and TW30 over (i) all configurations of Stages 1 and 2 with a converged trim, (ii) Stage 2 only; reported with the number of ties. Stage 1 has EDA 0 for every outline, so SS = 0 there and carries no ranking information.
- yaw floor = 1 / |Cnr| (per rad), reported.

### Outputs

`configs.csv` (all metrics; mass and wing loading tagged CALCULATED / ASSUMED), `sweep.gif` (one frame per outline: 57 Stage-1 / reference frames, then 90 Stage-2 frames = 10 outlines x 3 dihedral x 3 fin options, plan view to a common scale with the front view showing dihedral and fins, scores printed on each frame; both Stage-2 trim CL levels are listed on the frame; about 0.6 s per frame; dark background), `contact_sheet.svg`, `index.html` (dark; sortable table, Pareto plot, candidates highlighted), `sheet_builder_top3/` (a sheet_builder JSON per top-3 configuration plus a kit run into a new subfolder), `runs.json`, `prestated_section.sha256`. AVL scratch files go to a work folder outside this OneDrive folder.

### Limits (stated now)

AVL is inviscid and treats thin low-aspect-ratio plates optimistically: Clb is unreliable at Re 10-30k (tip-vortex roll-up is destabilising, Linehan & Mohseni), so TW30, SS and the derivatives are for RANKING configurations, not for predicting radii or tolerances; CLa is uncertain by about +-30 %. Masses are partly CALCULATED (kit volumes) but the lead ballast, the point-mass distribution, the inertias, the flap / tail-bend effectiveness at low Re, the profile drag CD0, the asymmetry sizes (1 deg) are ASSUMED. The sheet_builder kit prints a uniform half-dihedral from the rail, not an outer-panel kink, and no canted fins: the printed JSON for a candidate with tip dihedral or canted fins is therefore only the flat kit plus a note. No pitch drop-simulation and no prints are part of this study; the pitch margin is a screening quantity (trim alpha against the 8 deg break), not a swoop prediction. A pass here is not a flight result.

RESULTS
Filled in 2026-10-21 18:33 (local) by `rigid_glider/shape_sweep_v1.py`; AVL model time 32 min (sum of per-configuration times; wall clock of the compute run 32 min). MODEL = inviscid AVL 3.52; CALCULATED = sheet_builder STL volumes; ASSUMED = lead ballast, point-mass inertias, CD0, flap / tail-bend effectiveness, asymmetry sizes.

### Summary (read this first)

- **No configuration passes the pre-stated rule: 0 candidates of 234 eligible configurations** (54 outlines flat, 180 Stage-2 variants). Part 5 (bed) passes everywhere; part 2 (oscillation damping >= 0.10 with Cnb > 0) fails in 232; the best damping of any feasible configuration with Cnb > 0 is +0.095 (T_AR6_S15_L0.8__d0__tip0__cl0.35), against +0.061 for B45 and +0.039 for Swift A as flown. Part 2 is also the least certain part (point-mass inertias are ASSUMED).
- **Pre-stated top 3** (no candidates, so the near-miss rule fills it): B_AR8.5_H-8_W30_T18.5__d20__c2__cl0.25 (twist 9.12 deg, zeta +0.007, alpha 3.6); B_AR8.5_H-8_W5_T30__d20__c2__cl0.35 (twist 5.09 deg, zeta +0.014, alpha 4.9); B_AR8.5_H+0_W30_T30__d20__c2__cl0.35 (twist 1.16 deg, zeta +0.012, alpha 4.9). All three are Swift-lineage (family B) with 20 deg tip dihedral and a 2 % S centre fin and carry the marker 'cancellation: twist number not credible'; their huge twist tolerances are cancellations of the r_hat numerator (cancellation ratio 0.01, 0.01, 0.05 against 0.59 for B45), so they are not good evidence. See the POST-HOC reading below.
- **Post-hoc pick (a trade, not a candidate; details in the POST-HOC reading):** B_AR7_H-8_W13.9_T30__d20__c4__cl0.25: twist tolerance 3.5 x B45, but fin-toe tolerance worse than B45, zeta +0.051 below B45, trim speed 9.4 m/s and mass 29.4 g (printed, 10.2 g lead). It fails part 2.
- **Features (Stage 2 medians):** tip dihedral 0 -> 20 deg multiplies the twist tolerance by x9.93 (T) and x8.63 (B) but moves zeta by -0.097 and -0.040 and Cnb by -0.0002 and -0.0005; tip fins on T add +0.0057 to Cnb, canting them 20 deg outward changes zeta by -0.015 (no gain); a centre fin on B adds +0.0089 (2 % S) and +0.0169 (4 % S) to Cnb and +0.036 / +0.055 to zeta; trim CL 0.25 -> 0.35 raises the trim alpha by +2.12 (T) / +1.39 deg and does not move the twist tolerance systematically (median ratio x1.08 for B).
- **Outlines (Stage 1, flat):** family B trims at alpha 3.9-6.5 deg with a tail bend of 5-21 deg and is spiral-stable in 26 of 26; family T with a 15 % flap trims at 6.0-8.0 deg (never at or below 5 deg) and 7 of 27 T outlines cannot be trimmed within 25 deg of flap. No flat outline reaches the twist tolerance of rule 3 (0.104 deg); the best flat outline reaches 0.052 deg. Dihedral is what moves it.
- **Printed mass:** printed versions are 12-54 g (lead ballast 1-31 g) because the CG target of SM 12 % is far ahead of the bare printed sheet's CG; B45 foam is 17.8 g and Swift A foam 9.8 g. The dynamic numbers carry that mass and inertia. A lighter build (foam, or moving mass into the fin and tail) was not swept.
- **Glide angle is unmeasured.** This sweep uses a per-configuration gamma from CD0 = 0.0307 (about -6.6 deg for Swift A); `swoop_model_v1` gives about -11 deg (L/D about 6). The modes and closed form were rerun at -11 deg for the references, the top 3 and the post-hoc pick (section 'Post-hoc checks' below; `gamma11_check.csv`).
- **Direction is a lateral-only ranking.** Swift A as flown passes rule 4 (alpha 4.37 deg) yet swooped heavily with stalls in flight, and neither pitch model explains that, so pitch behaviour of the B family (and of every configuration here) is unvalidated; rule 4 and the pitch margin are screening quantities only.
- **Next flight day:** record the flights from the side or from behind to check for a yaw-roll wobble; the existing video is from beneath and says nothing about lateral oscillation. Measured glide angle and a gauged wing-half twist test would calibrate the two largest model uncertainties.
- **Caveat, rank do not predict:** inviscid AVL on thin low-aspect-ratio plates (Clb unreliable at Re 10-30k, CLa about +-30 %); every number here ranks configurations and is not a flight prediction.

### Counts

237 AVL configurations: 3 references + 54 Stage-1 outlines (27 T + 27 B) + 180 Stage-2 variants (10 outlines x 3 dihedral x 3 fin options x 2 trim CL); cap 300. Trim converged in 236, trim infeasible (delta above 25 deg or not converged) in 8, failed runs 0. Stage-2 outlines chosen by the pre-stated rule: T_AR4.5_S15_L0.3, T_AR6_S15_L0.3, T_AR3.5_S15_L0.3, T_AR6_S15_L0.8, T_AR6_S15_L0.5, B_AR10_H-8_W30_T30, B_AR7_H-8_W13.9_T30, B_AR8.5_H-8_W5_T30, B_AR8.5_H+0_W30_T30, B_AR8.5_H-8_W30_T18.5.

**Candidates passing all five parts: 0 of 234 eligible configurations.**

### Baselines (same pipeline, same frame)

| configuration | twist tolerance R=30 m (deg) | x B45 | spiral | osc zeta | Cnb | Clb | SM % MAC | trim alpha | trim CL | gamma | mass g | parts passed |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| B45_as_flown | 0.0346 | 1.00 | doubles in 10.7 s | +0.061 | +0.0118 | -0.0402 | 5.07 | 6.99 | 0.382 | -6.61 | 17.8 | 1/5 |
| SwiftA_as_flown | 0.0466 | 1.35 | stable | +0.039 | +0.0032 | -0.0403 | 9.79 | 4.37 | 0.296 | -6.61 | 9.8 | 3/5 |
| Zanonia_400 | 0.0276 | 0.80 | stable | +0.081 | +0.0046 | -0.0437 | 0.98 | 5.88 | 0.391 | -6.61 | 5.1 | 2/5 |

Rule 3 threshold = 3 x 0.0346 = 0.1038 deg of wing-half incidence difference for R = 30 m. (`straight_smooth_v1` gives 0.033 deg at R = 31.7 m for B45, i.e. 0.035 at 30 m: the reproduction is within 2 %, see Tests.)

### Parts passed by group

| group | n | 1 spiral | 2 oscillation | 3 twist | 4 pitch | 5 bed | >= 4 parts | candidates |
|---|---|---|---|---|---|---|---|---|
| Stage 1 T | 27 | 8 | 2 | 0 | 0 | 27 | 0 | 0 |
| Stage 1 B | 27 | 26 | 0 | 1 | 15 | 27 | 0 | 0 |
| Stage 2 T | 90 | 79 | 0 | 61 | 4 | 90 | 1 | 0 |
| Stage 2 B | 90 | 80 | 0 | 42 | 87 | 90 | 39 | 0 |
| references | 3 | 2 | 0 | 0 | 1 | 3 | 0 | 0 |

### Top 3

| rank | label | configuration | twist deg (R30) | x B45 | x Swift A | cancel ratio | spiral | zeta | Cnb | Clb | SM % | alpha deg | trim CL | surface deg up | gamma | mass g | wing loading N/m2 | bed | parts |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | NEAR-MISS | B_AR8.5_H-8_W30_T18.5__d20__c2__cl0.25 | 9.120 | 263.6 | 195.8 | 0.006 | stable | +0.007 | +0.0085 | -0.1576 | 12.0 | 3.60 | 0.250 | 5.8 | -7.57 | 18.4 | 10.3 | yes | 4/5 |
| 2 | NEAR-MISS | B_AR8.5_H-8_W5_T30__d20__c2__cl0.35 | 5.095 | 147.2 | 109.4 | 0.013 | stable | +0.014 | +0.0102 | -0.1683 | 12.0 | 4.86 | 0.350 | 7.1 | -5.77 | 21.9 | 12.2 | yes | 4/5 |
| 3 | NEAR-MISS | B_AR8.5_H+0_W30_T30__d20__c2__cl0.35 | 1.159 | 33.5 | 24.9 | 0.055 | stable | +0.012 | +0.0112 | -0.1683 | 12.0 | 4.93 | 0.350 | 7.0 | -5.75 | 22.1 | 12.3 | yes | 4/5 |

- **#1 B_AR8.5_H-8_W30_T18.5__d20__c2__cl0.25 (NEAR-MISS)**: spiral passes (stable); oscillation FAILS (zeta +0.007, Cnb +0.0085); twist tolerance passes (9.120 deg = 263.6 x B45, needs 3); pitch passes (SM 12.0 %, alpha 3.6 deg, trim feasible); bed fits. Versus B45 as flown (twist 0.035 deg, spiral doubles in 10.7 s, zeta +0.061) and Swift A as flown (twist 0.047 deg, stable, zeta +0.039). Rawdon SS 6.92; trim tail bend 5.8 deg; mass 18.4 g (4.1 g of it lead ballast).
- **#2 B_AR8.5_H-8_W5_T30__d20__c2__cl0.35 (NEAR-MISS)**: spiral passes (stable); oscillation FAILS (zeta +0.014, Cnb +0.0102); twist tolerance passes (5.095 deg = 147.2 x B45, needs 3); pitch passes (SM 12.0 %, alpha 4.9 deg, trim feasible); bed fits. Versus B45 as flown (twist 0.035 deg, spiral doubles in 10.7 s, zeta +0.061) and Swift A as flown (twist 0.047 deg, stable, zeta +0.039). Rawdon SS 6.87; trim tail bend 7.1 deg; mass 21.9 g (5.7 g of it lead ballast).
- **#3 B_AR8.5_H+0_W30_T30__d20__c2__cl0.35 (NEAR-MISS)**: spiral passes (stable); oscillation FAILS (zeta +0.012, Cnb +0.0112); twist tolerance passes (1.159 deg = 33.5 x B45, needs 3); pitch passes (SM 12.0 %, alpha 4.9 deg, trim feasible); bed fits. Versus B45 as flown (twist 0.035 deg, spiral doubles in 10.7 s, zeta +0.061) and Swift A as flown (twist 0.047 deg, stable, zeta +0.039). Rawdon SS 6.93; trim tail bend 7.0 deg; mass 22.1 g (5.9 g of it lead ballast).

sheet_builder configs and kit runs: `sheet_builder_top3/top1.json` .. `top3.json`, kits in `sheet_builder_top3/kits/` (new subfolder; nothing existing was overwritten). Results: top1_B_AR8.5_H-8_W30_T18.5: exit 0 (kit all-up 18.01 g with 3.03 g lead; the sweep's mass model said 18.4 g with 4.1 g lead); top2_B_AR8.5_H-8_W5_T30: exit 0 (kit all-up 22.90 g with 5.72 g lead; the sweep's mass model said 21.9 g with 5.7 g lead); top3_B_AR8.5_H+0_W30_T30: exit 0 (kit all-up 23.31 g with 6.11 g lead; the sweep's mass model said 22.1 g with 5.9 g lead). The kit cannot print the outer-panel dihedral, the canted fins or the T-family reflex flap: see the `_comment` in each JSON.

### Closest configurations (parts passed, then twist tolerance)

| configuration | parts | failed parts | twist deg | x B45 | spiral | zeta | Cnb | alpha | SM % |
|---|---|---|---|---|---|---|---|---|---|
| B_AR8.5_H-8_W30_T18.5__d20__c2__cl0.25 | 4/5 | 2 | 9.120 | 263.6 | stable | +0.007 | +0.0085 | 3.60 | 12.0 |
| B_AR8.5_H-8_W5_T30__d20__c2__cl0.35 | 4/5 | 2 | 5.095 | 147.2 | stable | +0.014 | +0.0102 | 4.86 | 12.0 |
| B_AR8.5_H+0_W30_T30__d20__c2__cl0.35 | 4/5 | 2 | 1.159 | 33.5 | stable | +0.012 | +0.0112 | 4.93 | 12.0 |
| T_AR6_S15_L0.3__d10__tip20__cl0.25 | 4/5 | 2 | 0.837 | 24.2 | stable | +0.003 | +0.0057 | 5.00 | 12.0 |
| B_AR10_H-8_W30_T30__d20__c2__cl0.35 | 4/5 | 2 | 0.826 | 23.9 | stable | +0.004 | +0.0077 | 4.61 | 12.0 |
| B_AR10_H-8_W30_T30__d20__c2__cl0.25 | 4/5 | 2 | 0.509 | 14.7 | stable | +0.014 | +0.0095 | 3.30 | 12.0 |
| B_AR7_H-8_W13.9_T30__d20__c2__cl0.35 | 4/5 | 2 | 0.494 | 14.3 | stable | +0.027 | +0.0127 | 4.97 | 12.0 |
| B_AR8.5_H-8_W5_T30__d20__c2__cl0.25 | 4/5 | 2 | 0.330 | 9.5 | stable | +0.022 | +0.0117 | 3.47 | 12.0 |
| B_AR8.5_H-8_W30_T18.5__d10__c2__cl0.35 | 4/5 | 2 | 0.290 | 8.4 | stable | +0.016 | +0.0059 | 4.96 | 12.0 |
| B_AR8.5_H+0_W30_T30__d20__c2__cl0.25 | 4/5 | 2 | 0.285 | 8.2 | stable | +0.021 | +0.0127 | 3.52 | 12.0 |
| B_AR7_H-8_W13.9_T30__d20__c2__cl0.25 | 4/5 | 2 | 0.238 | 6.9 | stable | +0.032 | +0.0140 | 3.56 | 12.0 |
| B_AR10_H-8_W30_T30__d20__c4__cl0.35 | 4/5 | 2 | 0.191 | 5.5 | stable | +0.023 | +0.0144 | 4.61 | 12.0 |

### POST-HOC reading (NOT part of the pre-stated rule; chosen after seeing the results)

**The pre-stated top 3 rest on a cancellation.** Their twist tolerances are large because the two terms of the r_hat numerator (Cnb dCl and Clb dCn for the wing-half incidence) almost cancel: cancellation ratio |Cnb dCl - Clb dCn| / (|Cnb dCl| + |Clb dCn|) = 0.006, 0.013, 0.055 against 0.59 for B45 and 0.62 for Swift A as flown. A tolerance of 9 deg is not credible as a build tolerance; it says that, at that trim and dihedral, the model's roll and yaw responses to a twist happen to offset each other. Read those three rows as 'passes the pre-stated rule parts 1, 3, 4, 5 by cancellation', not as the best shapes.

**What stops every configuration is part 2 (oscillation damping >= 0.10).** 0 of 226 feasible configurations reach zeta 0.10 with Cnb > 0 (none); B45 and Swift A as flown have +0.061 and +0.039. 36 configurations pass parts 1, 3, 4, 5 with Cnb > 0 and fail only on damping; 26 of them are not cancellation-dominated (ratio >= 0.25). The 0.10 threshold is borrowed from manned-aircraft practice (the robustness report: no small-glider criterion exists) and the damping depends on the ASSUMED point-mass inertias, so this is the least certain of the five parts.

| configuration | zeta | twist deg (R30) | x B45 | cancel ratio | Cnb | alpha deg | spiral | SM % | mass g | Rawdon SS |
|---|---|---|---|---|---|---|---|---|---|---|
| B_AR7_H-8_W13.9_T30__d20__c4__cl0.25 | +0.051 | 0.121 | 3.5 | 0.61 | +0.0245 | 3.56 | stable | 12.0 | 29.4 | 12.0 |
| B_AR7_H-8_W13.9_T30__d20__c4__cl0.35 | +0.048 | 0.128 | 3.7 | 0.50 | +0.0233 | 4.97 | stable | 12.0 | 29.4 | 8.6 |
| B_AR8.5_H-8_W5_T30__d20__c4__cl0.25 | +0.039 | 0.122 | 3.5 | 0.47 | +0.0196 | 3.47 | stable | 12.0 | 22.6 | 9.1 |
| B_AR8.5_H+0_W30_T30__d20__c4__cl0.25 | +0.039 | 0.116 | 3.3 | 0.48 | +0.0207 | 3.52 | stable | 12.0 | 22.9 | 9.2 |
| B_AR8.5_H-8_W5_T30__d20__c4__cl0.35 | +0.033 | 0.150 | 4.3 | 0.35 | +0.0182 | 4.86 | stable | 12.0 | 22.6 | 6.5 |
| B_AR7_H-8_W13.9_T30__d20__c2__cl0.25 | +0.032 | 0.238 | 6.9 | 0.31 | +0.0140 | 3.56 | stable | 12.0 | 28.4 | 12.6 |
| B_AR10_H-8_W30_T30__d20__c4__cl0.25 | +0.031 | 0.132 | 3.8 | 0.41 | +0.0162 | 3.29 | stable | 12.0 | 19.5 | 7.6 |
| B_AR8.5_H+0_W30_T30__d20__c4__cl0.35 | +0.030 | 0.137 | 4.0 | 0.37 | +0.0193 | 4.93 | stable | 12.0 | 22.8 | 6.5 |
| B_AR8.5_H-8_W5_T30__d10__c2__cl0.35 | +0.026 | 0.113 | 3.3 | 0.39 | +0.0087 | 4.79 | stable | 12.0 | 21.8 | 3.4 |
| B_AR8.5_H-8_W30_T18.5__d20__c4__cl0.25 | +0.026 | 0.172 | 5.0 | 0.29 | +0.0138 | 3.60 | stable | 12.0 | 19.0 | 6.4 |

Count of configurations passing parts 1, 3, 4, 5 with Cnb > 0, not cancellation-dominated, and a relaxed damping threshold: zeta >= 0.1: 0; zeta >= 0.07: 0; zeta >= 0.05: 1; zeta >= 0.04: 2; zeta >= 0.03: 8.

**The post-hoc pick is a trade, not a candidate: B_AR7_H-8_W13.9_T30__d20__c4__cl0.25** (highest zeta among the 26 configurations that pass parts 1, 3, 4, 5, have Cnb > 0 and a cancellation ratio >= 0.25; that cut is POST-HOC). Outline B_AR7_H-8_W13.9_T30 (ar_target 7, hand_shift -8, wrist_eta 0.139, tail_area_frac 0.3, tail_len 0.204691), tip dihedral 20 deg on the outer 40 %, centre fin 4% S, trim CL 0.25, SM 12 % MAC, tail bend 5.2 deg.
- What it gains: wing-half twist tolerance 0.121 deg (3.5 x B45, 2.6 x Swift A); spiral stable; Cnb +0.0245 (B45 +0.0118, Swift A +0.0032); trim alpha 3.56 deg.
- What it costs: fin-toe tolerance TW30(c) 0.249 deg against 0.646 deg for B45 (worse); oscillation damping zeta +0.051 below B45 +0.061 (above Swift A +0.039), and it fails part 2; trim speed 9.4 m/s against 4.6 (B45 foam) and 5.5 (Swift A foam), wing loading 13.6 N/m2 (2.8 x B45, 2.5 x Swift A), kinetic energy m V^2 7 x B45 and 9 x Swift A (consequences for pull-out height, impact and propeller strike); mass 29.4 g with 10.2 g lead, set by the SM 12 % target and the kit's ballast station (a lighter layout was not swept); mode shape |beta|/|phi| 0.536, close to the 0.5 threshold used for calling the oscillation a Dutch roll.
- Glide-angle sensitivity: at gamma -11 deg (instead of -7.6) the spiral eigenvalue is -0.2040 1/s (part 1 passes) and TW30(b) is 0.158 deg.
- Status: it is not a candidate (part 2 fails); whether to build it is the owner's decision. A sheet_builder config and flat kit are in `sheet_builder_top3/posthoc.json` (an addition to the three required, labelled post-hoc) (kit all-up 31.67 g with 11.00 g lead; the sweep's mass model said 29.4 g with 10.2 g lead).
### Post-hoc checks: glide angle and the Clb band (POST-HOC, not part of the pre-stated rule)

**Glide angle.** Each configuration's own gamma comes from CD0 = 0.0307 (Swift A about -6.6 deg); `swoop_model_v1` gives about -11 deg. Modes were rerun in AVL at theta0 = alpha + gamma (Ixz included) at both angles, and the closed-form twist tolerance TW30 (deg for R = 30 m) was recomputed for asymmetries a (differential trim surface), b (wing-half incidence), c (fin toe). `spiral own` is the rerun at the own gamma and reproduces the stored value (a check of the rerun). Part 1 is the 30 s spiral rule. Full numbers: `gamma11_check.csv`.

| configuration | role | gamma own | spiral own 1/s | part 1 own | spiral at -11 1/s | part 1 at -11 | zeta own | zeta at -11 | TW30 a/b/c own | TW30 a/b/c at -11 |
|---|---|---|---|---|---|---|---|---|---|---|
| B45_as_flown | reference | -6.61 | +0.0648 | FAIL | -0.0482 | pass | +0.061 | +0.049 | 0.319 / 0.035 / 0.646 | 0.235 / 0.026 / 0.477 |
| SwiftA_as_flown | reference | -6.61 | -0.0518 | pass | -0.1556 | pass | +0.039 | +0.027 | 1.505 / 0.047 / - | 4.500 / 0.139 / - |
| Zanonia_400 | reference | -6.61 | -0.0354 | pass | -0.2248 | pass | +0.081 | +0.059 | - / 0.028 / - | - / 0.173 / - |
| B_AR8.5_H-8_W30_T18.5__d20__c2__cl0.25 | NEAR-MISS | -7.57 | -0.1497 | pass | -0.1937 | pass | +0.007 | +0.005 | 2.033 / 9.120 / 0.370 | 2.620 / 11.755 / 0.477 |
| B_AR8.5_H-8_W5_T30__d20__c2__cl0.35 | NEAR-MISS | -5.77 | -0.1492 | pass | -0.2216 | pass | +0.014 | +0.009 | 2.570 / 5.095 / 0.345 | 3.798 / 7.530 / 0.509 |
| B_AR8.5_H+0_W30_T30__d20__c2__cl0.35 | NEAR-MISS | -5.75 | -0.1412 | pass | -0.2155 | pass | +0.012 | +0.007 | 2.496 / 1.159 / 0.341 | 3.790 / 1.760 / 0.518 |
| B_AR7_H-8_W13.9_T30__d20__c4__cl0.25 | POST-HOC PICK | -7.64 | -0.1564 | pass | -0.2040 | pass | +0.051 | +0.049 | 1.883 / 0.121 / 0.249 | 2.451 / 0.158 / 0.325 |


**Clb band.** AVL's Clb is uncertain on thin low-aspect-ratio plates, so TW30(b) was recomputed with Clb scaled by 0.7 and 1.3 (closed form only, no AVL rerun; own gamma); the ratio is against B45 as flown at the same factor. The 'robust' set uses a cancellation-ratio cut of 0.25, which is POST-HOC (chosen after seeing the results). Full table: `clb_band.csv`.

| configuration | role | cancel ratio | TW30(b) Clb x0.7 | x B45 | TW30(b) Clb x1.0 | x B45 | TW30(b) Clb x1.3 | x B45 | keeps 3x at all factors |
|---|---|---|---|---|---|---|---|---|---|
| B45_as_flown | reference | 0.587 | 0.049 | 1.0 | 0.035 | 1.0 | 0.017 | 1.0 | no |
| SwiftA_as_flown | reference | 0.623 | 0.019 | 0.4 | 0.047 | 1.4 | 0.079 | 4.6 | no |
| Zanonia_400 | reference | 0.287 | 0.013 | 0.3 | 0.028 | 0.8 | 0.117 | 6.9 | no |
| B_AR8.5_H-8_W30_T18.5__d20__c2__cl0.25 | NEAR-MISS | 0.006 | 0.217 | 4.5 | 9.120 | 263.6 | 0.538 | 31.5 | yes |
| B_AR8.5_H-8_W5_T30__d20__c2__cl0.35 | NEAR-MISS | 0.013 | 0.215 | 4.4 | 5.095 | 147.2 | 0.703 | 41.1 | yes |
| B_AR8.5_H+0_W30_T30__d20__c2__cl0.35 | NEAR-MISS | 0.055 | 0.164 | 3.4 | 1.159 | 33.5 | 1.085 | 63.4 | yes |
| B_AR7_H-8_W13.9_T30__d20__c4__cl0.25 | POST-HOC PICK | 0.614 | 0.064 | 1.3 | 0.121 | 3.5 | 0.190 | 11.1 | no |


Of the 26 'robust' configurations, 1 keep at least 3 x the B45 twist tolerance (same Clb factor) at all three factors; at Clb x0.7: 1, at x1.3: 26. Range of the ratio over the set: x0.7 0.8-3.3, x1.3 5.8-29.4.

### Stage 1: what the outlines do (flat, SM 12 %, CL 0.30)

- Family T (trapezoid, tip fins) (20 of 27 with a feasible trim; infeasible: T_AR3.5_S35_L0.8, T_AR4.5_S35_L0.5, T_AR4.5_S35_L0.8, T_AR6_S25_L0.8, T_AR6_S35_L0.3, T_AR6_S35_L0.5, T_AR6_S35_L0.8; ranges below are over the feasible ones): trim alpha 5.99-8.04 deg (0 of 20 at or below 5 deg), reflex / bend 11.9-23.1 deg, twist tolerance 0.004-0.022 deg (0.1-0.7 x B45), spiral stable in 0 of 20, oscillation zeta +0.027..+0.088, Cnb > 0 in 20, mass 20.6-53.9 g. Best twist tolerance: T_AR3.5_S15_L0.3 (0.022), T_AR4.5_S15_L0.3 (0.020), T_AR6_S15_L0.8 (0.020). Lowest alpha: T_AR6_S15_L0.3 (5.99), T_AR6_S15_L0.5 (6.25), T_AR4.5_S15_L0.3 (6.27).
- Family B (bird lineage) (26 of 27 with a feasible trim; infeasible: B_AR10_H+8_W13.9_T10; ranges below are over the feasible ones): trim alpha 3.88-6.50 deg (15 of 26 at or below 5 deg), reflex / bend 5.4-21.4 deg, twist tolerance 0.022-0.052 deg (0.6-1.5 x B45), spiral stable in 26 of 26, oscillation zeta +0.014..+0.043, Cnb > 0 in 26, mass 12.1-27.0 g. Best twist tolerance: B_AR7_H-8_W13.9_T30 (0.052), B_AR10_H-8_W30_T30 (0.049), B_AR8.5_H-8_W5_T30 (0.048). Lowest alpha: B_AR10_H-8_W30_T30 (3.88), B_AR8.5_H-8_W5_T30 (4.09), B_AR8.5_H+0_W30_T30 (4.16).

### Stage 2: feature effects (matched pairs, median with range)

| family | change | twist tolerance | delta zeta | delta spiral eig 1/s | delta Cnb | delta Clb | delta trim alpha deg |
|---|---|---|---|---|---|---|---|
| T | tip dihedral 0 -> 10 deg | x22.95 (x0.84..x387.92, n=30) | -0.064 (-0.103..-0.047, n=30) | -0.1668 (-0.3217..-0.0898, n=30) | -0.0016 (-0.0027..-0.0009, n=30) | -0.0614 (-0.0680..-0.0428, n=30) | +0.02 (+0.01..+0.06, n=30) |
| T | tip dihedral 0 -> 20 deg | x9.93 (x1.00..x235.59, n=30) | -0.097 (-0.145..-0.062, n=30) | -0.2862 (-0.4920..-0.1602, n=30) | -0.0002 (-0.0021..+0.0012, n=30) | -0.1171 (-0.1297..-0.0821, n=30) | +0.11 (+0.07..+0.20, n=30) |
| T | fin: no fin -> tip fins cant 0 | x2.67 (x0.02..x37.77, n=30) | +0.016 (-0.009..+0.039, n=30) | +0.1357 (+0.0439..+0.2026, n=30) | +0.0057 (+0.0043..+0.0069, n=30) | -0.0032 (-0.0062..-0.0004, n=30) | -0.00 (-0.02..+0.05, n=30) |
| T | fin: tip fins cant 0 -> tip fins cant 20 | x0.86 (x0.12..x21.18, n=30) | -0.015 (-0.026..-0.011, n=30) | -0.0296 (-0.0470..-0.0196, n=30) | -0.0009 (-0.0012..-0.0006, n=30) | -0.0113 (-0.0127..-0.0070, n=30) | -0.01 (-0.02..-0.01, n=30) |
| T | fin: no fin -> tip fins cant 20 | x2.25 (x0.17..x59.69, n=30) | +0.002 (-0.020..+0.026, n=30) | +0.1078 (+0.0034..+0.1639, n=30) | +0.0049 (+0.0036..+0.0057, n=30) | -0.0133 (-0.0179..-0.0081, n=30) | -0.01 (-0.03..+0.03, n=30) |
| T | trim CL 0.25 -> 0.35 (SM held) | x1.08 (x0.01..x19.98, n=45) | -0.003 (-0.018..+0.037, n=45) | -0.0073 (-0.0787..+0.0833, n=45) | +0.0006 (-0.0008..+0.0018, n=45) | -0.0072 (-0.0082..-0.0053, n=45) | +2.12 (+1.99..+2.32, n=45) |
| B | tip dihedral 0 -> 10 deg | x5.42 (x1.10..x14.34, n=30) | -0.032 (-0.063..-0.018, n=30) | -0.0884 (-0.1655..-0.0533, n=30) | -0.0024 (-0.0036..-0.0015, n=30) | -0.0712 (-0.0751..-0.0664, n=30) | +0.01 (+0.00..+0.01, n=30) |
| B | tip dihedral 0 -> 20 deg | x8.63 (x1.20..x610.31, n=30) | -0.040 (-0.079..-0.024, n=30) | -0.1460 (-0.2393..-0.0950, n=30) | -0.0005 (-0.0026..+0.0018, n=30) | -0.1356 (-0.1430..-0.1268, n=30) | +0.07 (+0.06..+0.09, n=30) |
| B | fin: no fin -> centre fin 2% S | x1.21 (x0.18..x87.23, n=30) | +0.036 (+0.023..+0.048, n=30) | +0.0471 (+0.0134..+0.0916, n=30) | +0.0089 (+0.0063..+0.0119, n=30) | +0.0006 (+0.0004..+0.0010, n=30) | +0.00 (-0.00..+0.01, n=30) |
| B | fin: centre fin 2% S -> centre fin 4% S | x0.50 (x0.02..x1.28, n=30) | +0.019 (+0.012..+0.021, n=30) | +0.0131 (+0.0032..+0.0412, n=30) | +0.0080 (+0.0054..+0.0109, n=30) | -0.0002 (-0.0006..+0.0003, n=30) | +0.00 (-0.00..+0.01, n=30) |
| B | fin: no fin -> centre fin 4% S | x0.79 (x0.13..x3.47, n=30) | +0.055 (+0.035..+0.069, n=30) | +0.0591 (+0.0194..+0.1286, n=30) | +0.0169 (+0.0117..+0.0228, n=30) | +0.0004 (-0.0002..+0.0013, n=30) | +0.01 (+0.00..+0.01, n=30) |
| B | trim CL 0.25 -> 0.35 (SM held) | x1.08 (x0.04..x15.42, n=45) | +0.001 (-0.011..+0.020, n=45) | +0.0223 (-0.0484..+0.0798, n=45) | -0.0001 (-0.0018..+0.0019, n=45) | -0.0089 (-0.0118..-0.0082, n=45) | +1.39 (+1.29..+1.43, n=45) |

Twist-tolerance ratios have long upper tails: where the r_hat numerator nearly cancels the tolerance explodes (see the cancellation ratio in configs.csv), so read the median, not the range; the zeta, Cnb and Clb differences are not affected. Dihedral raises -Clb (more dihedral effect) by about 0.06-0.07 per 10 deg of outer-panel dihedral (medians in the table) while Cnb falls: that is the trade the rule parts 2 and 3 pull on.

### What fails and why

Eligible configurations failing each part: part 1: 41; part 2: 232; part 3: 130; part 4: 128; part 5: 0. Failing exactly one part (the part): {1: 0, 2: 40, 3: 0, 4: 0, 5: 0}. Trim alpha above 5 deg: 128 of 234; Cnb <= 0: 8; spiral unstable beyond a 30 s doubling time: 41; oscillation zeta below 0.10: 232.

Reading the failures: (i) family T is limited by its reflex device: a flap of 15 % chord over 0.15..0.40 of the semi-span needs 11.9-84.8 deg to trim CL 0.30 at SM 12 % (7 of 27 infeasible beyond 25 deg, none at or below 5 deg alpha in Stage 1); a larger reflex or a tip washout was not swept, so 'T fails' means 'T with this flap'. (ii) Holding SM at 12 % with a reflex trim costs alpha: configurations above 5 deg are nearly all T and the CL 0.35 variants; CL 0.25 is what brings alpha down. (iii) The printed-T masses come from the kit's default ballast station right behind the plate front, which needs a lot of lead; with the B45-style nose extension and station -37.5 mm the lead would be a fraction of that (a development check gave 25 g all-up instead of 34.7 g for T_AR4.5_S15_L0.3), so T inertias and speeds here are conservative. (iv) Dihedral buys twist tolerance and spiral stability but costs oscillation damping and Cnb (tables above); a fin buys Cnb and damping back.

### Rawdon SS against the AVL twist tolerance

Spearman rank correlation between SS and TW30, Stage 2 only: +0.60 (n=180). Over all converged configurations it is +0.75 (n=233), but that figure includes 112 ties, mostly the flat Stage-1 outlines with SS = 0 (EDA 0), so it flatters the agreement. A high rank correlation would say the textbook spiral number and the AVL turn sensitivity order the configurations alike; it is not a check on either.

### Rank, do not predict

AVL's Clb for thin, low-aspect-ratio plates at Re of 10-30k is unreliable (measured roll due to sideslip departs from vortex-lattice predictions because tip-vortex roll-up is destabilising; Linehan & Mohseni, cited in the robustness report) and CLa is uncertain by about +-30 %. Twist tolerances, spiral eigenvalues and damping ratios here therefore order configurations; they are not predicted radii or tolerances, and a configuration that 'passes' has passed a model rule, not a flight. The reflex flap and tail-bend effectiveness at low Re is below AVL's, the CG height and inertias are point-mass assumptions, and the 1 deg asymmetry is a unit of comparison, not a measured build error. Measured calibration (a B45 with a gauged wing-half twist, the circle radius timed on repeated drops) is the missing data point.

### Files

`configs.csv`, `sweep.gif`, `contact_sheet.svg`, `index.html`, `sheet_builder_top3/`, `runs.json`, `prestated_section.sha256`. Code: `rigid_glider/shape_sweep_v1.py`; tests `rigid_glider/test_shape_sweep_v1.py`. AVL scratch files: outside this folder (`--work`).

### Tests (2026-10-21 18:35, py -B rigid_glider/test_shape_sweep_v1.py)

Ran 28 tests in 69 s: OK (0 failures, 0 skipped). Covered: B45 and Swift A as-flown references reproduce `straight_smooth_v1` (Cnb, Clb, spiral eigenvalue, oscillation damping, trim alpha within 2 %, trim speed within 1 %); all 54 outlines and the three references are closed, non-self-intersecting and mirror-symmetric with a 400 mm span; Swift A is generated exactly and the 27 B outlines form a balanced fractional factorial; every outline fits the 250 x 250 bed by the sheet_builder fit check; SM and trim targeting converge for a T and a B case; dihedral and fin sign checks; the five rule parts, the Stage-2 selection rule, the top-3 rule, Spearman, Pareto and the Newton step on synthetic data; the GIF writer round-trips through the stdlib decoder and the written `sweep.gif` decodes to 147 frames of 800 x 420 at 60 cs on the dark background, with red-border frames carrying the cancellation marker; the glide-angle rerun at each configuration's own gamma reproduces the stored spiral eigenvalue (within 2 %); the closed-form twist tolerance reproduces the stored values; the cancellation marker is present in `configs.csv`, `contact_sheet.svg`, `index.html` and the sheet_builder JSON headers and kits README; generated outputs contain no personal names; `test_prestated_section_unchanged`: the sha256 of the README bytes before the line RESULTS equals the value in `prestated_section.sha256`.

Files changed outside the owned paths by this work: none.

