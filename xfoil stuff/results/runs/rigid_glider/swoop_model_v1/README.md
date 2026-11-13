# Swoop model v1: can a stall-limited pitch model reproduce the persistent swoops? (MODEL study, nothing here is a flight result)

## Question, method, pre-stated check (written 2026-10-21 17:25, BEFORE any run of swoop_model_v1.py)

**Question.** Kai's 400 mm gliders (B45 foamboard flat wing, 10 deg-up centre flaps, CG 66.0 mm aft of root LE; Swift A foam 400, CG 73.0 mm behind the nose tip, 6 deg tail bend) still swoop after about 10 cycles, with a small stall at each crest. The earlier quasi-steady drop sim (rigid_glider/swiftA_oscillation_study.py) predicts a decaying phugoid and failed 2 of 3 of its checks. Literature (research/lateral_robustness, Okamoto & Jinba 2009 at Re 1.2-1.5e4, Li et al. 2022, Uhlig & Selig 2013) suggests a lift-slope break near 8 deg, CP moving aft past the break, L/D only about 6, and a stall-limited "bounding" limit cycle. Does a pitch model with those ingredients reproduce the swoops, and how far forward the CG, how much extra reflex / tail bend, or how much more pitch damping removes them?

**Method (all MODEL).** 3-DOF longitudinal rigid body (x, z, theta, vx, vz, q) plus one separation state f (0 attached, 1 separated), fixed-step RK4 dt 0.004 s, still air unless stated, drop from rest nose-down (theta -90 deg) as in swiftA_oscillation_study, run length 80 s (no ground). Whole-vehicle lift/drag/moment:
- Lift: attached slope s1 (deg^-1) up to a break alpha_b, reduced slope s2 after it, smooth blend (half-width 1.5 deg), CL at 35 deg anchored to the measured CLmax 0.85 (so s2 = (0.85 - s1*alpha_b)/(35 - alpha_b), floored at 0.04*s1), flat-plate formula (CN90 1.3) blended in beyond 35 deg. alpha is the effective angle alpha - tau*device, tau = device lift loss / s1 (as the old sim), unless the "break on geometric alpha" variant.
- Base s1 = AVL whole-vehicle CLa (this is the planform / tail "3D scaling"), compared with Okamoto & Jinba AR 4-8 slopes 3.3-5.0 /rad.
- Drag: CD = CD0 + k CL^2 below the break, blending to CD0 + CL tan(alpha) (normal-force limit) as f rises; CD0 = CL*/(2 L/D), k = 1/(2 L/D CL*), CL* = s1 * 4.5 deg (Okamoto: alpha at best L/D 4.0-5.4 deg), base L/D max 6.
- Aerodynamic centre = AVL neutral point; past the break the centre of pressure moves aft by dx_max * f chords (dx_max from the Ananda Re-60k table in the project: xcp at the end of the table minus its measured AC), more toward the broadside plate value beyond 35 deg.
- Device (bend / flap) moment, pitch damping Cmq, masses and inertia, as the old sim (AVL, kit mass model).
- Optional first-order lag of f with time constant n convective times (n c / V), Goman-Khrabrov style.

**Variants (11 per glider).** base; slope -30 %; slope +30 %; break 6 deg; break 10 deg; L/D max 5; L/D max 7.5; no CP shift; lag n = 2; lag n = 5; break on geometric alpha.

**Definitions.** Pull-out: first time after 0.05 s, speed > 0.3 m/s, that the flight path gamma is shallower than -20 deg. Crest k: k-th local maximum of gamma after pull-out (0.5 deg hysteresis). Cycle k: crest k to crest k+1 (only complete cycles count). Cycle amplitude: half of (gamma at crest k minus the minimum gamma in the cycle). A(10-12): mean amplitude of the existing cycles among 10, 11, 12 (0 if cycle 10 does not exist). Stall at crest k: the break variable (effective angle, or geometric angle in that variant) exceeds alpha_b at some time within +-0.5 s of the crest. Stall count: number of crests among 3..12 that stall (maximum 10).

**Pre-stated agreement check, per glider (Swift A at 73.0 mm / 6 deg bend; B45 at 66.0 mm / flaps 10 deg), per variant.**
- (a) Pulls out and glides: after the first level-off gamma never returns steeper than -80 deg, the mean gamma over the last 5 s is shallower than -45 deg, and theta never spans 360 deg or more (no tumble).
- (b) The oscillation is still present after 10 cycles: A(10-12) >= 2 deg AND stall count >= 8.
- (c) Moving the CG 3 mm forward (same bend / flap) reduces A(10-12) to <= 50 % of the as-flown value (fails if the as-flown A(10-12) is 0).
Pass / fail is reported per part per variant. Failing is a finding about the model, not a reason to change the check. Note for the reader: part (c) is the literature hypothesis; Kai reported for the Swift that moving the CG slightly forward or aft "did not help".

**Map classes** (CG x device grid; Swift CG 66-76 mm behind the nose in 1 mm steps x bend 3-10 deg in 1 deg steps; B45 CG 56-70 mm aft of root LE in 1 mm steps x flap 5-15 deg in 1 deg steps): tumble (theta spans 360 deg or more); dive (not (a), not tumble); bounding limit cycle ((a), A(10-12) >= 2 deg, stall count >= 8); persistent, weak stall ((a), A(10-12) >= 2 deg, stall count < 8); smooth ((a), A(10-12) < 2 deg, cycle-3 amplitude < 25 % of cycle-1 amplitude, no stalled crest among cycles 2-12); decaying phugoid (everything else with (a) and A(10-12) < 2 deg). Trim alpha from the static moment balance.

**Further runs (all variants, as-flown CG and device):** pitch-damping (Cmq) multiplier 1, 1.5, 2; wing loading (mass and inertia, same geometry) 1, 1.5, 2; vertical gust (Ornstein-Uhlenbeck, 0.3 m/s rms, 1.0 s correlation time (ASSUMED), seeds 1-5) against still air, at the as-flown and the recommended setting. A response is called gust-driven if still-air A(10-12) < 2 deg while the gusty rms deviation of gamma in 25-45 s is >= 2 deg; limit cycle if still-air (b) holds. The tail-area part of "tail volume" is a net static-margin change and is read from the CG maps; only the damping part is multiplied.

**Recommendation rule (fixed before looking at maps).** Per glider, among grid cells that are never "dive" or "tumble" in more than 1 of 11 variants, choose the one with the most variants classed smooth or decaying phugoid (ties: more smooth variants, then smaller distance from the as-flown cell in grid steps). Also report the top 5 and the cells in the as-flown neighbourhood. "Model suggests, flight decides."

**What may not change after the run:** this section. Parameters read from existing outputs are listed with their source in the results part below.
<!-- END FROZEN -->

## Results (written 2026-10-21 18:05 by swoop_model_v1.py; the section above is byte-identical to the frozen one, hash checked)

Tags: MODEL (this model / AVL), ASSUMED (my choice, no source), MEASURED-literature (cited data), KAI (Kai's report). Nothing here is a flight result.

### Model inputs and where each comes from

| quantity | value / rule | tag and source |
|---|---|---|
| lift slope s1 (base) | AVL whole-vehicle CLa per glider (3D planform scaling, tail included); carried +-30 % as variants | MODEL; corroborated by MEASURED-literature (Okamoto & Jinba AR 4-8 slopes 3.3-5.0 /rad) |
| break alpha_b | 8 deg (variants 6, 10); smooth over +-1.5 deg | MEASURED-literature (bends near 8 deg) / ASSUMED width |
| post-break slope s2 | set so that CL = 0.85 at 35 deg, floor 4 % of s1; plate formula (CN90 1.3) beyond | MEASURED-literature (CLmax 0.76-0.96 at 33-40 deg; CDmax 1.26-1.52) |
| drag polar | CD = CD0 + k CL^2, CD0 = CL*/(2 L/D), CL* = s1 x 4.5 deg, L/D max 6 (5, 7.5); CL tan(alpha) past the break | MEASURED-literature ((L/D)max 5.5-6.5 at 4.0-5.4 deg) / ASSUMED blend |
| CP shift past the break | 0.134 chord (xcp at the end of the Ananda table 0.351 minus its AC 0.217), x separation fraction; none as a variant | MEASURED-literature at Re 6e4 (qualitative agreement with Okamoto & Jinba: CP moves aft past the bend) |
| lift lag | first order on the separation fraction, 0 / 2 / 5 convective times (c/V) | ASSUMED (Goman-Khrabrov style) |
| gust | Ornstein-Uhlenbeck vertical, 0.3 m/s rms, 1.0 s correlation time, seeds 1-5 | rms KAI/brief, correlation time ASSUMED |

**Swift A foam 400** (CG unit: mm behind the nose tip)
- CLa: AVL whole-vehicle CLa 5.010/rad (5 deg bend run, avl/swA400_b5.log) [MODEL / ASSUMED as stated]
- NP: AVL Xnp 41.70 mm from root LE (30.0% MAC) [MODEL / ASSUMED as stated]
- Cmq: AVL Cmq -3.346 (constant, as the old sim) [MODEL / ASSUMED as stated]
- device: AVL per deg bend TE up: dCL -0.01392, dCm(at NP) 0.00501 [MODEL / ASSUMED as stated]
- mass: kit mass model, ballast re-solved at the fixed hole for each CG (as the old sim); 9.76 g at 73 mm [MODEL / ASSUMED as stated]

**B45 flat wing, centre flaps** (CG unit: mm aft of the root LE)
- CLa: AVL fit CLa 0.06552/deg (3.754/rad, alpha 0/4/8, flap 10, nominal location; results.json) [MODEL / ASSUMED as stated]
- NP: AVL Xnp 70.30 mm aft of root LE [MODEL / ASSUMED as stated]
- Cmq: AVL Cmq of the B45 tab decks at CG 57/70/78 mm (-1.68/-0.98/-0.70), linear in CG (as the old Vehicle) [MODEL / ASSUMED as stated]
- device: AVL flap 10 deg up, nominal location: dCL -0.00766/deg, dCm(at NP) 0.00187/deg (flap effect taken linear in the angle) [MODEL / ASSUMED as stated]
- mass: kit parts (plate, fins, stick) + ballast at the -37.5 mm station re-solved for each CG; 17.87 g at 66 mm (ASSUMED kit masses) [MODEL / ASSUMED as stated]

The Okamoto & Jinba numbers are the transcription in research/lateral_robustness/research_notes/.../low_re_pitch.md Table 1; the PDF (sources/akita_okamoto/akita.pdf) could not be rendered here (no PDF tool), so no figure was read from it.

### Pre-stated check, per variant

**Swift A foam 400** at 73 / 6

| variant | (a) | (b) | (c) | class as flown | A(10-12) deg | stall crests of 3-12 | crests found in 80 s | A(10-12) CG -3 mm |
|---|---|---|---|---|---|---|---|---|
| base | PASS | FAIL | FAIL | smooth | 0.00 | 0 | 5 | 0.00 |
| slope -30% | PASS | FAIL | FAIL | smooth | 0.00 | 0 | 6 | 0.00 |
| slope +30% | PASS | FAIL | FAIL | smooth | 0.00 | 0 | 4 | 0.00 |
| break 6 deg | PASS | FAIL | FAIL | smooth | 0.00 | 0 | 5 | 0.00 |
| break 10 deg | PASS | FAIL | FAIL | smooth | 0.00 | 0 | 5 | 0.00 |
| L/D max 5 | PASS | FAIL | FAIL | smooth | 0.00 | 0 | 4 | 0.00 |
| L/D max 7.5 | PASS | FAIL | FAIL | smooth | 0.00 | 0 | 7 | 0.00 |
| no CP shift | PASS | FAIL | FAIL | smooth | 0.00 | 0 | 5 | 0.00 |
| lag 2 conv. times | PASS | FAIL | FAIL | smooth | 0.00 | 0 | 5 | 0.00 |
| lag 5 conv. times | PASS | FAIL | FAIL | smooth | 0.00 | 0 | 5 | 0.00 |
| break on geometric alpha | PASS | FAIL | FAIL | smooth | 0.00 | 0 | 5 | 0.00 |

- Variants passing both (a) and (b): 0 of 11. (a) passes in 11, (b) in 0, (c) in 0 of 11.

**B45 flat wing, centre flaps** at 66 / 10

| variant | (a) | (b) | (c) | class as flown | A(10-12) deg | stall crests of 3-12 | crests found in 80 s | A(10-12) CG -3 mm |
|---|---|---|---|---|---|---|---|---|
| base | PASS | FAIL | PASS | decaying phugoid | 0.56 | 0 | 12 | 0.00 |
| slope -30% | PASS | FAIL | FAIL | smooth | 0.00 | 0 | 5 | 0.00 |
| slope +30% | PASS | FAIL | PASS | decaying phugoid | 0.54 | 0 | 11 | 0.00 |
| break 6 deg | PASS | FAIL | FAIL | smooth | 0.00 | 0 | 6 | 0.00 |
| break 10 deg | PASS | FAIL | PASS | decaying phugoid | 0.73 | 0 | 13 | 0.00 |
| L/D max 5 | PASS | FAIL | FAIL | smooth | 0.00 | 0 | 9 | 0.00 |
| L/D max 7.5 | PASS | FAIL | PASS | decaying phugoid | 1.59 | 0 | 17 | 0.00 |
| no CP shift | PASS | FAIL | PASS | persistent, weak stall | 3.26 | 5 | 17 | 0.00 |
| lag 2 conv. times | PASS | FAIL | PASS | decaying phugoid | 0.67 | 0 | 12 | 0.00 |
| lag 5 conv. times | PASS | FAIL | PASS | decaying phugoid | 0.88 | 0 | 14 | 0.00 |
| break on geometric alpha | PASS | FAIL | FAIL | decaying phugoid | 0.00 | 0 | 8 | 0.00 |

- Variants passing both (a) and (b): 0 of 11. (a) passes in 11, (b) in 0, (c) in 7 of 11.

Reading (c): it is the literature hypothesis (CG forward removes bounding). For the Swift, Kai reported that moving the CG slightly forward or aft did not help (KAI), so a (c) PASS would not have matched that flight observation.

### What the models do over the whole grid (cells x 11 variants)

**Swift A foam 400**: smooth 806 (83 %); decaying phugoid 131 (14 %); bounding limit cycle 5 (1 %); dive 26 (3 %) of 968 runs.

- Variants with any persistent oscillation (limit cycle or weak stall) anywhere in the grid: no CP shift: B 5, w 0

**B45 flat wing, centre flaps**: smooth 1283 (71 %); decaying phugoid 404 (22 %); persistent, weak stall 31 (2 %); bounding limit cycle 32 (2 %); dive 65 (4 %) of 1815 runs.

- Variants with any persistent oscillation (limit cycle or weak stall) anywhere in the grid: slope +30%: B 3, w 5; break 10 deg: B 4, w 5; L/D max 7.5: B 0, w 6; no CP shift: B 18, w 7; lag 2 conv. times: B 1, w 3; lag 5 conv. times: B 6, w 5

### Where a persistent oscillation appears, and what removes it

**Swift A foam 400**

| variant | persistent cells | CG range (mm behind the nose tip) | tail bend range | CG shift forward to calm, mm (same device) | tail bend decrease to calm, deg (same CG) | tail bend increase to calm, deg (same CG) |
|---|---|---|---|---|---|---|
| no CP shift | 5 (5 bounding) | 75-76 | 5-10 | median 1 / max 1 | median 2 | median 2 (3 not reached) |

Class mix against the trim margin to the break (break variable at trim, all cells, all variants) for Swift A foam 400:

| margin to break, deg | runs | persistent (B or w) | of which bounding | dive / tumble |
|---|---|---|---|---|
| < 0 | 7 | 3 (43 %) | 3 | 0 |
| 0 to 1 | 85 | 2 (2 %) | 2 | 0 |
| 1 to 2 | 71 | 0 (0 %) | 0 | 0 |
| 2 to 3 | 62 | 0 (0 %) | 0 | 0 |
| 3 to 4 | 102 | 0 (0 %) | 0 | 0 |
| >= 4 | 641 | 0 (0 %) | 0 | 26 |

**B45 flat wing, centre flaps**

| variant | persistent cells | CG range (mm aft of the root LE) | flap up range | CG shift forward to calm, mm (same device) | flap up decrease to calm, deg (same CG) | flap up increase to calm, deg (same CG) |
|---|---|---|---|---|---|---|
| slope +30% | 8 (3 bounding) | 67-69 | 5-12 | median 1 / max 2 | median 1 (5 not reached) | median 5 |
| break 10 deg | 9 (4 bounding) | 67-69 | 5-10 | median 1 / max 2 | median 2 (5 not reached) | median 2 |
| L/D max 7.5 | 6 (0 bounding) | 67-68 | 5-9 | median 1 / max 2 | median 1 (5 not reached) | median 3 |
| no CP shift | 25 (18 bounding) | 65-70 | 5-15 | median 2 / max 6 | median 4 (8 not reached) | n/a |
| lag 2 conv. times | 4 (1 bounding) | 69-69 | 5-8 | median 1 / max 1 | n/a | median 3 |
| lag 5 conv. times | 11 (6 bounding) | 67-70 | 5-9 | median 2 / max 4 | median 2 (6 not reached) | median 2 |

Class mix against the trim margin to the break (break variable at trim, all cells, all variants) for B45 flat wing, centre flaps:

| margin to break, deg | runs | persistent (B or w) | of which bounding | dive / tumble |
|---|---|---|---|---|
| < 0 | 40 | 12 (30 %) | 12 | 25 |
| 0 to 1 | 349 | 18 (5 %) | 14 | 33 |
| 1 to 2 | 195 | 26 (13 %) | 6 | 2 |
| 2 to 3 | 116 | 6 (5 %) | 0 | 0 |
| 3 to 4 | 192 | 1 (1 %) | 0 | 0 |
| >= 4 | 923 | 0 (0 %) | 0 | 5 |

Note on (c): a PASS only needs the 3 mm-forward amplitude to be at most half the as-flown one, so it is vacuous when the as-flown amplitude is already small. Passes / passes with as-flown A(10-12) >= 2 deg: Swift A foam 400 0 / 0; B45 flat wing, centre flaps 7 / 1.

### Nose-dive edge at the flown CG (Kai: 'bending the tail less gave a nose dive' on the Swift)

- Swift A foam 400 at CG 73: tail bend 3: 0/11 variants dive; tail bend 4: 0/11 variants dive; tail bend 5: 0/11 variants dive; tail bend 6: 0/11 variants dive; tail bend 7: 0/11 variants dive; tail bend 8: 0/11 variants dive; tail bend 9: 0/11 variants dive; tail bend 10: 0/11 variants dive
- B45 flat wing, centre flaps at CG 66: flap up 5: 0/11 variants dive; flap up 6: 0/11 variants dive; flap up 7: 0/11 variants dive; flap up 8: 0/11 variants dive; flap up 9: 0/11 variants dive; flap up 10: 0/11 variants dive; flap up 11: 0/11 variants dive; flap up 12: 0/11 variants dive; flap up 13: 0/11 variants dive; flap up 14: 0/11 variants dive; flap up 15: 0/11 variants dive

### As flown versus the model's suggestion (base variant, still air)

| glider | setting | CG / device | class | trim alpha deg | margin to break deg | static margin % MAC | amp cycle 1 deg | amp cycle 3 deg | A(10-12) deg | period s | pull-out loss m |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Swift A foam 400 | as flown | 73 / 6 | smooth | 4.4 | 4.6 | 9.9 | 26.2 | 3.9 | 0.00 | 3.34 | 7.4 |
| Swift A foam 400 | model suggests | 73 / 6 | smooth | 4.4 | 4.6 | 9.9 | 26.2 | 3.9 | 0.00 | 3.34 | 7.4 |
| B45 flat wing, centre flaps | as flown | 66 / 10 | decaying phugoid | 7.2 | 2.0 | 4.7 | 39.5 | 12.7 | 0.56 | 2.82 | 6.6 |
| B45 flat wing, centre flaps | model suggests | 64 / 8 | smooth | 4.2 | 4.7 | 6.8 | 26.8 | 4.4 | 0.00 | 3.93 | 10.3 |

### Output of the frozen recommendation rule, framed as TEST PROPOSALS (model suggests, flight decides; Kai decides)

The model did not reproduce either glider's swoops or the Swift's nose dive, so nothing below is a validated prediction. Margins to the break are a ranking, not a number.

**Swift A foam 400**

| rank | CG (mm behind the nose tip) | tail bend (deg) | calm variants /11 | smooth variants | dive/tumble variants |
|---|---|---|---|---|---|
| 1 | 73 | 6 | 11 | 11 | 0 |
| 2 | 72 | 6 | 11 | 11 | 0 |
| 3 | 73 | 5 | 11 | 11 | 0 |
| 4 | 73 | 7 | 11 | 11 | 0 |
| 5 | 72 | 5 | 11 | 11 | 0 |

- As-flown cell: calm in 11/11 variants, smooth in 11, dive/tumble in 0.

**B45 flat wing, centre flaps**

| rank | CG (mm aft of the root LE) | flap up (deg) | calm variants /11 | smooth variants | dive/tumble variants |
|---|---|---|---|---|---|
| 1 | 64 | 8 | 11 | 11 | 0 |
| 2 | 63 | 10 | 11 | 11 | 0 |
| 3 | 63 | 9 | 11 | 11 | 0 |
| 4 | 63 | 8 | 11 | 11 | 0 |
| 5 | 64 | 7 | 11 | 11 | 0 |

- As-flown cell: calm in 10/11 variants, smooth in 3, dive/tumble in 0.

- **Swift A:** the frozen rule returns the as-flown setting. Read this as 'the model cannot see the Swift's swoops' (it predicts a smooth glide where Kai saw heavy oscillations and stalls), not as 'the Swift is fine'. No Swift setting change is proposed from this model.
- **B45 test proposal: 'trim faster to widen the margin to the break'.** Examples 63 / 10 (pull-out loss 6.6 -> 9.2 m, trim speed 4.48 -> 5.91 m/s, about +32 %) and 64 / 8 (pull-out loss 6.6 -> 10.3 m, static margin 4.7 -> 6.8 % MAC, trim speed 4.48 -> 6.11 m/s). In the post-hoc pitch-up CP case (dx -0.05) 63/10 stays calm while 64/8 still bounds, so moving the CG forward with the flap unchanged is the more robust direction. Because the model's dive edge is not trusted (it does not reproduce Kai's Swift dive, so it cannot be trusted for B45 either), step in small increments, CG 65 mm first (flap 10), then judge from the flight. Kai decides.

### Pitch damping, wing loading, gusts

Wording: higher wing loading at the same geometry damped the residual oscillation in this model (higher speed, lower CL). A printed glider differs in section, thickness, Re and mass distribution, so this does not transfer to the printed versions.

**Swift A foam 400, as flown (73 / 6)**

| setting | base class | A(10-12) base deg | variants calm /11 | variants persistent /11 | mean A(10-12) deg |
|---|---|---|---|---|---|
| x1 | smooth | 0.00 | 11 | 0 | 0.00 |
| Cmq x1.5 | smooth | 0.00 | 11 | 0 | 0.00 |
| Cmq x2 | smooth | 0.00 | 11 | 0 | 0.00 |
| wing loading x1.5 | smooth | 0.00 | 11 | 0 | 0.00 |
| wing loading x2 | smooth | 0.00 | 11 | 0 | 0.00 |

| variant | still A(10-12) deg | still gamma rms 25-45 s deg | gusty gamma rms deg (5 seeds) | gusty max break variable after 20 s deg | seeds past the break | verdict |
|---|---|---|---|---|---|---|
| base | 0.00 | 0.01 | 3.15 | 5.3 | 0/5 | gust-driven |
| slope -30% | 0.00 | 0.03 | 3.26 | 7.1 | 0/5 | gust-driven |
| slope +30% | 0.00 | 0.00 | 3.04 | 4.3 | 0/5 | gust-driven |
| break 6 deg | 0.00 | 0.01 | 3.17 | 5.3 | 0/5 | gust-driven |
| break 10 deg | 0.00 | 0.01 | 3.15 | 5.3 | 0/5 | gust-driven |
| L/D max 5 | 0.00 | 0.00 | 3.03 | 5.3 | 0/5 | gust-driven |
| L/D max 7.5 | 0.00 | 0.05 | 3.34 | 5.4 | 0/5 | gust-driven |
| no CP shift | 0.00 | 0.01 | 3.15 | 5.3 | 0/5 | gust-driven |
| lag 2 conv. times | 0.00 | 0.01 | 3.15 | 5.3 | 0/5 | gust-driven |
| lag 5 conv. times | 0.00 | 0.01 | 3.15 | 5.3 | 0/5 | gust-driven |
| break on geometric alpha | 0.00 | 0.01 | 3.15 | 6.3 | 0/5 | gust-driven |

**Swift A foam 400, model suggestion (73 / 6)**

| setting | base class | A(10-12) base deg | variants calm /11 | variants persistent /11 | mean A(10-12) deg |
|---|---|---|---|---|---|
| x1 | smooth | 0.00 | 11 | 0 | 0.00 |
| Cmq x1.5 | smooth | 0.00 | 11 | 0 | 0.00 |
| Cmq x2 | smooth | 0.00 | 11 | 0 | 0.00 |
| wing loading x1.5 | smooth | 0.00 | 11 | 0 | 0.00 |
| wing loading x2 | smooth | 0.00 | 11 | 0 | 0.00 |

| variant | still A(10-12) deg | still gamma rms 25-45 s deg | gusty gamma rms deg (5 seeds) | gusty max break variable after 20 s deg | seeds past the break | verdict |
|---|---|---|---|---|---|---|
| base | 0.00 | 0.01 | 3.15 | 5.3 | 0/5 | gust-driven |
| slope -30% | 0.00 | 0.03 | 3.26 | 7.1 | 0/5 | gust-driven |
| slope +30% | 0.00 | 0.00 | 3.04 | 4.3 | 0/5 | gust-driven |
| break 6 deg | 0.00 | 0.01 | 3.17 | 5.3 | 0/5 | gust-driven |
| break 10 deg | 0.00 | 0.01 | 3.15 | 5.3 | 0/5 | gust-driven |
| L/D max 5 | 0.00 | 0.00 | 3.03 | 5.3 | 0/5 | gust-driven |
| L/D max 7.5 | 0.00 | 0.05 | 3.34 | 5.4 | 0/5 | gust-driven |
| no CP shift | 0.00 | 0.01 | 3.15 | 5.3 | 0/5 | gust-driven |
| lag 2 conv. times | 0.00 | 0.01 | 3.15 | 5.3 | 0/5 | gust-driven |
| lag 5 conv. times | 0.00 | 0.01 | 3.15 | 5.3 | 0/5 | gust-driven |
| break on geometric alpha | 0.00 | 0.01 | 3.15 | 6.3 | 0/5 | gust-driven |

**B45 flat wing, centre flaps, as flown (66 / 10)**

| setting | base class | A(10-12) base deg | variants calm /11 | variants persistent /11 | mean A(10-12) deg |
|---|---|---|---|---|---|
| x1 | decaying phugoid | 0.56 | 10 | 1 | 0.75 |
| Cmq x1.5 | decaying phugoid | 0.00 | 11 | 0 | 0.05 |
| Cmq x2 | smooth | 0.00 | 11 | 0 | 0.00 |
| wing loading x1.5 | decaying phugoid | 0.00 | 11 | 0 | 0.22 |
| wing loading x2 | decaying phugoid | 0.00 | 11 | 0 | 0.05 |

| variant | still A(10-12) deg | still gamma rms 25-45 s deg | gusty gamma rms deg (5 seeds) | gusty max break variable after 20 s deg | seeds past the break | verdict |
|---|---|---|---|---|---|---|
| base | 0.56 | 0.39 | 4.30 | 8.5 | 5/5 | gust-driven |
| slope -30% | 0.00 | 0.00 | 3.67 | 9.2 | 5/5 | gust-driven |
| slope +30% | 0.54 | 0.39 | 4.30 | 7.6 | 1/5 | gust-driven |
| break 6 deg | 0.00 | 0.00 | 3.78 | 7.0 | 5/5 | gust-driven |
| break 10 deg | 0.73 | 0.69 | 4.49 | 9.7 | 1/5 | gust-driven |
| L/D max 5 | 0.00 | 0.15 | 4.01 | 8.4 | 5/5 | gust-driven |
| L/D max 7.5 | 1.59 | 1.16 | 4.81 | 8.5 | 5/5 | gust-driven |
| no CP shift | 3.26 | 0.98 | 4.62 | 10.7 | 5/5 | neither (calm) |
| lag 2 conv. times | 0.67 | 0.46 | 4.59 | 8.8 | 5/5 | gust-driven |
| lag 5 conv. times | 0.88 | 0.69 | 5.00 | 9.0 | 5/5 | gust-driven |
| break on geometric alpha | 0.00 | 0.02 | 3.94 | 8.6 | 5/5 | gust-driven |

**B45 flat wing, centre flaps, model suggestion (64 / 8)**

| setting | base class | A(10-12) base deg | variants calm /11 | variants persistent /11 | mean A(10-12) deg |
|---|---|---|---|---|---|
| x1 | smooth | 0.00 | 11 | 0 | 0.00 |
| Cmq x1.5 | smooth | 0.00 | 11 | 0 | 0.00 |
| Cmq x2 | smooth | 0.00 | 11 | 0 | 0.00 |
| wing loading x1.5 | smooth | 0.00 | 11 | 0 | 0.00 |
| wing loading x2 | smooth | 0.00 | 11 | 0 | 0.00 |

| variant | still A(10-12) deg | still gamma rms 25-45 s deg | gusty gamma rms deg (5 seeds) | gusty max break variable after 20 s deg | seeds past the break | verdict |
|---|---|---|---|---|---|---|
| base | 0.00 | 0.05 | 2.57 | 5.1 | 0/5 | gust-driven |
| slope -30% | 0.00 | 0.12 | 2.64 | 6.9 | 0/5 | gust-driven |
| slope +30% | 0.00 | 0.01 | 2.51 | 4.2 | 0/5 | gust-driven |
| break 6 deg | 0.00 | 0.04 | 2.58 | 5.1 | 0/5 | gust-driven |
| break 10 deg | 0.00 | 0.05 | 2.57 | 5.1 | 0/5 | gust-driven |
| L/D max 5 | 0.00 | 0.01 | 2.51 | 5.1 | 0/5 | gust-driven |
| L/D max 7.5 | 0.00 | 0.18 | 2.68 | 5.2 | 0/5 | gust-driven |
| no CP shift | 0.00 | 0.05 | 2.57 | 5.1 | 0/5 | gust-driven |
| lag 2 conv. times | 0.00 | 0.05 | 2.57 | 5.1 | 0/5 | gust-driven |
| lag 5 conv. times | 0.00 | 0.05 | 2.57 | 5.1 | 0/5 | gust-driven |
| break on geometric alpha | 0.00 | 0.05 | 2.57 | 6.1 | 0/5 | gust-driven |

### Post-hoc additions (added after the reviewer's REVISE; NOT part of the frozen plan; base variant only)

**1. CP-shift sweep.** The CP-shift size (0.134 chord) is an input taken from the Re-60k Ananda table. It is about 3 x the B45 static margin (4.7 % MAC) and its sign for a swept, tapered, tailless wing (which may tip-stall and pitch up) is not known. Negative values below mean the CP moves forward past the break (pitch-up).

| cell | CP shift dx_max, chords (+ aft) | class | A(10-12) deg | stalled crests / crests 3-12 | crests found | trim alpha deg | margin to break deg | static margin % MAC |
|---|---|---|---|---|---|---|---|---|
| Swift A as flown 73/6 | -0.05 | smooth | 0.00 | 0/3 | 5 | 4.4 | 4.6 | 9.9 |
| Swift A as flown 73/6 | 0 | smooth | 0.00 | 0/3 | 5 | 4.4 | 4.6 | 9.9 |
| Swift A as flown 73/6 | 0.03 | smooth | 0.00 | 0/3 | 5 | 4.4 | 4.6 | 9.9 |
| Swift A as flown 73/6 | 0.07 | smooth | 0.00 | 0/3 | 5 | 4.4 | 4.6 | 9.9 |
| Swift A as flown 73/6 | 0.134 | smooth | 0.00 | 0/3 | 5 | 4.4 | 4.6 | 9.9 |
| B45 as flown 66/10 | -0.05 | bounding limit cycle | 35.00 | 10/10 | 28 | 7.2 | 2.0 | 4.7 |
| B45 as flown 66/10 | 0 | persistent, weak stall | 3.26 | 5/10 | 17 | 7.2 | 2.0 | 4.7 |
| B45 as flown 66/10 | 0.03 | decaying phugoid | 0.70 | 1/10 | 13 | 7.2 | 2.0 | 4.7 |
| B45 as flown 66/10 | 0.07 | decaying phugoid | 0.64 | 0/10 | 12 | 7.2 | 2.0 | 4.7 |
| B45 as flown 66/10 | 0.134 | decaying phugoid | 0.56 | 0/10 | 12 | 7.2 | 2.0 | 4.7 |
| B45 64/8 | -0.05 | bounding limit cycle | 12.50 | 10/10 | 16 | 4.2 | 4.7 | 6.8 |
| B45 64/8 | 0 | smooth | 0.00 | 0/4 | 6 | 4.2 | 4.7 | 6.8 |
| B45 64/8 | 0.03 | smooth | 0.00 | 0/4 | 6 | 4.2 | 4.7 | 6.8 |
| B45 64/8 | 0.07 | smooth | 0.00 | 0/4 | 6 | 4.2 | 4.7 | 6.8 |
| B45 64/8 | 0.134 | smooth | 0.00 | 0/4 | 6 | 4.2 | 4.7 | 6.8 |
| B45 63/10 | -0.05 | decaying phugoid | 0.00 | 0/4 | 6 | 4.7 | 4.4 | 7.9 |
| B45 63/10 | 0 | smooth | 0.00 | 0/4 | 6 | 4.7 | 4.4 | 7.9 |
| B45 63/10 | 0.03 | smooth | 0.00 | 0/4 | 6 | 4.7 | 4.4 | 7.9 |
| B45 63/10 | 0.07 | smooth | 0.00 | 0/4 | 6 | 4.7 | 4.4 | 7.9 |
| B45 63/10 | 0.134 | smooth | 0.00 | 0/4 | 6 | 4.7 | 4.4 | 7.9 |

Reading: a statement that the aft CP shift removes a limit cycle is input-dependent, not a finding. The class changes with dx in the table above; the 0.134 rows are the base model.

**2. Gust sweep** (B45, base variant, seeds 1-5 per row). In gusty runs the cycle metrics are computed on a noisy flight path; stalls count as crest-locked only if the excursions past the break sit at crests.

| cell | gust rms, correlation time | seeds past the break | A(10-12) deg (mean of 5) | mean fraction of crests 3-12 that stall | mean max break variable after 20 s deg | excursions past the break per run (after 10 s) | of these within 0.5 s of a crest |
|---|---|---|---|---|---|---|---|
| B45 as flown 66/10 | still air | - | 0.56 | 0/10 | 6.4 | - | - |
| B45 as flown 66/10 | 0.15 m/s, 0.3 s | 4/5 | 1.88 | 26 % | 8.2 | 3.6 | 94 % |
| B45 as flown 66/10 | 0.15 m/s, 1 s | 0/5 | 1.84 | 4 % | 7.5 | 0.0 | - |
| B45 as flown 66/10 | 0.15 m/s, 3 s | 0/5 | 1.05 | 0 % | 7.0 | 0.0 | - |
| B45 as flown 66/10 | 0.3 m/s, 0.3 s | 5/5 | 2.18 | 62 % | 10.2 | 58.2 | 99 % |
| B45 as flown 66/10 | 0.3 m/s, 1 s | 5/5 | 1.20 | 32 % | 8.5 | 8.2 | 95 % |
| B45 as flown 66/10 | 0.3 m/s, 3 s | 0/5 | 1.63 | 12 % | 7.6 | 0.2 | 100 % |
| B45 as flown 66/10 | 0.5 m/s, 0.3 s | 5/5 | 3.90 | 74 % | 13.5 | 122.8 | 100 % |
| B45 as flown 66/10 | 0.5 m/s, 1 s | 5/5 | 2.30 | 60 % | 10.1 | 53.8 | 97 % |
| B45 as flown 66/10 | 0.5 m/s, 3 s | 5/5 | 1.39 | 32 % | 8.5 | 6.8 | 88 % |
| B45 64/8 | still air | - | 0.00 | 0/4 | 3.3 | - | - |
| B45 64/8 | 0.15 m/s, 0.3 s | 0/5 | 1.09 | 0 % | 4.9 | 0.0 | - |
| B45 64/8 | 0.15 m/s, 1 s | 0/5 | 0.80 | 0 % | 4.2 | 0.0 | - |
| B45 64/8 | 0.15 m/s, 3 s | 0/5 | 0.58 | 0 % | 3.8 | 0.0 | - |
| B45 64/8 | 0.3 m/s, 0.3 s | 0/5 | 2.29 | 6 % | 6.5 | 0.0 | - |
| B45 64/8 | 0.3 m/s, 1 s | 0/5 | 1.17 | 0 % | 5.1 | 0.0 | - |
| B45 64/8 | 0.3 m/s, 3 s | 0/5 | 0.65 | 0 % | 4.4 | 0.0 | - |
| B45 64/8 | 0.5 m/s, 0.3 s | 5/5 | 1.44 | 32 % | 8.7 | 3.2 | 100 % |
| B45 64/8 | 0.5 m/s, 1 s | 0/5 | 1.91 | 10 % | 6.5 | 0.0 | - |
| B45 64/8 | 0.5 m/s, 3 s | 0/5 | 1.66 | 0 % | 5.1 | 0.0 | - |

Crest-lock check: over all gusty runs 1284 excursions past the break after 10 s, 1267 of them within 0.5 s of a crest (99 %).

A high crest-lock percentage is only weak evidence: gusty flight paths contain many crests (0.5 deg hysteresis), so an excursion is almost always within 0.5 s of one. The gust explanation of the B45 swoops stays a HYPOTHESIS. The model shows gust-driven excursions past the break at the as-flown cell; whether they are locked to the swoop crests (what Kai saw) is read from the last two columns, and the 1-D time-correlated gust model and the 0.15-0.5 m/s levels are assumptions.

**3. Stepping cells for B45** (from the frozen-plan map runs; margins are a ranking, not a number):

| CG / flap | calm variants | base class | trim alpha deg | margin to break, effective alpha, deg | margin on geometric alpha deg | static margin % MAC | trim speed m/s | trim CL | pull-out loss m |
|---|---|---|---|---|---|---|---|---|---|
| 66 / 10 (as flown) | 10/11 | decaying phugoid | 7.2 | 2.0 | 1.2 | 4.7 | 4.48 | 0.40 | 6.6 |
| 65 / 10 | 11/11 | decaying phugoid | 6.1 | 3.1 | 1.9 | 5.8 | 4.99 | 0.32 | 7.4 |
| 65 / 9 | 11/11 | decaying phugoid | 5.5 | 3.6 | 2.5 | 5.8 | 5.26 | 0.29 | 8.2 |
| 65 / 8 | 11/11 | smooth | 4.9 | 4.1 | 3.1 | 5.8 | 5.58 | 0.26 | 9.1 |
| 64 / 10 | 11/11 | smooth | 5.3 | 3.9 | 2.7 | 6.8 | 5.47 | 0.27 | 8.3 |
| 64 / 8 | 11/11 | smooth | 4.2 | 4.7 | 3.8 | 6.8 | 6.11 | 0.22 | 10.3 |
| 63 / 10 | 11/11 | smooth | 4.7 | 4.4 | 3.3 | 7.9 | 5.91 | 0.23 | 9.2 |

### Caveats and disclosures

- **Glide angle is unmeasured and the models disagree.** The old Ananda-table sim gives about -6.6 deg; this model gives -10.8 deg (Swift) and -11.0 deg (B45), mostly because the drag polar has L/D 6 against about 8 in the old table (about 1 deg comes from the angle-shift treatment of the device inherited from the old sim). Which is right is unknown. Trim CL against the best-L/D CL (CL* = slope x 4.5 deg): Swift trim 0.30 against 0.39 (below best L/D); B45 trim 0.40 against 0.29 (ABOVE best L/D, so slower than best glide). Next flight day: measure glide angle (drop height, flight time, distance covered).
- **Margins to the break are a ranking, not a number.** The break (8 deg) comes from 0.5 mm plates; the foamboard is 5 mm (t/c about 11 %) with a blunt edge. Taking the break on geometric instead of effective alpha moves the B45 as-flown margin from 2.0 to 1.2 deg (0.8 deg at the base-model trim alpha of 7.2 deg; the trim itself shifts a little).
- **Dive edge not trusted.** The model fails to reproduce Kai's Swift nose dive at reduced bend (as the old sim did): it never dives at 73 mm for bends 3-10 deg. Its dive edge is therefore not a basis for any setting change, B45 included.
- **Okamoto & Jinba source check.** Table 1 of research/lateral_robustness/sources/akita_okamoto/akita.pdf was decoded with a stdlib parse of the PDF text streams (digit glyph ids inferred as consecutive glyphs (no ToUnicode map), validated by the exact match of every table row; no PDF renderer) and compared with the transcription in low_re_pitch.md: all AR>=4 rows used here (CLmax, alpha at CLmax, L/D max, alpha at L/D max, slope) match exactly. VERIFIED (Table 1 numbers only; digit mapping inferred from one offset and consistent across all rows; not renderer-checked). The statement that the lift curve bends near 8 deg is in the Japanese/figure text, which this route cannot read: the 8 deg break and the CP-moves-aft statement stay UNVERIFIED transcription wherever margin numbers appear.
- **Linear-test disclosure.** The linear (break removed) phugoid period is compared with the old sim's 3.220 s: with the old sim's own lift slope and L/D it is 3.376 s (4.8 %); with this model's base numbers it is 3.382 s (5.0 %, marginally outside). The assertion in the test was set to the old-matched case after the base-number comparison came out at 5.0 %; both numbers are printed by the test.
- Quasi-steady forces except the optional lag. 2 and 5 convective times are only 35-85 ms, short against the 3 s swoop, so the lag variants are close to the base model; no measured hysteresis loop at Re 1-3e4 exists in the sources.
- Rigid, still air (apart from the gust runs), longitudinal only; Kai's gliders also turn (lateral asymmetry), which is not modelled. No ground, 80 s run; real drops last about 40 s.
- The frozen gust verdict rule uses the inertial flight-path rms, which a 0.3 m/s vertical gust at 5 m/s raises to about 3 deg by air motion alone (0.3 / 5.4 rad), so 'gust-driven' there is not a discriminating label; seeds-past-the-break and crest-lock are the informative numbers.
- The recommended cell is the output of the frozen rule; when the as-flown cell is already calm in all 11 variants the rule returns it, which means the model sees nothing to change, not that the flight is fine.
- CP-shift size and sign are inputs (see post-hoc sweep); the model's persistence results depend on them.

### Next flight day (measurements that would settle what the model cannot)

- Glide angle: drop height, flight time and distance covered, per glider and per setting. Side-view video for swoop period and whether the nose visibly stalls at the crests. Gauged bend / flap angle and CG per flight.

Wall time 60 s. dt 0.004 s RK4, run length 80 s, no ground. Quick mode: False.

### Files

- cases.csv: pre-stated check rows (as-flown cell, all variants, with (a)(b)(c)) and the damping / loading / gust runs. maps.csv: every grid run (all variants). index.html: maps, time histories, tables. swoop_compare.gif: side views. README_frozen.sha256: hash of the frozen section.
