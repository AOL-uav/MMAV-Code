# Adjustable passive-glider test article: verified model and first physical test

Run `py rigid_glider/verify_model.py` and `py rigid_glider/prototype_study.py`
from the project root. Parameters are in [`prototype_config.json`](prototype_config.json).
The numerical [tail map](../results/runs/rigid_glider/prototype/tail_map.csv),
[fin map](../results/runs/rigid_glider/prototype/fin_map.csv),
[tolerance cases](../results/runs/rigid_glider/prototype/tolerances.csv),
[verification](../results/runs/rigid_glider/prototype/verification/verification.json),
plots, and raw AVL files are retained. These are low-order **screening**
results, not evidence of convergent released flight. No CFD, global optimizer,
folding, morphing, or active control was added.

## A. Verified aerodynamic model

The original 5% and 10% fixed-tail cases were rerun from geometry, not merely
read from the prior CSV. At about 10% MAC static margin, both NACA 0008 tails
at −3° incidence reproduce positive-lift trim:

| S_t/S | Tail chord | Tail AC arm from CG | V_H about CG | Trim alpha | Trim CL | Static margin |
|---:|---:|---:|---:|---:|---:|---:|
| 0.05 | 24.5 mm | 246 mm | 0.0707 | 2.53° | 0.1801 | 0.1002 MAC |
| 0.10 | 49.0 mm | 246 mm | 0.1412 | 4.23° | 0.3104 | 0.1005 MAC |

The geometry deck sets `Sref=0.205714 m²`, `Cref=0.1739997 m` (the **main
wing** MAC), and `Bref=1.2 m`; the tail is an additional lifting surface,
not silently included in Sref. Its span is 0.42 m. `Xref` is the selected CG
x-coordinate measured **aft from the root leading edge**; Yref=0 and the
currently assumed Zref=0. The physical CG z must be measured later. Tail
incidence −3° is relative to the 0° main wing and produces the expected
positive zero-alpha pitch-moment offset. The reported tail arm is from CG to
the tail quarter-chord; thus `V_H=(S_t/S)(l_t/c_bar)` and changes slightly
when CG moves.

Important axis distinction: AVL geometry input uses X downstream/aft and Z
up, while its **standard output** uses x forward and z down. We now parse
stability-axis `Cl'`/`Cn'` consistently with `Cl_beta`/`Cn_beta`, rather than
mixing unprimed moments with primed derivatives. `Cm` is about the y reference
axis through CG. Positive static margin is `(xNP−xCG)/MAC` in the geometry
coordinate, and the independent derivative check `−Cm_alpha/CL_alpha` agrees
within about 0.000002 MAC in each tail case. Positive `Cn_beta` is the
restoring/weathercock sign: explicit AVL runs at beta=−1° and +1° give
`Cn'≈−0.00023` and `+0.00023` with a 4% center fin, yielding a finite-
difference derivative of +0.01318/rad versus AVL's +0.01309/rad.

The previously reported **1° total left/right incidence mismatch** was also
rerun with an AVL full-chord-normal control proxy. It gives opposing
`Cl'≈±0.00645`; at the 10%-tail placeholder condition that is approximately
0.041 N·m aerodynamic roll moment. The new 8%-tail reference gives
`|Cl'|≈0.00648` or about 0.044 N·m. These are residual moments, not predicted
roll angles or rates. The [AVL primer](https://web.mit.edu/drela/Public/web/avl/AVL_User_Primer.pdf)
defines the geometry axes, moment normalization, stability axes, and
derivatives.

## B. Manufacturing-first prototype family

Hold the main wing simple during the first comparison: one planar symmetric
NACA 0008 wing, span 1.2 m, AR 7, taper 0.65, quarter-chord sweep 25°,
root/tip chord 208/135 mm, zero washout, zero reflex, and zero dihedral. These
are **test-article reference dimensions**, not revived packaging requirements
or a final MMAV. A simple center-joint dihedral can be added later without
warping either panel. Retain the same printed main wing while changing:

- a measured ballast/CG rail spanning roughly x=0.16–0.195 m from root LE;
- interchangeable straight symmetric horizontal tails, roughly 6–10% of
  main-wing area on a simple straight support, with adjustable quarter-chord
  x near 0.38–0.50 m;
- an optional straight removable center fin, roughly 0–4% of main-wing area,
  with quarter-chord x near 0.40–0.48 m.

The tail support and fin mount should use repeatable datum surfaces and
measurable incidence/yaw alignment. A simple fixed tail is mechanically less
sensitive than several degrees of distributed wing twist, but boom mass,
stiffness, and any future samara interference have **not** been modeled.
Actual mass may differ from the current 8 N/m² placeholder and must be
measured before predicting flight speed for the built article.

## C. Tail-sizing map

The bounded grid varies `S_t/S=0.04–0.12` and tail aerodynamic-center x from
0.38 to 0.56 m. Each case's CG is first placed at about 10% MAC static margin;
the same geometry is then run at ±5 mm CG. This avoids pretending one CG
works for every tail. Arm and `V_H` are computed from the **actual CG**.
Representative nominal cases show the area–arm exchange:

| S_t/S | Tail AC x | Arm from CG | V_H | Trim CL | Speed at 8 N/m² | Tail Re | CL range at ±5 mm CG |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.06 | 0.50 m | 0.326 m | 0.112 | 0.285 | 6.77 m/s | 13k | 0.222–0.399 |
| 0.08 | 0.44 m | 0.266 m | 0.122 | 0.288 | 6.74 m/s | 18k | 0.224–0.403 |
| 0.10 | 0.38 m | 0.209 m | 0.120 | 0.257 | 7.13 m/s | 23k | 0.200–0.360 |

All three maintain roughly 7–13% MAC static margin across those ±5 mm CG
shifts. Thus 10% tail area is **not required by this AVL screen**. Smaller
area plus longer arm can give comparable nominal pitch behavior, but it
lengthens the support and shrinks tail chord/Re: at 6% area the fixed 0.42 m
span tail chord is only 29 mm. A 4% tail can also meet the exploratory CL/SM
bands at longer arm in AVL, yet its chord is only 20 mm and modeled tail Re
falls to about 10–12k; its real viscous effectiveness is especially doubtful.
Conversely, too much area and arm push trim CL toward or beyond the upper
exploratory 0.5 band when CG is aft. The [tail-volume plot](../results/runs/rigid_glider/prototype/tail_cl_vs_volume.svg)
shows the bounded trend. No single optimum is claimed.

## D. Center-fin sizing map

The fin grid holds the moderate 8% tail/x=0.44 m family and its CG fixed,
then varies center-fin area 0–6% of main-wing area and fin aerodynamic-center
x=0.32–0.56 m. Here `V_V=(S_v/S)(l_v/b)` uses the CG-to-fin arm. A few
representative static results:

| S_v/S | Fin arm | V_V | AVL Cn_beta |
|---:|---:|---:|---:|
| 0 | — | 0 | +0.0021/rad |
| 0.02 | 0.306 m | 0.0051 | +0.0066/rad |
| 0.03 | 0.226 m | 0.0057 | +0.0094/rad |
| 0.03 | 0.306 m | 0.0077 | +0.0121/rad |
| 0.04 | 0.226 m | 0.0075 | +0.0148/rad |

An exploratory “clearly positive” band of roughly +0.01/rad starts around
3–4% fin area for these arms. **This is not a flight-dynamics requirement.**
Area, arm, and the changing fin aspect ratio all matter; `V_V` alone does not
fully collapse the data. A 3% center fin with the modeled 87 mm chord is
about 71 mm high. Fin addition changes the printed pitch trim CL negligibly
in AVL. There is no fin viscous-drag or low-Re effectiveness correction. See
the [fin-volume plot](../results/runs/rigid_glider/prototype/fin_cnb_vs_volume.svg).

## E. Exploratory manufacturing tolerances

The common comparison article is **not a final selection**: 8% tail at
x=0.44 m, 3% center fin at x=0.48 m, CG x≈0.1736 m. (The first-build
article in [`BUILD_ARTICLE.md`](BUILD_ARTICLE.md) instead starts with the
10% tail at the same station, chosen for its larger chord and tail Reynolds
number, with the 8% tail as the alternate; the numbers in this section are
for the 8% tail.) The current model gives
trim alpha≈3.93°, CL≈0.288, static margin≈10.0% MAC, and speed≈6.74 m/s
at W/S=8 N/m². The following are configurable *exploratory* errors, not
measured print/assembly distributions:

| Error | Modeled consequence | Priority for physical measurement |
|---|---|---|
| CG ±2 mm | CL 0.258–0.325; margin 8.9–11.2% | high: easy to measure, materially changes glide speed |
| CG ±5 mm | CL 0.224–0.403; margin 7.2–12.9% | high: bracket, not a permitted tolerance |
| Tail incidence ±1° | CL 0.192–0.383; margin near 10% | high: a few degrees matter but trim survives |
| Wing incidence mismatch ±1° | residual `Cl'≈±0.00648`, about 0.044 N·m | **highest unbalanced-moment concern** |
| Wing incidence mismatch ±2° | residual `Cl'≈±0.01296` | high: no wings-level static equilibrium at beta=0 |
| Fin yaw misalignment ±2° | residual `Cn'≈±0.00035`, equivalent to roughly 1.7° beta offset if linear | moderate; may bias path |

The measured probability of each error is unknown, so this ranks modeled
consequences—not defect likelihood. A left/right sweep mismatch of ±2° total
was also run. However, splitting a coplanar wing/tail into separate AVL
surfaces shifts the *zero-error* modeled trim significantly; the small
incremental change cannot be used as a reliable tolerance ranking. Treat
sweep/alignment as a required build measurement and physical-test variable,
not as “safe” because that VLM delta was small. The incidence test uses a
full-chord normal-deflection proxy, not an exact warped printed wing. The
fin-yaw test similarly rotates its normal, not a solid fin mesh.

## F. Physical inputs for dynamics, without invented inertias

Before AVL eigenmodes or a disturbed-release trajectory mean anything,
measure total mass; assembled CG x/y/z; Ixx, Iyy, Izz about that CG; and Ixz
if the geometry/mass distribution makes it non-negligible. Ixy/Iyz should be
small for a symmetric build, but inspect them in CAD rather than silently
zeroing measured asymmetry. Also bound release speed, pitch/roll/yaw angles,
initial p/q/r rates, and available height. Low-Re pitch moments, profile/
parasitic drag, and stall behavior remain aerodynamic uncertainties.

A practical inertia procedure: weigh every printed part, fastener, boom,
ballast, and eventual camera marker; give each component its measured mass
and location in CAD; compute its own inertia tensor and sum about the measured
assembled CG with the parallel-axis theorem. Recompute for every ballast/CG
position. Check total CAD mass and CG against a scale and two-axis balance or
suspension measurement. If modal predictions prove sensitive to an inertia,
cross-check that axis with a simple bifilar/torsional pendulum before relying
on the simulation. Use one explicit body-axis convention when transferring
CAD quantities into AVL's `.mass` input. The existing
[`dynamics_inputs.example.json`](dynamics_inputs.example.json) is the intake
template; no 6-DOF recovery claim is made yet.

## G. First drop-test matrix

**Superseded as the opening plan (2026-10-10).** The first releases are now
the 18 in [`BUILD_ARTICLE.md`](BUILD_ARTICLE.md) section 7, which start from
the 10% tail, not the 8% reference used below. This matrix and
`drop_test_matrix.csv` are retained unchanged as the fuller follow-on
campaign; `BUILD_ARTICLE.md` leaves the 8% tail, 4% fin and other tail
stations until after those 18. Within the tail branch, where the two
disagree on the starting configuration or CG, `BUILD_ARTICLE.md` is the
later document. As of 2026-10-11 the tail branch as a whole is a fallback,
not the selected aircraft (see the root `README.md`).

The complete [94-release matrix](drop_test_matrix.csv) is deliberately staged:

| Block | Purpose | Releases |
|---|---|---:|
| Opening/closing reference | Detect day-to-day drift or damage | 5 + 5 |
| CG −5/+5 mm | Pitch/trajectory sensitivity | 8 + 8 |
| 6%-long/10%-short tails | Same-volume-ish area/arm tradeoff | 8 + 8 |
| 8% tail with short/long arm | Arm sensitivity | 6 + 6 |
| Fin off/4% | Yaw/roll-path consequence | 8 + 8 |
| Pitch, roll, yaw release errors of ±5° | Recovery, four each sign | 24 |

Use the same measured wing and datums throughout. Log **actual** mass, CG,
tail/fin geometry and alignment before each block; reset CG for alternative
tail geometries near the model's 10% margin, then record the real position.
Randomize release order within each block when practical. Use a simple,
repeatable near-level release fixture in a calm, clear test space; measure
initial airspeed/attitude from video instead of assuming the commanded setup
was achieved. If releasing from rest, allow enough height to accelerate before
judging glide; do not mix that launch type with a forward-speed release in
one comparison. Keep people out of the flight/catch corridor.

Two ordinary high-frame-rate cameras are enough initially: a calibrated side
view for pitch/height/speed and an overhead or front-oblique view for lateral
path, bank, and heading. Floor/wall distance marks and a visible airframe
marker aid reconstruction; synchronize views with a simple clap/flash.
Record release pose/velocity, pitch/roll/yaw versus time where visible,
trajectory, time to the first sustained glide segment, maximum excursion,
late-flight oscillation trend, landing/catch location, and any stall, spin,
or structural change. Report each configuration's success count and the
median/spread of path and attitude measures, not only the best flight.

An initial **screening** success is repeated forward glide with disturbances
decaying rather than growing and no consistent divergent roll/yaw; a useful
provisional gate is at least 8 of 10 nominal reference releases showing that
behavior. The gate is not a certification standard. For the smaller groups,
look for trends and repeat a block if differences are near launch scatter.

## H. What would justify CFD

Not a merely imperfect single drop. First measure geometry, CG, release
conditions, and repeat enough launches to separate aircraft behavior from
launch scatter. A focused CFD case becomes justified if a **repeatable**
trim/fin-effectiveness discrepancy lies outside measurement uncertainty,
video or tufts show persistent low-Re separation/laminar-bubble behavior,
or the candidate's small tail/fin Reynolds number makes the low-order
prediction decisive yet doubtful. Then analyze only one measured geometry
and flight condition to answer that specific viscous-flow question, alongside
simple section or physical tests. CFD is not needed to choose which
adjustable prototype to build first.

## Model limits and validation

AVL gives inviscid static derivatives and a thin-surface tail/fin model.
XFOIL profile drag is only a local-Re estimate and is **not** coupled back to
AVL lift or moment. Tail Re about 13–23k in the most interesting area/arm
tradeoff is a conspicuous uncertainty. Boom/fuselage drag, support
interference, fin drag, structural deflection, stall, and release dynamics are
absent. The CG-z reference defaults to zero until measured. The reported
L/D≈15 for the moderate reference is not a test objective or reliable
flight-range prediction.

The 5%/10% reruns, neutral-point/derivative margin cross-checks, finite-beta
yaw-sign test, and incidence-moment test pass in `verify_model.py`. The
historical XFOIL/LLT code and original `rigid_glider/run.py` remain intact.
