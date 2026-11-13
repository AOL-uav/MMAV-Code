# Passive-glider simplicity and robustness screen

Run `py rigid_glider/robustness_study.py` from the repository root. This report
describes the run with [`robustness_config.json`](robustness_config.json) and
[`baseline.json`](baseline.json). Numerical tables, plots, and the actual AVL
decks/outputs are in [`results/runs/rigid_glider/robustness/`](../results/runs/rigid_glider/robustness/).
The values below are **model screening results, not a flight demonstration**.

The main question is repeatable passive glide, not maximum L/D. Nominal
performance and tolerance sensitivity are shown separately; no weighted score
or global optimizer was used. Reference bands of trim CL 0.15–0.5 and static
margin 5–15% MAC are exploratory discussion aids, not design requirements.

## A. Simplest configuration test

Twenty-seven planar, untwisted, symmetric, tailless wings were run: quarter-
chord sweep 0/20/35°, AR 5/7/9, taper 0.4/0.7/1.0, span 1.2 m, NACA 0008,
zero dihedral, with CG individually set 10% MAC ahead of AVL's zero-alpha
neutral point. All 27 have zero-lift `Cm=0` at alpha=0; none has a useful
positive-lift trim point with positive static margin in the screened −4° to
10° attached-flow range. The maximum absolute `Cm(alpha=0)` is below AVL's
printed precision. See [planar.csv](../results/runs/rigid_glider/robustness/planar.csv)
and [planar moment curves](../results/runs/rigid_glider/robustness/planar_cm.svg).

This is consistent with the symmetry of a zero-camber, untwisted wing:
geometric sweep, taper, AR, and a CG shift change the moment *slope*, not the
zero-lift moment offset needed to trim at useful positive CL while retaining
a restoring slope. Setting CG at the neutral point would remove that slope,
not supply robust stability. This is a result for this architecture and
small-angle AVL model; it is **not** proof that every zero-sweep or untwisted
real wing is incapable of flight.

## B. Airfoil / section comparison

The historical Ncrit=5 XFOIL table contains symmetric NACA 0004, 0006, 0008,
0010, 0012, 0015, 0018, 0021, and 0024. This study compares 0004 through
0015 plus two cambered diagnostic sections at matched 2D CL and Re:

| Section | XFOIL CD at Re 80k, cl 0.2 | XFOIL CD at Re 130k, cl 0.4 |
|---|---:|---:|
| NACA 0004 | 0.01093 | 0.01926 |
| NACA 0006 | 0.01186 | 0.01424 |
| NACA 0008 | 0.01277 | 0.01209 |
| NACA 0010 | 0.01398 | 0.01201 |
| NACA 0012 | 0.01549 | 0.01298 |
| NACA 0015 | 0.01738 | 0.01464 |
| NACA 1208 | 0.01281 | 0.01069 |
| NACA 2408 | 0.01367 | 0.01025 |

NACA 0004 is the thinnest stored proxy for a flat-plate-like section. It is
**not a zero-thickness flat plate**, and its 2D XFOIL drag should not be
transferred to a literal plate. At cl 0.2/80k it looks best among these
symmetric sections; by cl 0.4/130k it is much worse than 0008/0010. There is
no universally superior thickness in these two operating points. Thicker
symmetrics 0018/0021/0024 exist but were not screened here because even 0015
does not suggest an immediate advantage for the thin-wing question.

AVL represents only the camberline for NACA sections; all untwisted symmetric
thicknesses therefore have the same inviscid trim failure. Positive-camber
NACA 1208/2408 provide somewhat lower XFOIL 2D drag at cl 0.4/130k, but their
AVL zero-alpha pitch moments are −0.0245/−0.0660. At 10% static margin,
1208's `Cm=0` occurs at negative CL (−0.177); 2408 has no crossing in the
screened alpha range. Ordinary positive camber thus makes *this tailless
pitch problem* worse, not better. The database contains no actual reflexed
section polar. A fixed trailing-edge-up reflex is tested separately below
in AVL only, with its drag/L/D intentionally omitted.

## C. Minimum additional complexity for pitch

All following cases use a 1.2 m span, AR 7, taper 0.65, 25° swept NACA 0008
main wing, zero dihedral. Each candidate's CG is relocated to about 10% MAC
static margin, so the test asks what each added feature buys when reasonably
balanced. At W/S=8 N/m²:

| Pitch configuration | Trim CL | Trim alpha | Speed | Complexity | Main caveat |
|---|---:|---:|---:|---|---|
| Plain planar wing | 0 | 0° | — | lowest | no useful lift at trim |
| −2° distributed washout | 0.108 | 2.14° | 11.02 m/s | higher | below reference CL band |
| −4° distributed washout | 0.215 | 4.27° | 7.80 m/s | higher | precise spanwise twist |
| −1° fixed TE-up offset | 0.101 | 1.86° | 11.37 m/s | higher | inviscid control-surface proxy |
| −2° fixed TE-up offset | 0.202 | 3.72° | 8.05 m/s | higher | no viscous polar for deflection |
| 5% area fixed horizontal tail at −3° | 0.180 | 2.53° | 8.52 m/s | low–moderate | requires central support |
| 10% area fixed horizontal tail at −3° | 0.310 | 4.23° | 6.49 m/s | low–moderate | requires central support |

See [fixes.csv](../results/runs/rigid_glider/robustness/fixes.csv) and
[moment curves](../results/runs/rigid_glider/robustness/fixes_cm.svg). Both
tail cases use a simple symmetric NACA 0008, straight 0.42 m span stabilizer
behind the wing, at about x=0.408 m leading edge; the 10% case has about
0.049 m chord. A fixed negative tail incidence gives a positive zero-alpha
pitch moment while tail area and arm increase stability. The 10% case's CG is
x=0.1745 m aft of root LE, versus x≈0.1599 m for the wing-only fixes. The
tail is the lowest-*precision-geometry* fix tested; it is not yet a packaging
or deployment solution.

The reported XFOIL-plus-AVL L/D is about 12.5 for the 5% tail, 15.5 for the
10% tail, and 15.2 for −4° washout. These are rough comparisons, not a ranking
criterion: the tail/boom parasitic drag and viscous moment corrections are
not modeled, the 10% tail section itself is only around Re 21k at trim, and
the fixed-reflex cases have no matching deflected-section polars at all.

## D. Manufacturability

Low-complexity geometry in this study: a planar swept/tapered main wing, CG
adjustment, simple center-joint dihedral, straight fixed center tail, and
straight fixed fins. A tail needs a support/boom and an incidence-setting
joint, but does not require the *large main wings* to hold a spanwise 4° warp.
The 10% tail still trims in AVL for −2° to −4° tail incidence (CL 0.207–0.413),
suggesting a forgiving exploratory angle range. This is not a measured printer
or assembly tolerance.

Distributed washout and a full-span 1–2° trailing-edge-up offset depend on
maintaining small angles over a printed span; varying airfoils, compound
curvature, and tiny control features would be still more demanding. A fixed
reflex may also complicate eventual wing flipping. A center tail or fin might
be easy to print but could interfere with eventual reorientation; that must
be checked mechanically later, not assumed away now.

## E. Robustness, separate from nominal L/D

The deliberately non-optimal reference tolerance case is the planar untwisted
wing plus 10% fixed tail. The values in
[`robustness_config.json`](robustness_config.json) are **exploratory ranges,
not measured manufacturing variation**. At nominal W/S, ±2 mm CG shifts give
trim CL 0.279–0.350 and margin 8.9–11.2% MAC; ±5 mm gives CL 0.241–0.434
and margin 7.2–12.9%. No CG case loses positive-lift pitch trim. ±20% loading
leaves AVL trim and derivatives unchanged and changes calculated speed from
5.80 to 7.11 m/s; that invariance is a limitation of inviscid AVL, not a
validated viscous robustness result. Changing tail area ±20% leaves positive
trim near CL 0.304–0.314 and margin about 8.7–11.3%.

Left/right main-wing incidence mismatch of ±1° leaves pitch trim essentially
unchanged but creates stability-axis `Cl'≈±0.00645` at pitch trim; ±2° creates
`Cl'≈±0.01291`. At nominal dynamic pressure the 1° case corresponds to about
0.041 N·m rolling moment. **This is not a hands-off roll equilibrium** and
its effect on recovery requires measured inertia and the coupled lateral
dynamics. The incidence perturbation is an AVL full-chord-normal control
proxy, not a measured wing warp. A simple ±2° center-joint dihedral error
changes `Cl_beta` from about −0.008 to −0.058/rad; pitch trim survives.

Left/right sweep mismatch was tested on the wing-only −4° washout candidate,
not the tail case: splitting a coplanar wing/tail into independent AVL
surfaces produced a nontrivial wake-interaction offset. Relative to the
**split-wing** zero-error reference, ±2° total sweep mismatch changes trim CL
by under 0.001 and roll coefficient by around 0.0001 in this model. Do not
compare those absolute rows directly with the mirrored-tail baseline.
Section/profile-drag variation was not propagated into pitch trim, because
the current AVL/XFOIL coupling cannot represent viscous moment changes.
See [perturbations.csv](../results/runs/rigid_glider/robustness/perturbations.csv)
and the [incidence/roll plot](../results/runs/rigid_glider/robustness/incidence_roll.svg).

## F. Dynamic-stability path

AVL supplies dimensionalizable static and rate derivatives, not a release
recovery verdict. For the 10% tail case it reports `Cl_p≈−0.451`,
`Cm_q≈−3.77`, and `Cn_r≈−0.00139` per normalized rate, alongside the static
derivatives. For example, dimensional pitch-rate moment slope is
`M_q = (0.5 rho V²) S c × Cm_q × c/(2V)`; analogous factors use span for roll
and yaw. The [AVL user primer](https://web.mit.edu/drela/Public/web/avl/AVL_User_Primer.pdf)
documents `.mass` files, eigenmodes, and state matrices, and explicitly
requires realistic mass, inertia, and CG for meaningful modes.

No 6-DOF trajectory or modal damping number is asserted here: mass
distribution, Ixx/Iyy/Izz/Ixz, realistic viscous drag and low-Re moment
behavior, and bounded release states are still unknown. Their input schema
is [`dynamics_inputs.example.json`](dynamics_inputs.example.json). Once
measured, first run AVL eigenmodes and small perturbed initial states; only
then decide whether a richer nonlinear flight model is warranted.

## G. Promising regions, not one final aircraft

1. **Manufacturing-first:** planar symmetric swept main wing plus a fixed
   center tail around 5–10% main-wing area, with a few degrees negative fixed
   incidence and adjustable CG. The 10% example has the clearest preliminary
   CG and tail-incidence tolerance, but needs yaw and roll work.
2. **Pure-wing benchmark:** similar swept symmetric wing with around 4°
   distributed washout, if printing/assembly can reproduce that twist; this
   has useful trim in AVL but higher geometric precision demand.
3. **Fixed-reflex benchmark:** around 2° trailing-edge-up offset can also
   provide positive-lift trim, but its drag and low-Re pitch behavior need
   new section data or a physical test before comparing it fairly with the
   tail or washout.

These are parameter neighborhoods for prototypes or focused refinement, not
optimized dimensions. The simple planar wing *alone* is not a promising
static pitch solution in this model.

## H. Vertical/yaw stability and physical information needed

Without fins, the 10% tail case has `Cn_beta≈+0.00244/rad`: positive but too
small to call a robust restoring margin. At fixed CG/wing/tail, center-fin
total area 4% of main wing raises it to +0.0131/rad, whereas paired tip-fin
total area 4% reaches +0.00713/rad and 8% reaches +0.0208/rad. These provide
a first *area bracket*, not a required fin design. The center 4% example is
roughly 0.095 m high by 0.087 m chord; the two-tip 8% example uses about the
same size on each tip. Fins were modeled as thin AVL surfaces, with no fin
profile drag or low-Re effectiveness correction. See
[fins.csv](../results/runs/rigid_glider/robustness/fins.csv) and
[yaw derivative plot](../results/runs/rigid_glider/robustness/fin_cnb.svg).

Highest-value physical inputs now are: (1) plausible total mass and component
locations, from which CG and inertia can be measured; (2) actual printed wing
thickness/roughness and permissible tail/fin mount geometry; (3) measured
left/right incidence and CG assembly scatter; (4) realistic release speed,
height, attitude, and angular-rate ranges; and (5) eventual folded/deployed
envelope limits when the passive airframe has first demonstrated repeatable
flight. None of the old deployment constraints were imposed in this screen.

## I. Next technical step

Build and repeatedly drop a **simple planar-wing, adjustable-CG, fixed-tail
prototype**, recording CG, mass/inertia, wing alignment, release state, and
trajectory. Include an easily swappable small center fin. The first aim is
repeatability and disturbance recovery, not L/D. Those measurements will
validate or falsify the strongest AVL assumptions and supply the missing
inputs for an honest dynamic-stability model.

## Model and validation notes

- AVL is thin-surface, inviscid, quasi-steady, small-disturbance VLM. It does
  not predict low-Re transition bubbles, stall, nonlinear separation, real
  printed roughness, or release transients. XFOIL table CD is at local Re but
  does not feed section lift or moment back into AVL trim. Positive static
  margin is not proof of a convergent glide.
- The reference wing loading of 8 N/m² corresponds to about 0.168 kg for
  this wing; it is only a placeholder. At trimmed CL 0.310, the 10% tail
  example has speed 6.49 m/s and root/tip main-wing Re about 90k/58k.
- The 10% tail mesh check (6×18 through 12×36 main-wing panels, with tail
  refinement) changed CL at alpha 4.23° from 0.3110 to 0.3098 and neutral
  point x from 0.19212 to 0.19185 m. This checks discretization locally, not
  aerodynamic validity. Splitting a coplanar wing/tail for asymmetric sweep
  gives a larger wake-interaction offset; the asymmetric sweep result is
  therefore reported only as a relative perturbation in the wing-only case.
- Existing historical XFOIL/LLT source/data and the original
  `rigid_glider/run.py` workflow remain intact. No CFD or optimizer was built.
