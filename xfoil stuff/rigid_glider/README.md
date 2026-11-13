# Passive rigid flying-wing baseline

The dimensioned first-build test article (wing, CG range, tail, fin,
trailing-edge strip provision and first releases) is in
[`BUILD_ARTICLE.md`](BUILD_ARTICLE.md).

The later manufacturing-first fixed-tail work remains in
[`PROTOTYPE_REPORT.md`](PROTOTYPE_REPORT.md). The separate localized fixed-lip
comparison is documented in [`REFLEX_STUDY.md`](REFLEX_STUDY.md) and its
[`REFLEX_RESULTS.md`](REFLEX_RESULTS.md); it does not alter this baseline or
the historical polars. Its extension (`reflex_ext_*.py`, outputs in
`results/runs/rigid_glider/reflex_ext/`) is described in the same two files.
The 2026-10-09 decision that the tail branch is the first flight prototype
is superseded. As of 2026-10-11 the first test article is the P07 modular
wing in [`ARTICLE1.md`](ARTICLE1.md), flown clean, then with a lip, then
with the tail; the comparison behind it is
[`ARCH_COMPARISON.md`](ARCH_COMPARISON.md). `BUILD_ARTICLE.md` and
`PROTOTYPE_REPORT.md` describe the 1.2 m tail fallback branch.

Run from any working directory with Python 3.10+ on Windows:

```powershell
py C:/path/to/Codes/rigid_glider/run.py
```

Alternatively, from the project root: `py rigid_glider/run.py`. The explicit
inputs are in [`baseline.json`](baseline.json). Results go to
`results/runs/rigid_glider/`: `report.md`, `baseline.json`, `study.csv/json`,
SVG plots, and per-case raw AVL decks, commands, force/derivative/strip output,
and logs. Re-running replaces only these named generated case outputs. Use
`--config`, `--output`, or `--avl` to select alternate inputs, output location,
or AVL executable. No Python packages beyond the standard library are needed.
Smoke checks (including an actual AVL run when installed):
`py -m unittest discover -s rigid_glider -p test_*.py`.

The case is deliberately a modest swept, tapered, washed-out, symmetric-section
wing, **not** an optimized or build-ready design. `Wing` in `geometry.py`
accepts span and exactly one of AR/area, taper, quarter-chord sweep, tip twist,
dihedral, any four-digit NACA designation, and a CG x coordinate measured aft from
the root leading edge. Root/tip chord, MAC, area, and stations are derived.
Vertical surfaces are explicitly off; trying to enable them raises an error
until their dimensions and locations can be modeled. `FlightCondition` accepts
exactly one of wing loading or mass plus density, kinematic viscosity, and g.

## Models and data provenance

- `avl.py` generates an MIT AVL v3.52 deck and parses actual AVL force,
  derivative, neutral-point, and strip-force outputs. The default executable is
  `tools/avl/avl352.exe`; its provenance is documented there. The whole wing is
  present in the AVL model so sideslip derivatives can be calculated. Default
  paneling is 8 chordwise by 24 spanwise **per semispan**.
- `polars.py` reads only the selected section from the preserved historical
  `data/polars/Currentmegapolar.dat` XFOIL table, at Ncrit=5. It interpolates
  log(Re) and alpha where data actually exist. Given each AVL strip's local cl,
  it finds a nearby attached-flow 2D polar branch and area-weights its profile
  CD; it does not extrapolate beyond stored Re/alpha or across gaps >1 degree.
- The historical `src/wing/LLTparalleler.cpp` is intentionally **not** called.
  Its legacy design sweep assumes unswept, untwisted geometry and an independently
  fixed Re/CL. Reusing that solver for this swept, washed-out geometry would be
  misleading. AVL supplies the finite-wing induced drag and spanwise lift here.
- `run.py` finds a Cm=0 crossing from an alpha sweep, evaluates AVL again at
  the interpolated trim alpha, then solves `V=sqrt(2(W/S)/(rho CLtrim))` and
  `Re(y)=V c(y)/nu`. No fixed 80,000 Reynolds number or CL=0.5 is used.
  Because AVL is inviscid, its lift and moment are independent of Re, so this
  particular coupling requires no numerical iteration. XFOIL affects the
  **profile-drag estimate only**; a Reynolds-dependent viscous correction to
  3D lift/moment/trim is not implemented.

The generated report and `study.csv` present one-at-a-time changes in sweep,
washout, taper, and CG. A speed can be mathematically computed even if a
variant is outside the stored XFOIL polar range; in that case profile drag and
L/D are omitted. A Cm=0 point with nonpositive CL is **not** a positive-lift
gliding equilibrium.

## Coordinates and interpretation

AVL **geometry input** uses X aft/downstream, Y toward the right wing, Z up.
AVL's standard **force/moment output** instead labels positive x forward and
z down; the stability-axis roll/yaw outputs `Cl'`/`Cn'` are additionally
rotated by alpha. The study parses these primed moments consistently with
`Cl_beta`/`Cn_beta`. Alpha is nose-up; positive beta denotes wind from the
right; positive Cm is nose-up, positive Cl' right-wing-down roll, and positive
Cn' nose-right yaw. Positive static margin
`(xNP-xCG)/MAC`, or negative Cm_alpha, is the usual longitudinal restoring
condition. In this convention negative Cl_beta is a restoring dihedral effect,
while positive Cn_beta is directional weathervaning. Derivatives in `study.csv`
are per radian. See the [official AVL user primer](https://web.mit.edu/drela/Public/web/avl/AVL_User_Primer.pdf)
for AVL's precise axis and derivative definitions.

This is **static, small-disturbance, inviscid** screening. A positive trim CL
and static margin do not prove stable released flight. Missing physics include
transition/separation bubbles, viscous pitching-moment changes, stall,
surface roughness, structural flexibility, dynamic modes, release transients,
and parasitic drag. Symmetric-airfoil zero 2D moment does not preclude a
swept/washed-out wing's geometric 3D trim, but that trim is sensitive to model
fidelity and CG. No literal zero-thickness flat-plate XFOIL drag is used.

For small glide angle the workflow uses `L≈W`; exact steady glide has
`L=W cos(gamma)`. The difference is second order in glide angle, smaller than
the other current model uncertainties. Treat the reported L/D as comparative,
not a measured glide ratio.

## Bounded simplicity/robustness study

Run `py rigid_glider/robustness_study.py`. Editable exploratory reference bands,
geometry grid, and perturbation sizes are in `robustness_config.json`; these
are **not** measured manufacturing tolerances. The study screens 27 planar,
untwisted symmetric wings, compares a few minimal pitch fixes, section polars,
CG/loading/alignment perturbations, and simple center/tip fins. It writes raw
AVL output, CSV/JSON, and SVG plots under `results/runs/rigid_glider/robustness/`.
Read [`ROBUSTNESS_REPORT.md`](ROBUSTNESS_REPORT.md) for the engineering
interpretation and limitations. The original `run.py` baseline is preserved.

For the adjustable physical test article, run `py rigid_glider/verify_model.py`
and `py rigid_glider/prototype_study.py`. The bounded inputs are in
`prototype_config.json`; the verified sizing maps and first release matrix
are in [`PROTOTYPE_REPORT.md`](PROTOTYPE_REPORT.md). New outputs stay under
`results/runs/rigid_glider/prototype/`.
