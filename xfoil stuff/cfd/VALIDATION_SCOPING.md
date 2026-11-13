# Validation scoping: Ananda/Sukumar/Selig 2015 flat plate, AR 3, Re 60k, alpha 4 deg

Scoping run only (2026-10-12). Nothing was tuned. Case data lives in MMAV-CFD at `/home/cfd/run/vp_a4_coarse` and `/home/cfd/run/vp_a4_medium`. Setup files, force histories and plots are in `validation_scoping/` next to this file.

## Plate geometry (all from the paper text)
- "A flat-plate wing with an aspect ratio (A) of 3, taper ratio (lambda) of 1, and zero sweep was chosen as a benchmark."
- "mean aerodynamic chord length of 3.5 in (88.9 mm)"; "thickness-to-chord ratio ... 4.3% ... translated to a wing thickness of 0.15 in (3.81 mm)".
- "the flat-plate airfoil has a 5-to-1 elliptical leading edge thickness ratio (semi-major axis of 0.375 in) and a 10-to-1 elliptical trailing edge thickness ratio (semi-major axis of 0.75 in)" (Fig. 1a).
- In chords: t = 0.0429 c, LE half-ellipse a = 0.1071 c, TE half-ellipse b = 0.2143 c, flat in between. Both ends are rounded tips (radius about 0.004 c at LE, 0.002 c at TE), not sharp.
- Re is based on the mean aerodynamic chord; moment is about the quarter chord ("mounted at the quarter chord", CM c/4). Reference area = planform area.
- Not stated in the extracted text: the shape of the wing tip. I assumed a flat square cut. The plate rotates about the quarter-chord point. The paper's wing was full span, sting-mounted at the tip, with tunnel corrections applied (solid/wake blockage, streamline curvature). Our model has none of that.
- Model: half wing, root on the y=0 symmetry plane, half span 1.5 c, Re = 60,000 (c = 1, U = 1, nu = 1/60000). AR 3 was tested at Re 60k (oil-flow runs at 60k, Fig. 11).

## Published comparison numbers
Available as text (no values for AR 3 at Re 60k, alpha 4 deg exist as text):
- CLmax 0.55 to 0.70 for all ten wings; stall for the AR 3 wing at about 10 deg (oil flow, Re 60k).
- CDmin about 0.01 to 0.02 for all wings; flat-plate skin friction 0.0066-0.0108 laminar, 0.0134-0.0164 turbulent (Re 60k-160k); at Re 80k: 0.0094 and 0.0155; XFOIL Cd0 0.0122. Pelletier and Mueller CDmin about 0.0175.
- Aerodynamic centre at low alpha about 0.20 to 0.27 c from the leading edge (Fig. 19 is at Re 100k), moving to about 0.5 c near CLmax. Eq. 4: x_ac = -CMalpha/CLalpha + 0.25.
- CM "small" before stall, "large negative" after stall.
- Oswald e0: about 0.81 (AR 2) and 0.33 (AR 5), rectangular. Literature at Re 140k: 0.39, 0.34, 0.22 for AR 2, 3, 6.
- Lift-slope formulas only: ideal CLalpha = 2pi A/(A+2) = 3.77/rad (CL(4 deg) = 0.263); Helmbold 3.36/rad (CL(4 deg) = 0.235). The text says AR 2 and 3 data "closely agree" with the ideal formula and that CLalpha rises toward it with Re.
- Uncertainties: dCL 0.0103 (3.3%), dCD 0.00107 (2.7%), dCM 0.00024 (4.6%). Tunnel turbulence below 0.1%.

Figure-only (not in the extracted text; read from figures or Ananda thesis Ref. [4]): CL, CD, CM versus alpha for AR 3 (Fig. 6 is Re 80k, Fig. 9 and 13 are Re 120k); CLalpha versus Re (Fig. 17, 18); CLmax versus Re (Fig. 14); x_ac versus alpha (Fig. 19); e0 (Fig. 22-24). The measured CL, CD, CM at alpha = 4 deg, Re 60k, AR 3 are therefore NOT available to me as numbers. The comparison below is limited to formulas and the text numbers above.

## Method
- pimpleFoam, laminar (no turbulence model), nu = 1.6667e-5.
- Time: backward (2nd order). Convection: `Gauss limitedLinearV 1` (limited 2nd-order TVD). Gradients and Laplacian: linear, corrected.
- PIMPLE: 1 outer, 3 pressure correctors, 1 non-orthogonal corrector. Adaptive time step, max Courant 1.5 (reached throughout; dt about 0.0035 coarse, 0.0018 medium; the leading-edge cells set the limit).
- Domain: x -10 to +15 c, y 0 to 7.5 c (6 c beyond the tip), z -8 to +8 c. Inlet fixed U, outlet p = 0, slip on top/bottom/side, symmetry at the root.
- Plate rotated nose-up 4 deg about (0.25, 0, 0). Flow is along +x. Lift = +z, drag = +x, pitch axis = +y, CofR = (0.25,0,0), Aref = 1.5 (half-wing area), lRef = 1.
- CM sign: nose-up positive. With pitch axis +y, a lift force F_z acting aft of the reference point at r = (d,0,0) gives M_y = -d F_z, nose-down, negative. A positive M_y rotates a point at x < 0 (the nose) upward. The paper's Eq. 4 (aerodynamic centre moves aft when CMalpha < 0) is consistent with the same convention.
- Mesh: blockMesh background 0.25 c, snappyHexMesh with box refinement around the plate, near wake and tip, plus 5 surface layers (expansion 1.2). Layers built cleanly (99.4% of faces fully extruded coarse, 99.7% medium; no failure).
- Start: uniform flow, impulsive start with dt = 1e-5. (potentialFoam initialisation failed because of a missing Phi solver entry; not needed.) The huge force spikes at t < 0.1 come from this impulsive start.
- Chord-passage time = c/U = 1.

## Runs (12 MPI ranks = physical cores)
| | Coarse | Medium |
|---|---|---|
| Cells | 1,003,245 | 5,759,810 |
| Near-plate cell size | 0.0078 c (level 5) | 0.0039 c (level 6) |
| Layers: first layer, total | 0.0019 c, 0.0139 c | 0.0009 c, 0.0070 c |
| checkMesh (default) | OK; max non-orth 64.9 (avg 6.2); max skew 2.16; max aspect 7.0 | OK; max non-orth 64.8 (avg 5.2); max skew 2.16; max aspect 8.3 |
| checkMesh -allGeometry | flags 21,293 concave cells (2.1%), the only failure | flags 72,411 concave cells (1.3%) |
| Start | uniform | coarse t = 20 field mapped (cellPointInterpolate) |
| Simulated time | 20 chord passages | 4 chord passages after mapping (stopped by me; see cost) |
| Mean dt / steps | 0.0035 / 5,639 | 0.0018 / 2,267 |
| Wall time | 5,233 s = 1.45 h | 14,293 s = 3.97 h |
| Peak memory (sum of ranks, sampled/20 s) | 2.5 GB | 9.8 GB (under the 12 GB cap) |
| y+ at plate | measured (t = 15): avg 2.8, min 0.13, max 18.8 (leading edge) | not measured (no field written); about 1.4 by scaling the first-layer height, an estimate |

### Force coefficients (mean, standard deviation of fluctuation about the mean, time-weighted)
| Run, window | CL | CD | CM c/4 (nose-up +) |
|---|---|---|---|
| Coarse, t = 8-20 | 0.2189, sd 0.0015 | 0.02850, sd 0.00008 | +0.00745, sd 0.0005 |
| Coarse, t = 12-20 | 0.2195 | 0.02853 | +0.00728 |
| Coarse, t = 4-20 | 0.2175 | 0.02843 | +0.00783 |
| Medium, t = 2-4 | 0.2190, sd 0.0026 | 0.02423, sd 0.00026 | +0.00820, sd 0.0013 |
| Medium, t = 1.5-4 | 0.2190 | 0.02435 | +0.00790 |

Coarse: quarter-window means across t = 8-20 vary CL 0.2175-0.2205, CD 0.0284-0.0286, CM 0.0070-0.0078; effectively steady (CL fluctuation 0.7%). Medium: not settled. Over its 2 to 4 window the quarter-window means drift CL 0.222 to 0.216, CD 0.0246 to 0.0240, CM 0.0067 to 0.0094, with an oscillation of period about 0.17 c visible in `validation_scoping/medium_forces_t2-4.png`.

Effective lift slope (assuming CL = 0 at alpha = 0 for a symmetric plate): 3.14/rad (coarse). Against the text formulas: 17% below the ideal formula (0.263), 7% below Helmbold (0.235). If CM(0) = 0 also holds, x_ac = 0.25 - (CM/alpha)/(CL/alpha) = 0.216 chord, inside the text's 0.20-0.27 range. That x_ac is a one-angle inference, not a slope.

## Comparison to the paper
It cannot be judged from the text: no measured CL, CD or CM exists as numbers at this angle and Re. What can be said: CL is plausible against the formulas (below ideal, as the text says low Re does); x_ac is in range; CD 0.024-0.029 at 4 deg is above the quoted CDmin of 0.01-0.02, as expected with induced drag (rough check: 0.0175 + CL^2/(pi*0.34*3) = 0.032, my arithmetic mixing two sources, order of magnitude only). The next step is to get the AR 3 Re 60k measured curves (digitise Fig. 17/18 values or obtain the thesis data) before saying anything about agreement. This is not a pass or a fail.

## What this resolution can and cannot be trusted for
- CL: probably within a few percent between meshes (0.219 both) but the medium run is still evolving, and no experimental number is at hand. Treat as plausible, not validated.
- CD: not converged under refinement. Medium is 15% lower than coarse and still falling. Do not quote CD from the coarse mesh.
- CM: values about 0.007-0.009 with a 0.001-0.002 drift between runs, versus an experimental uncertainty of 0.00024. Useful only as "small and near zero"; not for resolving CM differences.
- The coarse solution is steady: the leading-edge bubble and transition are not resolved (LE radius 0.004 c, cell 0.008 c). The medium mesh develops unsteadiness the coarse mesh damped, so the coarse "steady state" is partly numerical. Tip vortex and wake are coarse (0.031 c cells at best in the wake).
- Symmetry-plane half model suppresses any asymmetric instability. Wall distance to far field is 8 c; no tunnel-wall or sting effects are modelled.
- Not run: any other angle, Re 20k, any own-wing case.

## Cost on this machine (12 ranks)
Measured: coarse 0.93 s/step, 4.4 min per chord passage; medium 6.3 s/step, 59 min per chord passage (peak 9.8 GB, close to the machine limit for anything finer).
Extrapolated (estimates, not runs):
- (a) Re 60k plate, per angle of attack: coarse mesh, 12 chords to settle and average, about 1 to 1.5 h (20 chords measured: 1.45 h). A result I would trust (medium mesh from a mapped start, 10-15 chords to settle and average): about 10-15 h per angle. A finer mesh than medium does not fit in memory.
- (b) Same plate at Re 20k: same cell count and time step (Courant-limited, same velocity scale), so same cost per chord passage; about 1.5-2 h per angle on the coarse mesh. The boundary layer is 1.7x thicker, so the coarse mesh at Re 20k has roughly the boundary-layer resolution of the medium mesh at Re 60k. A trusted result is then about 2-3 h per angle, longer if the flow separates and goes unsteady, since more chords are needed to average.

## Issues met
- Initial coarse mesh with level-6 leading/trailing-edge boxes was 7.9 M cells, then 1.6 M; reduced to 1.0 M by removing those boxes. First solver attempt crashed (floating-point exception) from a uniform start at dt = 2e-3; fixed by starting at dt = 1e-5 with the adaptive step. First mapFields attempts failed (wrong method name, missing mapFieldsDict) and were rerun correctly.
- Mesh and run scripts are in `validation_scoping/` (mkcase.sh, mesh.sh, solve.sh, solve_map.sh, stats.py, plotslice.py). The plate surface script is in each `*_setup/geometry/make_plate.py`.
