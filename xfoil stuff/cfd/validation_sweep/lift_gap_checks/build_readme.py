#!/usr/bin/env python3
"""Writes README.md of this folder from the outputs of lift_gap_checks.py (tables are copied from the CSVs, nothing is typed by hand)."""
import csv, os
HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
rows = list(csv.DictReader(open("excess_table.csv")))
k = list(rows[0].keys())
col = lambda prefix: [c for c in k if c.startswith(prefix)][0]
ct, ca, ce, cu = col("tunnel_slope"), col("AVL_slope_LS"), col("excess_pct_vs_AVL_LS"), col("approx_uncertainty")
tbl = ["| wing (AR, taper) | Re | tunnel slope per deg | AVL slope per deg | excess % | approx. +/- % |", "|---|---|---|---|---|---|"]
for r in rows:
    tbl.append(f"| AR {r['AR']}, taper {r['taper']} | {int(float(r['Re'])):,} | {float(r[ct]):.4f} | {float(r[ca]):.4f} | {float(r[ce]):+.1f} | {float(r[cu]):.1f} |")
pl = list(csv.DictReader(open("avl_planforms.csv")))
hdr = list(pl[0].keys())
ptbl = ["| " + " | ".join(hdr) + " |", "|" + "---|" * len(hdr)]
for r in pl:
    ptbl.append("| " + " | ".join(r.values()) + " |")

import collections
by_ = collections.defaultdict(list)
for r in rows:
    by_[r["wing"]].append((float(r["Re"]), float(r[ce])))
zc = {}
for w_, v_ in sorted(by_.items()):
    xs_ = [a / 1e4 for a, _ in v_]; ys_ = [b for _, b in v_]; mx_ = sum(xs_) / len(xs_); my_ = sum(ys_) / len(ys_)
    tr_ = sum((x - mx_) * (y - my_) for x, y in zip(xs_, ys_)) / sum((x - mx_) ** 2 for x in xs_)
    zc[w_] = (mx_ - my_ / tr_) * 1e4
zc_pos = sorted(v for v in zc.values() if v > 0)
zc_txt = "; ".join(f"AR {w[2]} taper {w.split('taper')[1]}: " + (f"{v / 1e3:.0f}k" if v > 0 else "no crossing at positive Re") for w, v in sorted(zc.items()))
summ = open("excess_summary.txt").read()
wall = open("wall_corrections.txt").read()
readme = f"""# Lift-gap checks (no CFD): is the tunnel's lift excess over AVL a flow effect or a rig effect?

Zero-cost check, 2026-10-14. Script `lift_gap_checks.py` (stdlib; runs AVL 3.52 serially; `build_readme.py` writes this file from its outputs). No CFD, no access to the WSL distro. Facts only; what they mean for the CFD lift gap is left to the reviewer.

## Method
- **Task A.** AVL slope for all ten planforms in `research/plate_data/ananda2012_flatplate_all_wings.csv`: flat, zero thickness, zero camber, straight-tapered trapezoids with mean aerodynamic chord 3.5 in (thesis PDF p. 50), spans from thesis Fig. 3.14 (PDF p. 51), **quarter-chord line straight (unswept)** (thesis PDF p. 49: taper variations "about the quarter chord"; plate_balance.py header). Root and tip chord follow from the MAC; b^2/S reproduces the nominal AR to better than 1 % for every wing (table below). AVL panelling 8 chordwise x 12 spanwise per half wing (the density of `../setup_checks`). AVL's reference area is each wing's own planform area, as for the thesis coefficients.
- **AVL slopes, three ways** (`avl_planforms.csv`): tangent at 0 deg (AVL's stability derivative CLa), secant CL(4 deg)/4 (as in the setup check), and a least-squares line through CL at -4, -2, 0, 2, 4, 6 deg (the -4 and -2 points are the sign mirror of +4 and +2 for a symmetric flat plate). **The comparison below uses the least-squares line over -4 to 6 deg, the window the tunnel fit used.** The excess against the other two is in `excess_table.csv`.
- **Task B.** Tunnel slopes are read, not recomputed, from `results/runs/rigid_glider/plate_balance/step2_fits.csv`: least-squares fits of CL against alpha over **alpha = -4 to 6 deg** (FIT_RANGE in plate_balance.py), one per run and sweep leg; the slope used is the mean of the up and down legs. 40 runs, 10 wings. The `research/plate_data/README.md` fits (-6 to 6 deg, AR 3 taper 1 only: 0.0607 per deg at Re 60k, 0.0682 at 120k) are consistent with the -4..6 fits (0.0596 and 0.0602 at 60k, 0.0680 and 0.0683 at 120k). The approximate uncertainty column is the rms residual of the fit over sqrt(Sxx) for a uniform -4..6 window, combined with half the up/down-leg difference; a rough estimate only.

## Panelling check (AR 3 taper 1; `panelling_check.csv`)
AVL slope (least-squares, -4..6 deg) per deg: 6x8 panels 0.05469, 8x12 0.05469, 12x20 0.05470, 24x40 0.05470. **The slope is converged to 0.02 %**, inside the 1 % asked for.

## AVL planforms (`avl_planforms.csv`)
{chr(10).join(ptbl)}

## Excess of the tunnel slope over AVL (`excess_table.csv`; plot `excess_vs_Re.svg`)
{chr(10).join(tbl)}

Summary (`excess_summary.txt`):
```
{summ}```
Plot: `excess_vs_Re.svg` (excess percent against Reynolds number, one line per wing; colour by aspect ratio, dash pattern by taper; white plot background).

## Wall corrections (`wall_corrections.txt`), thesis Sec. 3.8
**Formulas** (thesis PDF pp. 57-59 = printed 41-43): solid blockage eps_sb = K1 tau1 (wing volume) / C^(3/2) (Eq. 3.20); wake blockage eps_wb = (S/4C) CD0 + (5S/4C)(CDu - CDi - CD0) (Eq. 3.21, Maskell); eps_T = eps_sb + eps_wb; q_c = q_u (1 + eps_T)^2 and V_c = V_u (1 + eps_T) (Eq. 3.23-3.25); streamline curvature: alpha_c = alpha_u + delta_SC (1 + tau2) CL_c (3.26), CD_c = D_u/(q_c S) + delta_SC CL_c^2 (3.27), CM_c = M_u/(q_c S c) + 0.125 tau2 delta_SC (dCL_c/dalpha_c) (3.28).
**Inputs.** Test section 2.8 x 4.0 ft (0.853 x 1.219 m); the wing is mounted with its span vertical and the model span fraction b/H has maximum 0.52 (thesis PDF p. 50), which puts H = 2.8 ft = 33.6 in along the span and 4.0 ft = 48 in along the lift direction, C = 1613 in^2. Model (AR 3, taper 1): b = 10.5 in, S = 36.75 in^2, S/C = 0.0228, MSF = 0.312.
**Not in the thesis text:** K1, tau1, delta and tau2 are "determined from figures in Ref. 55" and no values are printed (pages 57-59 give equations only). They are **not looked up here**: delta is computed from the image-vortex system of a horseshoe vortex of span pi b/4 centred in a closed 33.6 x 48 in section (my own calculation; the wing is assumed centred, the thesis gives no offset); K1 tau1 is bounded between 0.5 and 1.2 and tau2 between 0 and 0.3 (assumed bounds, not data). The second term of Eq. 3.22 as extracted from the PDF is ambiguous, so two readings of the wake-blockage drag term are evaluated (they agree to the digits quoted).
```
{wall}```
Summary for the AR 3 taper 1 plate at Re 60,000:
| quantity | 4 deg | 6 deg |
|---|---|---|
| boundary factor delta (computed) | 0.19 | 0.19 |
| angle correction alpha_c - alpha_u | 0.061 to 0.079 deg (1.5 to 2.0 % of the angle) | 0.093 to 0.122 deg (1.6 to 2.0 %) |
| blockage eps_sb; eps_wb; eps_T | 4e-5 to 9e-5; 5.5e-4; 6e-4 | 4e-5 to 9e-5; 5.9e-4; 6e-4 to 7e-4 |
| dynamic-pressure factor q_c/q_u - 1 | +0.12 to +0.13 % | +0.13 % |
| CL, CM and CD change from blockage | -0.12 to -0.13 % | -0.13 % |
| effect on the plotted lift slope of the angle shift | lowered by about 1.5 to 2.0 % | same |
| moment term 0.125 tau2 delta_SC dCL/dalpha | 0 to +0.0006 (tunnel CM +0.0122) | 0 to +0.0006 (tunnel CM +0.0150) |
The angle correction is added to the angle, so applying the corrections lowers the plotted slope by 1.5 to 2 %. The computed angle correction is 1.5 to 2.0 % of the angle at 4 deg; the reviewer's rough estimate was about 1 % of angle at 4 deg (0.04 deg): same order, somewhat smaller. The blockage terms are of order 0.1 %. Limits: wing position in the section unknown (centring assumed); K1 tau1 and tau2 bounded, not available; the image sum treats the sting and the wing-tip mount as absent.

## Were the published table values already corrected?
- Thesis PDF p. 39 (printed 23): "Once a run was complete, the data was corrected using correction methods detailed in Chapter 3.8. Raw, corrected and conditions data at all angles of attack tested were saved to different output files for future data reduction."
- Journal paper (Aerosp. Sci. Technol. 42 (2015), p. 395, Sec. 2.3): "Once the run was complete, the data were corrected for three-dimensional tunnel effects (solid blockage, wake blockage, and streamline curvature) according to the methods outlined in Ref. [5]."
- Thesis Appendix B and C introductions (PDF pp. 136 and 166, printed 120 and 150): the tables contain "all of the polar data seen in Chapter 5" and "all of the lift and moment data seen in Chapter 5". These introductions do not say corrected or raw in so many words; the chapter-level statements above say the run data were corrected. Thesis PDF p. 60 adds: "Uncertainties due to precision or wind tunnel corrections are not accounted for."

## Conclusion and caveats (reviewer-approved wording, 2026-10-14)
**The tunnel's lift excess over AVL is not a wall-correction or fixed-offset artefact: it depends on Reynolds number on every wing.** All ten wings rise with Reynolds number (+1.5 to +2.9 points per 10,000, table above), while the computed wall corrections are about 1.5 to 2 % of the angle and about 0.1 % in blockage and the thesis correction equations (3.20 to 3.28) contain Reynolds number only through the measured coefficients. No mechanism is named here as fact.
Caveats:
- **A speed-dependent facility effect is not excluded.** Changing Reynolds number in that tunnel changes the speed, and with it the turbulence and the acoustics. Results in this Reynolds-number range differ between tunnels (`research/bird_research_notes.md` section 6.3, secondary sources; below).
- **Aeroelastic twist is unlikely but untested.** AR 2, the shortest span, has the largest excess at a given Reynolds number (for example 18.6 % at Re 80,000 against 15.0 % for AR 3 and 6.8 % for AR 4, all taper 1).
- **AVL models zero thickness.** A 4.3 % plate with a rounded nose may carry a few percent more inviscid lift slope than AVL gives; that could account for part of the floor of the excess but not for its trend with Reynolds number.
- **CFD.** The laminar CFD probably lacks a lift-raising viscous effect present in this tunnel at Re 60,000 (probable, not established). The CFD's own lift-slope shortfall was not measured as a slope (no slope was fitted to the CFD points), and the refined-mesh result is one angle (6 deg).
- **Below Reynolds number 60,000 this is untested, and no number is given for it.** The excess falls with falling Reynolds number in these data; straight-line extrapolations of the per-wing trends reach zero at the Reynolds numbers listed here, and would go negative below: {zc_txt}. These are extrapolations of straight lines beyond the data, not predictions, so even the sign of the excess below 60,000 is unknown. The secondary sources disagree: Laitone found lift slopes more than 20 % above lifting-line theory at Re 40,000, McArthur found no significant disagreement at Re 10,000 to 20,000, and Pelletier & Mueller are summarised as finding lifting-line theory adequate at Re 60,000 to 200,000 (`research/bird_research_notes.md` section 6.3). Nothing in this folder says how AVL's lift slope compares with real wings at lower Reynolds numbers.

## Assumptions and limits
- Unswept quarter-chord line for the tapered wings; straight trapezoids; MAC 3.5 in for all wings; spans as in Fig. 3.14; zero thickness in AVL (the plates are 4.3 % thick).
- The tunnel fit window is -4 to 6 deg; the up/down-leg slope difference averages 2.2 % of AVL's slope, and the approximate fit s.e. 1.1 %.
- Wall-correction constants K1 tau1 and tau2 are bounded, not looked up; delta is my image-vortex calculation for a centred wing.
"""
open("README.md", "w", encoding="utf-8").write(readme)
print("README.md", len(readme))
