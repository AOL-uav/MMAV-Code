# Lift-gap checks (no CFD): is the tunnel's lift excess over AVL a flow effect or a rig effect?

Zero-cost check, 2026-10-14. Script `lift_gap_checks.py` (stdlib; runs AVL 3.52 serially; `build_readme.py` writes this file from its outputs). No CFD, no access to the WSL distro. Facts only; what they mean for the CFD lift gap is left to the reviewer.

## Method
- **Task A.** AVL slope for all ten planforms in `research/plate_data/ananda2012_flatplate_all_wings.csv`: flat, zero thickness, zero camber, straight-tapered trapezoids with mean aerodynamic chord 3.5 in (thesis PDF p. 50), spans from thesis Fig. 3.14 (PDF p. 51), **quarter-chord line straight (unswept)** (thesis PDF p. 49: taper variations "about the quarter chord"; plate_balance.py header). Root and tip chord follow from the MAC; b^2/S reproduces the nominal AR to better than 1 % for every wing (table below). AVL panelling 8 chordwise x 12 spanwise per half wing (the density of `../setup_checks`). AVL's reference area is each wing's own planform area, as for the thesis coefficients.
- **AVL slopes, three ways** (`avl_planforms.csv`): tangent at 0 deg (AVL's stability derivative CLa), secant CL(4 deg)/4 (as in the setup check), and a least-squares line through CL at -4, -2, 0, 2, 4, 6 deg (the -4 and -2 points are the sign mirror of +4 and +2 for a symmetric flat plate). **The comparison below uses the least-squares line over -4 to 6 deg, the window the tunnel fit used.** The excess against the other two is in `excess_table.csv`.
- **Task B.** Tunnel slopes are read, not recomputed, from `results/runs/rigid_glider/plate_balance/step2_fits.csv`: least-squares fits of CL against alpha over **alpha = -4 to 6 deg** (FIT_RANGE in plate_balance.py), one per run and sweep leg; the slope used is the mean of the up and down legs. 40 runs, 10 wings. The `research/plate_data/README.md` fits (-6 to 6 deg, AR 3 taper 1 only: 0.0607 per deg at Re 60k, 0.0682 at 120k) are consistent with the -4..6 fits (0.0596 and 0.0602 at 60k, 0.0680 and 0.0683 at 120k). The approximate uncertainty column is the rms residual of the fit over sqrt(Sxx) for a uniform -4..6 window, combined with half the up/down-leg difference; a rough estimate only.

## Panelling check (AR 3 taper 1; `panelling_check.csv`)
AVL slope (least-squares, -4..6 deg) per deg: 6x8 panels 0.05469, 8x12 0.05469, 12x20 0.05470, 24x40 0.05470. **The slope is converged to 0.02 %**, inside the 1 % asked for.

## AVL planforms (`avl_planforms.csv`)
| AR_nominal | taper | span_in | root_chord_in | tip_chord_in | area_in2 | AR_from_b2_over_S | AVL_CLa_tangent_at_0_per_deg | AVL_CL4_over_4_secant_per_deg | AVL_LS_fit_-4_to_6_per_deg |
|---|---|---|---|---|---|---|---|---|---|
| 2 | 0.5 | 6.75 | 4.5 | 2.25 | 22.781 | 2.0 | 0.04372 | 0.0436 | 0.04354 |
| 2 | 0.75 | 6.95 | 3.973 | 2.9797 | 24.161 | 1.999 | 0.04355 | 0.04343 | 0.04337 |
| 2 | 1.0 | 7.0 | 3.5 | 3.5 | 24.5 | 2.0 | 0.04318 | 0.04306 | 0.04301 |
| 3 | 0.5 | 10.13 | 4.5 | 2.25 | 34.189 | 3.001 | 0.05606 | 0.05593 | 0.05586 |
| 3 | 0.75 | 10.43 | 3.973 | 2.9797 | 36.258 | 3.0 | 0.05559 | 0.05545 | 0.05539 |
| 3 | 1.0 | 10.5 | 3.5 | 3.5 | 36.75 | 3.0 | 0.05489 | 0.05475 | 0.05469 |
| 4 | 0.5 | 13.5 | 4.5 | 2.25 | 45.563 | 4.0 | 0.06469 | 0.06455 | 0.06448 |
| 4 | 0.75 | 13.91 | 3.973 | 2.9797 | 48.356 | 4.001 | 0.06399 | 0.06385 | 0.06379 |
| 4 | 1.0 | 14.0 | 3.5 | 3.5 | 49.0 | 4.0 | 0.06304 | 0.0629 | 0.06283 |
| 5 | 1.0 | 17.5 | 3.5 | 3.5 | 61.25 | 5.0 | 0.069 | 0.06886 | 0.0688 |

## Excess of the tunnel slope over AVL (`excess_table.csv`; plot `excess_vs_Re.svg`)
| wing (AR, taper) | Re | tunnel slope per deg | AVL slope per deg | excess % | approx. +/- % |
|---|---|---|---|---|---|
| AR 2, taper 0.5 | 79,998 | 0.0491 | 0.0435 | +12.8 | 0.9 |
| AR 2, taper 0.5 | 89,997 | 0.0499 | 0.0435 | +14.7 | 1.2 |
| AR 2, taper 0.5 | 100,000 | 0.0510 | 0.0435 | +17.0 | 1.0 |
| AR 2, taper 0.5 | 119,995 | 0.0528 | 0.0435 | +21.3 | 1.4 |
| AR 2, taper 0.75 | 79,998 | 0.0493 | 0.0434 | +13.8 | 2.3 |
| AR 2, taper 0.75 | 100,001 | 0.0512 | 0.0434 | +18.2 | 2.4 |
| AR 2, taper 0.75 | 119,997 | 0.0535 | 0.0434 | +23.3 | 1.6 |
| AR 2, taper 0.75 | 139,997 | 0.0546 | 0.0434 | +25.9 | 1.9 |
| AR 2, taper 1.0 | 80,000 | 0.0510 | 0.0430 | +18.6 | 1.6 |
| AR 2, taper 1.0 | 99,999 | 0.0526 | 0.0430 | +22.2 | 1.5 |
| AR 2, taper 1.0 | 119,998 | 0.0542 | 0.0430 | +26.0 | 2.0 |
| AR 2, taper 1.0 | 139,995 | 0.0554 | 0.0430 | +28.9 | 1.6 |
| AR 2, taper 1.0 | 159,992 | 0.0562 | 0.0430 | +30.6 | 1.6 |
| AR 3, taper 0.5 | 59,997 | 0.0593 | 0.0559 | +6.2 | 0.9 |
| AR 3, taper 0.5 | 79,999 | 0.0621 | 0.0559 | +11.1 | 1.2 |
| AR 3, taper 0.5 | 99,995 | 0.0646 | 0.0559 | +15.7 | 1.1 |
| AR 3, taper 0.5 | 119,992 | 0.0666 | 0.0559 | +19.3 | 1.4 |
| AR 3, taper 0.75 | 60,000 | 0.0600 | 0.0554 | +8.3 | 0.8 |
| AR 3, taper 0.75 | 80,004 | 0.0629 | 0.0554 | +13.6 | 1.4 |
| AR 3, taper 0.75 | 99,997 | 0.0657 | 0.0554 | +18.6 | 1.1 |
| AR 3, taper 0.75 | 119,998 | 0.0682 | 0.0554 | +23.1 | 1.2 |
| AR 3, taper 1.0 | 59,999 | 0.0599 | 0.0547 | +9.5 | 1.1 |
| AR 3, taper 1.0 | 80,000 | 0.0629 | 0.0547 | +15.0 | 1.1 |
| AR 3, taper 1.0 | 99,999 | 0.0660 | 0.0547 | +20.7 | 1.2 |
| AR 3, taper 1.0 | 119,998 | 0.0682 | 0.0547 | +24.6 | 1.4 |
| AR 4, taper 0.5 | 59,999 | 0.0644 | 0.0645 | -0.2 | 1.0 |
| AR 4, taper 0.5 | 79,999 | 0.0662 | 0.0645 | +2.7 | 2.6 |
| AR 4, taper 0.5 | 99,998 | 0.0689 | 0.0645 | +6.8 | 2.2 |
| AR 4, taper 0.5 | 120,000 | 0.0739 | 0.0645 | +14.7 | 4.4 |
| AR 4, taper 0.75 | 59,996 | 0.0676 | 0.0638 | +5.9 | 1.0 |
| AR 4, taper 0.75 | 79,996 | 0.0716 | 0.0638 | +12.2 | 2.3 |
| AR 4, taper 0.75 | 99,998 | 0.0759 | 0.0638 | +19.0 | 2.9 |
| AR 4, taper 0.75 | 119,997 | 0.0785 | 0.0638 | +23.0 | 3.4 |
| AR 4, taper 1.0 | 59,998 | 0.0650 | 0.0628 | +3.5 | 3.8 |
| AR 4, taper 1.0 | 79,997 | 0.0671 | 0.0628 | +6.8 | 3.3 |
| AR 4, taper 1.0 | 99,996 | 0.0689 | 0.0628 | +9.7 | 2.2 |
| AR 4, taper 1.0 | 120,001 | 0.0712 | 0.0628 | +13.4 | 1.2 |
| AR 5, taper 1.0 | 60,000 | 0.0727 | 0.0688 | +5.6 | 1.0 |
| AR 5, taper 1.0 | 79,999 | 0.0763 | 0.0688 | +10.9 | 1.5 |
| AR 5, taper 1.0 | 99,996 | 0.0805 | 0.0688 | +17.0 | 1.2 |

Summary (`excess_summary.txt`):
```
AVL panelling check, AR 3 taper 1 (LS slope over -4..6 deg, 8x12 per half = reference): 6x8: 0.05469 (+0.00 %); 8x12: 0.05469 (+0.00 %); 12x20: 0.05470 (+0.02 %); 24x40: 0.05470 (+0.02 %). Largest difference 0.02 %.

40 runs, 10 wings. Excess of the tunnel low-angle slope (fit -4..6 deg, mean of up and down legs) over AVL's slope fitted over the same -4..6 deg (primary comparison):
  overall min -0.2 %, max 30.6 %, mean 15.2 %. Against AVL's secant slope CL(4)/4: min -0.3 %, max 30.4 %; against the tangent slope at 0 deg: min -0.5 %, max 30.0 %.
  per nominal Reynolds number (spread across wings at the same Re):
    Re  60,000: n =  7 wings, min  -0.2 %, max   9.5 %, mean   5.6 %, sd  3.0 points
    Re  80,000: n = 10 wings, min   2.7 %, max  18.6 %, mean  11.7 %, sd  4.2 points
    Re  90,000: n =  1 wings, min  14.7 %, max  14.7 %, mean  14.7 %, sd  0.0 points
    Re 100,000: n = 10 wings, min   6.8 %, max  22.3 %, mean  16.5 %, sd  4.5 points
    Re 120,000: n =  9 wings, min  13.4 %, max  26.0 %, mean  21.0 %, sd  4.1 points
    Re 140,000: n =  2 wings, min  25.9 %, max  28.9 %, mean  27.4 %, sd  1.5 points
    Re 160,000: n =  1 wings, min  30.6 %, max  30.6 %, mean  30.6 %, sd  0.0 points
  per wing (excess against Reynolds number): range over the Re tested and the least-squares trend per 10,000 of Re:
    AR2_taper0.5   Re 80k-120k (4 runs):  12.8 % ->  21.3 %  (trend +2.15 points per 10k Re)
    AR2_taper0.75  Re 80k-140k (4 runs):  13.8 % ->  25.9 %  (trend +2.08 points per 10k Re)
    AR2_taper1     Re 80k-160k (5 runs):  18.6 % ->  30.6 %  (trend +1.53 points per 10k Re)
    AR3_taper0.5   Re 60k-120k (4 runs):   6.2 % ->  19.3 %  (trend +2.19 points per 10k Re)
    AR3_taper0.75  Re 60k-120k (4 runs):   8.3 % ->  23.1 %  (trend +2.47 points per 10k Re)
    AR3_taper1     Re 60k-120k (4 runs):   9.5 % ->  24.6 %  (trend +2.55 points per 10k Re)
    AR4_taper0.5   Re 60k-120k (4 runs):  -0.2 % ->  14.7 %  (trend +2.43 points per 10k Re)
    AR4_taper0.75  Re 60k-120k (4 runs):   5.9 % ->  23.0 %  (trend +2.90 points per 10k Re)
    AR4_taper1     Re 60k-120k (4 runs):   3.5 % ->  13.4 %  (trend +1.63 points per 10k Re)
    AR5_taper1     Re 60k-100k (3 runs):   5.6 % ->  17.0 %  (trend +2.84 points per 10k Re)
  mean up/down-leg slope difference 0.0013 per deg (2.2 % of AVL's slope); mean approximate fit s.e. 1.1 % of AVL's slope.
```
Plot: `excess_vs_Re.svg` (excess percent against Reynolds number, one line per wing; colour by aspect ratio, dash pattern by taper; white plot background).

## Wall corrections (`wall_corrections.txt`), thesis Sec. 3.8
**Formulas** (thesis PDF pp. 57-59 = printed 41-43): solid blockage eps_sb = K1 tau1 (wing volume) / C^(3/2) (Eq. 3.20); wake blockage eps_wb = (S/4C) CD0 + (5S/4C)(CDu - CDi - CD0) (Eq. 3.21, Maskell); eps_T = eps_sb + eps_wb; q_c = q_u (1 + eps_T)^2 and V_c = V_u (1 + eps_T) (Eq. 3.23-3.25); streamline curvature: alpha_c = alpha_u + delta_SC (1 + tau2) CL_c (3.26), CD_c = D_u/(q_c S) + delta_SC CL_c^2 (3.27), CM_c = M_u/(q_c S c) + 0.125 tau2 delta_SC (dCL_c/dalpha_c) (3.28).
**Inputs.** Test section 2.8 x 4.0 ft (0.853 x 1.219 m); the wing is mounted with its span vertical and the model span fraction b/H has maximum 0.52 (thesis PDF p. 50), which puts H = 2.8 ft = 33.6 in along the span and 4.0 ft = 48 in along the lift direction, C = 1613 in^2. Model (AR 3, taper 1): b = 10.5 in, S = 36.75 in^2, S/C = 0.0228, MSF = 0.312.
**Not in the thesis text:** K1, tau1, delta and tau2 are "determined from figures in Ref. 55" and no values are printed (pages 57-59 give equations only). They are **not looked up here**: delta is computed from the image-vortex system of a horseshoe vortex of span pi b/4 centred in a closed 33.6 x 48 in section (my own calculation; the wing is assumed centred, the thesis gives no offset); K1 tau1 is bounded between 0.5 and 1.2 and tau2 between 0 and 0.3 (assumed bounds, not data). The second term of Eq. 3.22 as extracted from the PDF is ambiguous, so two readings of the wake-blockage drag term are evaluated (they agree to the digits quoted).
```
Wall-correction sizes for the AR 3 taper 1 plate (thesis PDF pp. 57-59 = printed 41-43, Eq. 3.20-3.28), computed here
Test section 33.6 in (span direction, H) x 48.0 in (lift direction) = 1613 in^2; model span 10.5 in, area 36.75 in^2, S/C = 0.02279; MSF = b/H = 0.312.
From the table (run 0383ar): minimum CD 0.0136; CD against CL^2 slope 0.190 (fit over |CL| < 0.5, |alpha| < 8 deg).
alpha_u about 4 deg (tunnel CL 0.2465, CD 0.0282, CM +0.0122): [centre of the span] boundary factor delta = 0.189; angle correction 0.061 to 0.079 deg = 1.52 to 1.97 % of the angle (tau2 0..0.3 assumed)
alpha_u about 4 deg (tunnel CL 0.2465, CD 0.0282, CM +0.0122): [span average] boundary factor delta = 0.191; angle correction 0.061 to 0.080 deg = 1.53 to 1.99 % of the angle (tau2 0..0.3 assumed)
   blockage: solid eps_sb 3.6e-05 to 8.7e-05 (K1 tau1 assumed 0.5..1.2), wake eps_wb 5.5e-04 to 5.5e-04; eps_T 5.9e-04 to 6.4e-04; q_c/q_u - 1 = 0.118 to 0.128 %; CL and CM change by the inverse of that factor (-0.128 to -0.118 %)
   effect on the lift slope if the plotted angle is shifted by delta_alpha: slope reduced by about 1.53 to 1.99 % (CL itself unchanged)
   moment term 0.125 tau2 delta_SC (dCL/dalpha) with tau2 0..0.3: 0 to 0.00057 (tunnel CM at this angle +0.0122)
alpha_u about 6 deg (tunnel CL 0.3761, CD 0.0448, CM +0.0150): [centre of the span] boundary factor delta = 0.189; angle correction 0.093 to 0.121 deg = 1.54 to 2.01 % of the angle (tau2 0..0.3 assumed)
alpha_u about 6 deg (tunnel CL 0.3761, CD 0.0448, CM +0.0150): [span average] boundary factor delta = 0.191; angle correction 0.094 to 0.122 deg = 1.56 to 2.03 % of the angle (tau2 0..0.3 assumed)
   blockage: solid eps_sb 3.6e-05 to 8.7e-05 (K1 tau1 assumed 0.5..1.2), wake eps_wb 5.9e-04 to 5.9e-04; eps_T 6.3e-04 to 6.8e-04; q_c/q_u - 1 = 0.125 to 0.135 %; CL and CM change by the inverse of that factor (-0.135 to -0.125 %)
   effect on the lift slope if the plotted angle is shifted by delta_alpha: slope reduced by about 1.56 to 2.03 % (CL itself unchanged)
   moment term 0.125 tau2 delta_SC (dCL/dalpha) with tau2 0..0.3: 0 to 0.00057 (tunnel CM at this angle +0.0150)
```
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
- **Below Reynolds number 60,000 this is untested, and no number is given for it.** The excess falls with falling Reynolds number in these data; straight-line extrapolations of the per-wing trends reach zero at the Reynolds numbers listed here, and would go negative below: AR 2 taper 0.5: 21k; AR 2 taper 0.75: 12k; AR 2 taper 1: no crossing at positive Re; AR 3 taper 0.5: 30k; AR 3 taper 0.75: 26k; AR 3 taper 1: 22k; AR 4 taper 0.5: 65k; AR 4 taper 0.75: 38k; AR 4 taper 1: 39k; AR 5 taper 1: 41k. These are extrapolations of straight lines beyond the data, not predictions, so even the sign of the excess below 60,000 is unknown. The secondary sources disagree: Laitone found lift slopes more than 20 % above lifting-line theory at Re 40,000, McArthur found no significant disagreement at Re 10,000 to 20,000, and Pelletier & Mueller are summarised as finding lifting-line theory adequate at Re 60,000 to 200,000 (`research/bird_research_notes.md` section 6.3). Nothing in this folder says how AVL's lift slope compares with real wings at lower Reynolds numbers.

## Assumptions and limits
- Unswept quarter-chord line for the tapered wings; straight trapezoids; MAC 3.5 in for all wings; spans as in Fig. 3.14; zero thickness in AVL (the plates are 4.3 % thick).
- The tunnel fit window is -4 to 6 deg; the up/down-leg slope difference averages 2.2 % of AVL's slope, and the approximate fit s.e. 1.1 %.
- Wall-correction constants K1 tau1 and tau2 are bounded, not looked up; delta is my image-vortex calculation for a centred wing.
