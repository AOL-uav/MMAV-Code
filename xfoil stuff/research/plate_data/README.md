# Measured flat-plate wing data - Ananda (2012) thesis, UIUC

Retrieved 2026-10-12. All CL/CD/CM values are copied from printed TABLES (no figure reading, no interpolation).

## Source
- G. K. Ananda Krishnan, "Aerodynamic performance of low-to-moderate aspect ratio wings at low Reynolds
  numbers", M.S. thesis, Univ. of Illinois at Urbana-Champaign, Dec 2012. This is Ref. [4] of
  Ananda, Sukumar & Selig, Aerosp. Sci. Technol. 42 (2015) 392-406.
  - Record: https://www.ideals.illinois.edu/items/42267 (handle https://hdl.handle.net/2142/42319)
  - PDF:    https://www.ideals.illinois.edu/items/42267/bitstreams/127369/data  -> `Ananda_2012_MS_thesis_UIUC.pdf` (235 pp)
  - Appendix B "Tabulated Drag Polar Data" (alpha, CL, CD): PDF pp. 136-165 (printed 120-149)
  - Appendix C "Tabulated Lift and Moment Data" (alpha, CL, CM): PDF pp. 166-195 (printed 150-179)
  - PDF page = printed page + 16.
- `ast2015.pdf`: the journal paper (https://m-selig.web.engr.illinois.edu/pubs/AnandaSukumarSelig-2015-AeroSciTech-LowReWings.pdf).
- `aiaa2012.pdf`: AIAA 2012-3026 conference version (downloaded, NOT read or used).

## Files
- `ananda2012_flatplate_AR3_taper1.csv` - AR 3, taper 1 (model AR3-l100-FP1, thesis Fig. 5.7), 4 runs x 90 points:
  Re 60k (run 0383ar), 80k (0384ar), 100k (0386ar), 120k (0385ar). PDF pp. 153-155 (CL,CD) and 183-185 (CM).
- `ananda2012_flatplate_all_wings.csv` - all 10 flat-plate wings (AR 2,3,4 at taper 0.5/0.75/1; AR 5 taper 1), 40 runs, 3482 rows.
- Columns: alpha_deg, CL, CD, CM_c4, Re, AR, source, how_obtained, uncertainty, then extras taper_ratio, run,
  sweep (increasing/decreasing alpha leg of the same run), seq (row order in the table).
- Re is the value printed in the table header (e.g. 59999.4), not the nominal.

## How the CSV was made
Text extracted with pypdf; Appendix B and C parsed separately and merged row by row. For every row alpha and CL
agree exactly between the two appendices (asserted). No unparsed lines. Spot-check against the PDF is still advisable.

## Things to know before comparing with CFD
- No point is at exactly alpha = 0, 2, 4 ... deg. Angles are wall-corrected, so they land at e.g. 3.48 and 4.54 deg.
  Nothing was interpolated; interpolate yourself if needed and say so.
- Each run is an up-sweep followed by a down-sweep (about -15 to +30 deg). No hysteresis reported for the flat plates,
  so the two legs give two independent readings near each angle; their difference is a fair repeatability indicator
  (CD differs by up to ~0.004 near 4 deg at Re 60k).
- CM is about the quarter chord (wing mounted and moment measured at c/4; thesis p. 32 PDF, Table 3.1 "Mc/4").
  It is slightly POSITIVE at low alpha for this wing.
- Data ARE corrected for tunnel walls (thesis Sec. 3.8, PDF pp. 57-59): solid blockage, Maskell wake blockage,
  streamline curvature / boundary-induced upwash applied to alpha, CD and CM. Buoyancy neglected.
  Sting tare subtracted at each Re; sting interference judged negligible from a mirror-sting test (done on the
  Wortmann wing at Re 30k and 60k, not on this plate).
- Mounting: wing vertical in the test section, cantilevered from a sting attached to one wing tip at the quarter
  chord, sting going to a 3-component balance on the tunnel ceiling (thesis PDF p. 32, 54; AST paper p. 394).
  Test section 2.8 x 4.0 ft, 8 ft long.
- Geometry: mean aerodynamic chord 3.5 in, span 10.5 in for AR 3 taper 1; t/c 4.3% (0.150 in), 5:1 elliptical LE,
  10:1 elliptical TE (thesis Fig. 3.13, 3.14, PDF pp. 50-51). Clean (no trip).
  NOTE: the thesis ABSTRACT says "chord length of 3 in and a wingspan of 7 to 17.5 in"; the body text, Table 5.1
  (cmac 0.29167 ft) and the spans all say 3.5 in. 3.5 in is taken as correct.
- Uncertainty (thesis Sec. 3.9, PDF pp. 63-64): average load-check bias lift 3.28%, drag 2.66%, moment 4.56%;
  example-case coefficient uncertainties CL 3.33%, CD 2.72%, CM 4.62% (computed for the Wortmann wing at Re 90k,
  alpha 5 deg - not for this plate); alpha 0.02 deg. The thesis also warns that CDmin scatter is within
  uncertainty and that sting-tare setup differences may offset CD (PDF p. 121).
- Three runs are shorter in the thesis tables themselves: AR3 taper 0.75 Re 80k (0372ga, 30 rows),
  AR4 taper 1 Re 120k (0399cs, 62 rows), AR5 taper 1 Re 100k (0423ga, 60 rows).

## Not found / not stated
- Wing-tip shape (square cut vs rounded): NOT stated in the text of either document. Fig. 3.14 (PDF p. 51) is an
  isometric drawing of the wings; it could not be rendered here to inspect. Open item.
- Lift-curve slope, zero-lift angle, CM slope: no numeric values are stated for this wing. Slopes are only plotted:
  thesis Fig. 6.6 (PDF p. 126) and Fig. 6.7; AST paper Fig. 17 and 18 (pp. 402). Not read off (no PDF renderer available).
- Aerodynamic centre: AST paper Fig. 19(b) plots x_ac vs alpha for AR 3 taper 1 at Re 100,000 only; text (p. 402-403)
  says that for the rectangular wings x_ac at low alpha is "approximately 0.20 to 0.27" and moves towards ~0.5 near
  CLmax. No value specific to Re 60,000.

## Derived here, NOT from the source (least-squares line through the tabulated points, both sweep legs)
Labelled as our own post-processing; the source's fit range is unspecified so these need not equal the plotted slopes.

| Re (run) | range deg | n | CL_alpha /deg | alpha_0 deg | CM_alpha /deg | dCM/dCL |
|---|---|---|---|---|---|---|
| 60k (0383ar)  | -6..6 | 24 | 0.0607 | -0.07 | 0.00255 | 0.042 |
| 60k (0383ar)  |  0..6 | 12 | 0.0622 |  0.02 | 0.00228 | 0.037 |
| 80k (0384ar)  | -6..6 | 24 | 0.0634 | -0.08 | 0.00208 | 0.033 |
| 100k (0386ar) | -6..6 | 24 | 0.0661 | -0.10 | 0.00170 | 0.026 |
| 120k (0385ar) | -6..6 | 24 | 0.0682 | -0.16 | 0.00132 | 0.019 |

## AR 3, taper 1, Re 60k (run 0383ar) - tabulated points bracketing the requested angles
| alpha | CL | CD | CM_c4 | leg |
|---|---|---|---|---|
| -0.43 | -0.028 | 0.0169 | -0.0009 | up |
| 0.46 | 0.030 | 0.0183 | 0.0026 | up |
| 0.37 | 0.031 | 0.0152 | 0.0029 | down |
| 1.26 | 0.069 | 0.0183 | 0.0049 | up |
| 2.59 | 0.147 | 0.0216 | 0.0087 | up |
| 2.33 | 0.142 | 0.0184 | 0.0088 | down |
| 3.48 | 0.203 | 0.0258 | 0.0127 | up |
| 4.54 | 0.277 | 0.0330 | 0.0124 | up |
| 4.28 | 0.276 | 0.0289 | 0.0118 | down |
| 3.30 | 0.198 | 0.0227 | 0.0119 | down |
| 5.52 | 0.341 | 0.0421 | 0.0147 | up |
| 6.50 | 0.402 | 0.0504 | 0.0164 | up |
| 6.32 | 0.400 | 0.0460 | 0.0148 | down |
| 7.57 | 0.472 | 0.0669 | 0.0178 | up |
| 8.63 | 0.535 | 0.0868 | 0.0104 | up |
| 8.37 | 0.532 | 0.0812 | 0.0107 | down |
| 9.52 | 0.594 | 0.1085 | -0.0068 | up |
| 10.58 | 0.630 | 0.1292 | -0.0277 | up |
| 10.41 | 0.635 | 0.1262 | -0.0258 | down |
