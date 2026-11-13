# Pure flying wing: coupled AVL + XFOIL feasibility, first pass

2026-10-11. First bounded pass at the question: does a small, tailless,
untwisted, zero-reflex, symmetric-section swept wing have a useful
positive-lift, statically stable trim once low-Reynolds section pitching
moment is included? Independently reviewed; the wording below follows that
review. Neither AVL nor XFOIL is validated here.

**Scope of every statement below:** wing loading 2–12 N/m², span
0.20–0.50 m, 46 planforms (AR 3/4.5/6, quarter-chord sweep 10/20/30/35 deg,
taper 0.4/0.7/1.0, dihedral 0/4/8 deg on five base shapes), NACA
0004 (thin proxy, not a flat plate)/0006/0008/0010, XFOIL Ncrit 5 and 9, CG
from 5% MAC aft to 20% MAC ahead of the AVL neutral point. The wing-loading
range has not been tied to a measured mass (see "Open").

## Run

```powershell
py -B rigid_glider/pure_wing_polars.py     # clean-section polar database
py -B rigid_glider/pure_wing_study.py      # coupled study, figures
py -B rigid_glider/pure_wing_reanalysis.py # tallies used in this report
py -B -m unittest rigid_glider/test_pure_wing.py rigid_glider/test_pure_wing_logic.py
```

Outputs: `results/runs/rigid_glider/pure_wing/` (`summary.json` holds the
assumption list; `reanalysis/reanalysis_summary.txt` the tallies quoted
here; `fig1`–`fig6` SVG figures, not yet visually checked). 21 of 21 tests
pass (`reanalysis/unittest_output.txt`).

## Model

AVL is linear for these wings, so it is run once per planform shape for the
spanwise loading, neutral point, induced drag and lateral derivatives. At a
trial CL, speed follows from wing loading, each strip gets its local Re and
local cl, and XFOIL supplies that section's quarter-chord moment, drag and
alpha at matched cl. Total pitching moment about the CG is

    Cm = −SM_AVL · CL + (chord-weighted section cm)

and trim is every CL where that is zero. Stability is judged by dCm/dCL both
along the constant-wing-loading glide line and at fixed speed; a trim counts
as stable only if both are negative. A second level (viscous decambering,
which lets the section lift curve change the span loading) was run on four
planforms.

Because Cm_AVL is exactly −SM·CL, **every positive-lift trim in this model
comes from XFOIL's clean-section moment and from nothing else.**

## Result

1. **AVL alone:** trim at CL = 0 for any positive static margin.
2. **Coupled:** stable trims appear from CL 0.15 (the lower edge of the band
   that was imposed, not a model finding) to about 0.38, with the CG 2.5 to
   10% MAC ahead of the AVL neutral point.
3. **Mechanism:** at low cl the upper-surface boundary layer is laminar to
   the trailing edge and the section moment rises nose-up, peaking at +0.011
   to +0.024 depending on section, Re and Ncrit
   (`reanalysis/max_section_cm_per_re.csv`). At a particular alpha the flow
   reattaches, lift jumps and the moment falls. The stable trims sit on that
   falling step: 57% of them at Ncrit 5 and 85% at Ncrit 9 have most of the
   wing area on the alpha interval where upper-surface transition first
   leaves the trailing edge. On the rising side below it the section slope
   is destabilising (median dcm/dcl +0.05), and up to about half of the
   smoother trims are statically unstable at slightly lower CL under a
   fixed-speed disturbance.
4. **Robustness.** "Smooth" means the study's flag for trims not on a
   steep (above 0.25 cl per degree) polar segment. That flag cuts through
   one feature and is not itself a robustness measure; the CG window is.

   | Requirement | Cases with a smooth stable trim | CG window, median / max |
   |---|---:|---|
   | Ncrit 5 only | 1390 of 3680 | 0.5% / 4.0% MAC (max 6.2 mm) |
   | Ncrit 9 only | 814 of 3680 | at scan resolution / 3.0% MAC (max 2.3 mm) |
   | Both, same CG | 398 of 3680 | at scan resolution / 1.0% MAC (max 0.3 mm) |

   "At scan resolution" is a single hit on a 0.5% MAC grid: the true width
   is below 1% MAC. Of 194 planform/section/span combinations surviving
   both Ncrit, 75 do so at one wing loading only. By section: NACA 0006 has
   339 of the 398, 0008 has 53, the 0004 proxy 6, 0010 none.
5. **CG at or aft of the AVL neutral point.** 5124 stable in-band trims
   exist with SM_AVL of 0, −2.5% and −5%. They are stable only through the
   viscous aft position of the section aerodynamic centre (section dcm/dcl
   negative in all of them, median −0.10). None of the smooth ones survives
   both Ncrit. The scan stops at −5%, so this class is truncated.
6. **Lateral (inviscid, no fin).** At CL 0.4, AR 4.5, taper 0.7: Cl_beta
   −0.016 to −0.063 per rad for sweep 10 to 35 deg, restoring. Cn_beta
   0.002 to 0.008 per rad, which is negligible, and dihedral reduces it
   (0.0069 at 0 deg to 0.0011 at 8 deg for sweep 30; slightly negative for
   one sweep-20 shape). A pure wing of this kind has essentially no
   directional stability in the model.

**Reading:** a positive-lift trim may exist for a clean symmetric pure wing,
but in this model it depends on where a laminar separation closes, has a CG
window of about a millimetre or less when both transition settings must
agree, and comes with no yaw stability. That is not a defensible design
basis from the model alone. It is also not evidence the pure wing cannot
work: the deciding quantity is unmeasured.

## Scale

Glide speed depends on wing loading and CL only. Span sets chord and
Reynolds number. Implied all-up mass in grams (AR 4.5; `mass_table_g.csv`
has AR 3 and 6):

| Span | 2 | 4 | 6 | 8 | 12 N/m² |
|---|---:|---:|---:|---:|---:|
| 0.20 m | 1.8 | 3.6 | 5.4 | 7.3 | 10.9 |
| 0.30 m | 4.1 | 8.2 | 12.2 | 16.3 | 24.5 |
| 0.40 m | 7.3 | 14.5 | 21.8 | 29.0 | 43.5 |
| 0.50 m | 11.3 | 22.7 | 34.0 | 45.3 | 68.0 |

Reynolds number and speed versus span are in `scale_table.csv` and
`fig5a`–`fig5c`.

## Limitations

- Section data are applied streamwise with no sweep correction. The pitch
  result is therefore nearly independent of sweep (median trim-CL spread
  0.003 across 10–35 deg). At 35 deg the normal-section cl is about 1.5
  times the streamwise cl, which would move the reattachment onset by a
  first-order amount. This is a model limitation, not a finding.
- The decambering level agrees with the one-way level on four planforms,
  but the both-Ncrit smooth set there is one to six cells. Too thin to
  confirm or refute anything.
- XFOIL polars at these Re are irregular: at each Re from 10k to 100k, 5 to
  7 of the 8 polars have a CL jump above 0.1 within 0.5 deg.
- Paneling. The database uses XFOIL's default 160 panels. For the reflex
  study at Re 60k–100k the clean-section moment slope is paneling-sensitive,
  less so toward lower Re (`REFLEX_RESULTS.md`, 2026-10-11 addendum). A 160
  versus 240-panel check on NACA 0006 and 0008 at Re 20k and 40k
  (`reflex_ext/gaps2/item2_summary.txt`) found the reattachment-onset alpha
  unchanged at the 0.5 deg grid in the seven cases where it could be
  compared, the cl on the attached side of it up by 0.004–0.008, the moment
  peak down by 0.0005–0.0014 and the moment at matched cl changed by at
  most 0.0017. So at these Re the feature the trims depend on is stable to
  that one refinement step. Not tested: more than 240 panels, the 0.25 deg
  alpha resolution, other sections, Re 10k–15k.
- No spanwise boundary-layer flow, tip or centre-section viscous effects,
  dynamics, or hysteresis.

## What would settle it

A measured clean-section lift and quarter-chord moment at Re 15k–40k. The
quantity to resolve is the cl at which the upper surface reattaches, and
whether it differs between increasing and decreasing alpha, more than the
moment magnitude. `SECTION_EXPERIMENT.md` has the rig requirements; its
matrix must be re-derived for this scale.

## Open

- Realistic all-up mass or wing loading at 0.2–0.4 m span (Kai). If real
  articles are heavier than the table above, the aircraft sits at higher
  speed and Re than most of this grid; the wing-loading range and the aft
  end of the CG scan should then be extended before the reflex step, which
  would otherwise inherit the same grid.
- Minimal reflex on this model: not started.
