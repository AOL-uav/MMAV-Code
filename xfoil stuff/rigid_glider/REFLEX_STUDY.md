# Fixed-reflex branch (screening study)

This is a separate branch from the retained fixed-tail study in
`prototype_study.py`. It does not select a final aircraft or change the old
XFOIL/LLT database.

Run from the repository root, with `py`, `tools/avl/avl352.exe`, and
`tools/xfoil/xfoil.exe` available:

```powershell
py rigid_glider/reflex_study.py
py rigid_glider/reflex_xfoil.py --chord 0.10 --angle 3
py rigid_glider/reflex_xfoil.py --chord 0.10 --angle 3 --inviscid-check-only
py rigid_glider/reflex_xfoil.py --chord 0.10 --angle 3 --reverse-check-only
py rigid_glider/reflex_drag.py
```

Parameters and exploratory error ranges are in `reflex_config.json`. Outputs
are written to `results/runs/rigid_glider/reflex/` as CSV/JSON plus raw AVL and
XFOIL decks/logs. `reflex_geometry.py` creates two explicitly modeled halves
so a left/right lip-angle or incidence mismatch can be tested. A shared
mid-span SECTION marks inboard/outboard control boundaries, as described in
the AVL user primer. AVL's CONTROL changes panel-normal orientation aft of a
hinge; it is an inviscid flap approximation, not a resolved printed lip.

`reflex_xfoil.py` creates actual cosine-spaced NACA 00xx coordinates, rotating
the trailing portion upward about a straight hinge and leaving the forward
section symmetric. NACA 0004 is a thin symmetric *proxy*, not a literal flat
plate. The focused viscous runs use Ncrit=5 and XFOIL's default 160-panel PANE
re-paneling. Requested alpha is -3 to 8 degrees in 0.5-degree steps; absent
rows are convergence/coverage failures, never filled. The historical table
used Ncrit=5 but is not regenerated or modified. The optional inviscid check
isolates the effect of geometry from XFOIL's low-Re viscous prediction.

`reflex_drag.py` interpolates the focused polars by local AVL strip CL and
Reynolds number, adding area-weighted profile drag to AVL induced drag. It
refuses out-of-range/gapped polar data. Even when available, this additive
drag estimate does **not** feed XFOIL's viscous pitching moment back into AVL;
the AVL trim and static margin remain inviscid. This distinction is central
to interpreting the study.

## Extension study

`reflex_ext_run.py` and `reflex_ext_xfoil.py` add a static-margin sweep,
more spanwise splits, outboard-lip mismatch, multi-case and combined
tolerances, viscous polars for four more lips, and a re-run of the three
grid cases that found no trim. They import the modules above without
changing them and write only to `results/runs/rigid_glider/reflex_ext/`:

```powershell
py rigid_glider/reflex_ext_run.py repro
py rigid_glider/reflex_ext_run.py item1 item2 item3 item4 item6
py rigid_glider/reflex_ext_xfoil.py repro item5
py -m unittest rigid_glider/test_reflex_ext.py
```

`repro` must reproduce two rows of the original `static.csv` and one stored
XFOIL polar before the other stages are trusted. A finished stage is skipped
unless `--force` is given. Inputs are in `reflex_ext_config.json`. When the
original -2 to 8 degree alpha probe finds no Cm crossing, the extension
retries once over -4 to 20 degrees and flags the row
`alpha_range_extended`; such trims are near stall and should be read with
care.

`reflex_ext_predict.py` produces the model predictions for the section
validation experiment (larger lips, 15%/5, 15%/8 and 20%/10 degrees) into
`reflex_ext/predict/`; run `py -B rigid_glider/reflex_ext_predict.py geom
repro xfoil tables avl`. See section H of `REFLEX_RESULTS.md`.

`reflex_ext/reanalysis/` holds the audit of the original XFOIL results and
the matched-Cl re-analysis cited in `REFLEX_RESULTS.md`. Its helper scripts
are kept as a record of how the tables were produced; they were run from a
temporary directory and some contain that directory's absolute path, so they
are not runnable entry points as stored.

Sign convention: positive AVL/XFOIL Cm is nose-up; positive `angle_deg` is a
trailing-edge-up feature, implemented as a negative AVL control deflection.
Positive static margin is `(Xnp-Xcg)/MAC` for the main wing; negative Cm_alpha
is restoring. A positive Cl_beta in the raw AVL outputs is its published roll
coefficient convention, not a scalar measure of dynamic recovery.

The tested 1.2 m, AR 7, tapered wing and W/S=8 N/m² are comparison references
from the prototype study, not renewed MMAV size or mass requirements. The
±1° lip/±5 mm CG errors are exploratory, **not measured manufacturing
tolerances**. The lip's effect in a later reversed/reoriented samara wing is
not analyzed here.
