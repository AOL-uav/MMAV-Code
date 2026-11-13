# MAV analysis workspace

> **Current state is in [`CURRENT.md`](CURRENT.md)** (flight phase, 2026-10-21). The direction notes below date from 2026-10-11 and are kept for history.

This folder preserves the previous XFOIL and lifting-line work while keeping new runs separate from historical data. The existing LLT ranks airfoils and estimates wing performance; it does not establish trim or passive flight stability.

## Layout

| Path | Purpose |
| --- | --- |
| `src/polars/` | XFOIL batch generation and polar processing source |
| `src/wing/` | LLT analysis source |
| `scripts/plots/` | Plotting scripts |
| `data/polars/` | Historical combined XFOIL datasets; preserve these inputs |
| `data/polars/generated/` | New batch-generated polar files |
| `results/legacy/` | Original wing-analysis outputs and figures |
| `results/runs/` | Outputs from newly run analysis programs |
| `tools/xfoil/` | Bundled XFOIL 6.99 for Windows |
| `tools/bin/` | Rebuilt active executables |
| `tools/avl/` | Official MIT AVL 3.52 executable and provenance |
| `rigid_glider/` | Isolated passive flying-wing baseline, manual sensitivities, and documentation |
| `archive/experiments/` | Earlier drivers and temporary files, retained for reference |
| `archive/binaries/` | Original compiled executables, retained without changing them |

## Current workflows (Windows, PowerShell)

The new passive rigid-wing baseline is `py rigid_glider/run.py`; see
[`rigid_glider/README.md`](rigid_glider/README.md). It uses the historical
XFOIL table and AVL, leaving the legacy LLT source and results untouched.
The bounded simplicity/robustness screen is
`py rigid_glider/robustness_study.py`; its interpretation is in
[`rigid_glider/ROBUSTNESS_REPORT.md`](rigid_glider/ROBUSTNESS_REPORT.md).
**Direction as of 2026-10-11 (Kai):** no architecture is selected. The order
of preference is pure flying wing, then flying wing with a minimal
reflex/lip, then a small tail as the fallback, at a much smaller scale than
the 1.2 m reference (roughly 0.2-0.5 m span, to be chosen from the model).
The tail work below is a developed fallback branch, not the chosen aircraft;
statements further down that the tail branch is "the chosen first flight
prototype" record the superseded 2026-10-09 decision.

The three-way comparison (pure wing / reflex lip / small tail) at 0.40 m
span and the modular test article are in
[`rigid_glider/ARCH_COMPARISON.md`](rigid_glider/ARCH_COMPARISON.md); start
there. Figures: `results/runs/rigid_glider/figures.html`.

The coupled AVL + XFOIL pure-wing feasibility study (first pass) is in
[`rigid_glider/PURE_WING_RESULTS.md`](rigid_glider/PURE_WING_RESULTS.md),
and a draft section-measurement design in
[`rigid_glider/SECTION_EXPERIMENT.md`](rigid_glider/SECTION_EXPERIMENT.md).

The dimensioned tail-branch test article and its 18 releases are in
[`rigid_glider/BUILD_ARTICLE.md`](rigid_glider/BUILD_ARTICLE.md).
The adjustable-tail/fin prototype sizing map and the fuller follow-on
drop-test matrix are in
[`rigid_glider/PROTOTYPE_REPORT.md`](rigid_glider/PROTOTYPE_REPORT.md).
The fixed trailing-edge reflex (lip) comparison and its extension are run
with `py rigid_glider/reflex_study.py` and `py rigid_glider/reflex_ext_run.py`;
method and commands are in
[`rigid_glider/REFLEX_STUDY.md`](rigid_glider/REFLEX_STUDY.md) and results in
[`rigid_glider/REFLEX_RESULTS.md`](rigid_glider/REFLEX_RESULTS.md). Outputs
go to `results/runs/rigid_glider/reflex/` and `reflex_ext/`. (The
2026-10-09 choice of the tail branch as first flight prototype is
superseded; see "First test article" below.)

**First test article (decided by Kai 2026-10-11):** planform P07 (400 mm
span, AR 4.5, taper 0.5, 25 deg sweep), tip fins for the clean wing with a
centre fin as backup, built first as a sheet flat-plate wing on a printed
centre piece and then as a printed NACA 0006 wing. Test order: clean wing,
lip, tail. Specification, cut list and test card:
[`rigid_glider/ARTICLE1.md`](rigid_glider/ARTICLE1.md); drawings and 1:1
templates: `results/runs/rigid_glider/article1/index.html`. A plain-language
summary of the findings is `results/runs/rigid_glider/findings.html`.

A flat-plate section could not be modelled: XFOIL's plate polars depend on
the trailing-edge treatment and disagree with published measurements
(`results/runs/rigid_glider/plate/summary.txt`,
`results/runs/rigid_glider/plate_te_diag/diag_summary.txt`).

Build from the project root with MSYS2 UCRT64 `g++`:

```powershell
g++ -std=c++17 -O2 -pthread src/wing/LLTparalleler.cpp -o tools/bin/LLTparalleler.exe
g++ -std=c++17 -O2 src/wing/megaWingAnalyzer.cpp -o tools/bin/megaWingAnalyzer.exe
g++ -std=c++17 -O2 src/polars/polarParalleler.cpp -o tools/bin/polarParalleler.exe
g++ -std=c++17 -O2 src/polars/megaPolarSorter.cpp -o tools/bin/megaPolarSorter.exe
g++ -std=c++17 -O2 src/polars/mergeMegapolar.cpp -o tools/bin/mergeMegapolar.exe
```

Run the historical LLT bulk case:

```powershell
./tools/bin/LLTparalleler.exe --bulk --threads 4
```

It reads `data/polars/Currentmegapolar.dat` and writes `results/runs/llt/`. The executable resolves the project root from its location in `tools/bin`, so it can also be launched from another working directory. `--single` is retained but currently writes invalid sentinel rows for foils it did not load; do not interpret that output as a one-foil summary.

The airfoil-pair driver is `tools/bin/megaWingAnalyzer.exe`; it writes to `results/runs/airfoil_pairs/`. Its optional per-combination plots call `scripts/plots/llt_plot.py`.

The XFOIL batch driver is `tools/bin/polarParalleler.exe`. Its current source enables only three foils and a large Reynolds-number sweep, and a full run takes substantial time. It writes to `data/polars/generated/megapolar.dat`, not to the historical datasets. Review its hard-coded grid in `src/polars/polarParalleler.cpp` before launching a full batch. The original executables in `archive/binaries/` retain their old path assumptions and are not the current entry points.

Plotting examples:

```powershell
py scripts/plots/llt_plot.py results/legacy/wingAnalysis/glider_wingpolar_llt.dat results/legacy/wingAnalysis/glider_llt_bestalpha_stations.dat results/runs/glider_lltPlot.png
py scripts/plots/plot_polar.py results/legacy/test_polar_output.dat results/runs/test_polar_dashboard.png
```

Run `LDplotter.py` or `Cm_plotter.py` from `scripts/plots/` to read the historical broad polar file; their new output goes to `results/runs/plots/`. Install their dependencies with `py -m pip install -r scripts/plots/requirements.txt`. They were not installed in the audit environment, so the plotting commands have not been run to completion here.

## Data and interpretation

`Currentmegapolar.dat` contains the broad historical four-digit NACA dataset. `megapolar_A.dat` and `megapolar_B.dat` are its source portions; the smaller `megapolar.dat` contains three foils. The archived `results/legacy/wingAnalysis/glider_metrics.csv` is the reference output for the 0.9 m span, AR 6, zero-sweep, Re 80,000 LLT bulk case. Preserve it for comparison; it is not a verified stable-glider design.

The repository currently has no Git metadata, automated aerodynamic validation suite, or CAD/CFD case. The new passive-flight model is an inviscid static screening workflow, not validated flight dynamics. Keep historical assumptions with the archived results.
