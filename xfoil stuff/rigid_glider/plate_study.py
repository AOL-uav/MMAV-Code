"""Coupled architecture model with flat-plate sections on candidate wing P07 (AR 4.5, taper 0.5, sweep 25 deg).

Writes ONLY under results/runs/rigid_glider/plate/.  Reads (never writes) arch_compare_v2/models_final.json (the
final AVL deck: 20x32 panels, 3 deg dihedral, 6 % centre fin; tail case tail + fin) and arch_compare_v2/scans_v2.json
(the stored NACA 0006 uncensored scans, turned into window rows by the existing V2.window_rows2; NACA 0006 is not
recomputed).  Plate section data come from plate_polars.PlateTables (plate/polars/plate_polar_database.csv).

Same machinery as arch_v2_study.work: V2.evaluators -> V2.build_curve_ev -> V2.scan_curve2 -> V2.window_rows2,
uncensored SM scan -0.15..+0.60 step 0.005, CL band 0.2..0.5, both Ncrit, 160 panels only.

  py -B rigid_glider/plate_study.py scan | tables | figs | all
"""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import csv
import json
import time
from multiprocessing import Pool
from pathlib import Path

import arch_model as A
import arch_v2_model as V2
import pure_wing_model as m
import pure_wing_polars as pwp
import pure_wing_v2_model as v2
import plate_polars as pp
import plot_svg

OUT = pp.PLATE_OUT
V2DIR = V2.OUT
B = 0.40
WING = "P07"
WS_LIST = [3, 6, 8, 12, 16]
NACA_WS = [6, 8, 12, 16]                  # loadings present in the existing NACA 0006 study (3 N/m2 is not)
SECS = ["PLATE_43", "PLATE_30"]
NACA = "NACA_0006"
PAN = 160
TAIL_STN = 2.0
TAIL_INC = -3.0
LIP_BEST = [(0.20, 10.0), (0.15, 8.0)]
BAND = V2.BAND
COV_FRAC_MIN = 0.5      # per-Ncrit fraction of the 31 band CL points with min(cov, derivative cov) >= 0.9 below which a cell is "insufficient coverage"

# (label, kind, cfg, u, form key in the scan, sections run)
ARCH = [("pure", "pure", None, 0.0, "coupled", SECS)]
for _cf, _ang in LIP_BEST:
    _lab = f"lip {int(_cf * 100)}/{_ang:g} outboard"
    ARCH += [(f"{_lab} (a)", "lip", [_cf, "outer"], _ang, "coupled", ["PLATE_43"]),
             (f"{_lab} (b)", "lip", [_cf, "outer"], _ang, "replace", ["PLATE_43"]),
             (f"{_lab} (c)", "lip", [_cf, "outer"], _ang, "replace_k", ["PLATE_43"])]
ARCH += [("tail 10% AVL lift", "tail", [0.10, TAIL_STN], TAIL_INC, "coupled", SECS),
         ("tail 10% 0.59 lift (5% area model)", "tail", [0.05, TAIL_STN], TAIL_INC, "coupled", SECS)]

_S = {}


def load_final_models() -> dict:
    return json.loads((V2DIR / "models_final.json").read_text())


def _init():
    _S["models"] = load_final_models()
    _S["tabs"] = pp.PlateTables()


def forms_for(kind):
    return ["avl_only", "coupled"] + (["replace", "replace_k"] if kind == "lip" else [])


def band_cov_frac(curve) -> float:
    pts = [p for p in curve if BAND[0] - 1e-9 <= p["cl"] <= BAND[1] + 1e-9]
    return sum(1 for p in pts if min(p["cov"], p["covd"]) >= v2.COV_MIN) / len(pts)


def work(job):
    kind, sec, cfg, u = job
    M = _S["models"][V2.model_key(kind, WING, cfg)]
    recs = []
    for ws in WS_LIST:
        for form in forms_for(kind):
            for nc in ([None] if form == "avl_only" else [5, 9]):
                ev, ti_fn, avl = V2.evaluators(M, B, u, kind, _S["tabs"], sec, form, nc, PAN, cfg, ws)
                curve, nonconv = V2.build_curve_ev(ev, ti_fn, avl_only=avl)
                sets, pts = V2.scan_curve2(curve)
                recs.append(dict(deck="final", kind=kind, wing=WING, section=sec, cfg=cfg, u=u, ws=ws, form=form, ncrit=nc,
                                 panels=None if avl else PAN, sets=sets, pts=pts, nonconv=nonconv,
                                 band_cov=None if avl else band_cov_frac(curve)))
    return recs


def all_jobs():
    jobs = []
    for _lab, kind, cfg, u, _form, secs in ARCH:
        for s in secs:
            j = (kind, s, cfg, u)
            if j not in jobs:
                jobs.append(j)
    return jobs


def stage_scan():
    t0 = time.time()
    jobs = all_jobs()
    with Pool(6, initializer=_init) as pool:
        res = pool.map(work, jobs, chunksize=1)
    recs = [r for x in res for r in x]
    (OUT / "scans_plate.json").write_text(json.dumps(recs))
    print(f"scan: {len(jobs)} jobs, {len(recs)} records in {time.time() - t0:.0f}s")


# ------------------------------------------------------------------ window rows
def rows_from(recs, models) -> dict:
    rows = V2.window_rows2(recs, {"final": models})
    return {(r["kind"], r["section"], r["cfg1"], r["cfg2"], r["u"], r["ws"], r["form"], r["panels"], r["tier"], r["mode"]): r for r in rows}


def cell(idx, kind, sec, cfg, u, ws, form, tier="both", mode="any"):
    c1, c2 = (None, None) if cfg is None else (cfg[0], cfg[1])
    pan = None if form == "avl_only" else PAN
    return idx.get((kind, sec, c1, c2, u, ws, form, pan, "avl" if form == "avl_only" else tier, mode))


def fields(r, covs=None):
    """-> dict(width_mm, start_mm, end_mm, cl_lo, cl_hi, status)."""
    if covs is not None and min(covs) < COV_FRAC_MIN:
        return dict(width_mm="", start_mm="", end_mm="", cl_lo="", cl_hi="", status="insufficient coverage")
    if r is None:
        return dict(width_mm="", start_mm="", end_mm="", cl_lo="", cl_hi="", status="not computed")
    if not r["n_points"]:
        return dict(width_mm=0.0, start_mm="", end_mm="", cl_lo="", cl_hi="", status="no window")
    return dict(width_mm=round(r["width_mm"], 3), start_mm=round(r["cg_mm_lo"], 2), end_mm=round(r["cg_mm_hi"], 2),
                cl_lo=round(r["cl_lo"], 3), cl_hi=round(r["cl_hi"], 3),
                status="edge-censored" if (r["touches_lo_edge"] or r["touches_hi_edge"]) else "ok")


def stage_tables():
    plate_recs = json.loads((OUT / "scans_plate.json").read_text())
    models = load_final_models()
    pidx = rows_from(plate_recs, models)
    t0 = time.time()
    naca_recs = [r for r in json.loads((V2DIR / "scans_v2.json").read_text())
                 if r["deck"] == "final" and r["wing"] == WING and r["section"] == NACA and r["panels"] in (None, PAN)]
    nidx = rows_from(naca_recs, models)
    covmap = {}
    for r in plate_recs:
        if r["band_cov"] is not None:
            covmap.setdefault((r["kind"], r["section"], json.dumps(r["cfg"]), r["u"], r["ws"], r["form"]), {})[r["ncrit"]] = r["band_cov"]
    out = []
    for lab, kind, cfg, u, form, secs in ARCH:
        for ws in WS_LIST:
            row = dict(architecture=lab, form_key=form, u_deg=u, ws_N_m2=ws)
            for tag, sec, idx in (("plate43", "PLATE_43", pidx), ("plate30", "PLATE_30", pidx), ("naca0006", NACA, nidx)):
                if sec not in secs + [NACA]:
                    for tier in ("any", "strict"):
                        row.update({f"{tag}_{tier}_{k}": "" for k in ("width_mm", "start_mm", "end_mm", "cl_lo", "cl_hi")})
                        row[f"{tag}_{tier}_status"] = "not run"
                    row[f"{tag}_band_cov_N5"] = row[f"{tag}_band_cov_N9"] = ""
                    continue
                if sec == NACA and ws not in NACA_WS:
                    for tier in ("any", "strict"):
                        row.update({f"{tag}_{tier}_{k}": "" for k in ("width_mm", "start_mm", "end_mm", "cl_lo", "cl_hi")})
                        row[f"{tag}_{tier}_status"] = "not in existing study"
                    row[f"{tag}_band_cov_N5"] = row[f"{tag}_band_cov_N9"] = ""
                    continue
                cm_ = covmap.get((kind, sec, json.dumps(cfg), u, ws, form)) if sec != NACA else None
                covs = None if cm_ is None else [cm_[5], cm_[9]]
                for tier, mode in (("any", "any"), ("strict", "offti")):
                    r = cell(idx, kind, sec, cfg, u, ws, form, "both", mode)
                    row.update({f"{tag}_{tier}_{k}": v for k, v in fields(r, covs).items()})
                row[f"{tag}_band_cov_N5"] = "" if cm_ is None else round(cm_[5], 3)
                row[f"{tag}_band_cov_N9"] = "" if cm_ is None else round(cm_[9], 3)
            # AVL-only (inviscid, section independent) window from the same deck, for reference
            av = fields(cell(pidx, kind, "PLATE_43", cfg, u, ws, "avl_only"))
            row.update({f"avl_only_{k}": v for k, v in av.items()})
            out.append(row)
    pp.g.write_csv(OUT / "comparison_P07.csv", out)
    # AVL-only reference per architecture (section independent), for the summary
    print(f"comparison_P07.csv: {len(out)} rows ({time.time() - t0:.0f}s for the NACA rows)")
    return out


# ------------------------------------------------------------------ figures
def figs():
    # --- fig 1: section cm(c/4) vs cl.  NACA 0006 viscous (Ncrit 5, two Re) and inviscid, plate 4.3 % inviscid;
    #     plate VISCOUS series are plotted too but have no converged points (empty series are dropped by plot_svg)
    visc, inv = pwp.read_database()
    pol = pp.load_polars()
    series = []
    for re in (30000, 80000):
        n = [(r["cl"], r["cm"]) for r in visc.get(("NACA_0006", 5, re), []) if r["alpha"] >= 0 and r["cl"] >= 0]
        series.append((f"NACA 0006 Re {re // 1000}k", n))
        series.append((f"plate 4.3% Re {re // 1000}k (none converged)",
                       [(r["cl"], r["cm"]) for r in pol.get(("PLATE_43", 0.0, 0.0, re, 5), []) if r["alpha"] >= 0 and r["cl"] >= 0]))
    series.append(("NACA 0006 inviscid", [(r["cl"], r["cm"]) for r in inv["NACA_0006"] if r["alpha"] >= 0]))
    series.append(("plate 4.3% inviscid", [(r["cl"], r["cm"]) for r in pol[("PLATE_43", 0.0, 0.0, 0, 0)] if r["alpha"] >= 0]))
    series = [x for x in series if x[1]][:5]
    plot_svg.plot(OUT / "fig1_section_cm_vs_cl.svg", "Section cm (c/4) vs cl, alpha >= 0: NACA 0006 vs plate 4.3% (plate viscous: none converged)",
                  "section cl", "cm about c/4 (nose-up +)", series, zero_line=True)
    # --- fig 2: CG window width by architecture, P07, 8 and 16 N/m2.  Plate cells are all 'insufficient coverage' and are not plotted.
    with (OUT / "comparison_P07.csv").open(encoding="utf-8") as s:
        rows = list(csv.DictReader(s))
    labs = [a[0] for a in ARCH]
    ser = []
    for ws in (8, 16):
        for col, name in (("naca0006_any_width_mm", "NACA 0006"), ("plate43_any_width_mm", "plate 4.3%")):
            pts = []
            for i, lab in enumerate(labs, 1):
                r = next(x for x in rows if x["architecture"] == lab and int(x["ws_N_m2"]) == ws)
                if r[col] != "":
                    pts.append((float(i), float(r[col])))
            if pts:
                ser.append((f"{name} {ws} N/m2", pts))
    ser.append(("AVL-only (inviscid)", [(float(i), float(next(x for x in rows if x["architecture"] == lab and int(x["ws_N_m2"]) == 8)["avl_only_width_mm"]))
                                        for i, lab in enumerate(labs, 1)]))
    key = "; ".join(f"{i}={lab}" for i, lab in enumerate(labs, 1))
    plot_svg.plot(OUT / "fig2_cg_window_by_architecture.svg", "P07 CG window (both Ncrit, any tier): NACA 0006; plate = insufficient coverage, not plotted",
                  "architecture index, key in fig2_key.txt", "window width, mm", ser, zero_line=True)
    (OUT / "fig2_key.txt").write_text(key + chr(10))


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    fn = {"scan": stage_scan, "tables": stage_tables, "figs": figs}
    for s in (list(fn) if what == "all" else [what]):
        fn[s]()


if __name__ == "__main__":
    main()
