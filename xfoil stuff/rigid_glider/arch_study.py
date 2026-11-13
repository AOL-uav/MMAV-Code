"""Architecture comparison driver (pure / lip / tail) on the three candidate wings.  Writes only to
results/runs/rigid_glider/arch_compare/.

py -B rigid_glider/arch_study.py avl | lipcmp | core | windows | lateral | tables | figs | all
"""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import csv
import json
import math
import statistics
import time
from collections import defaultdict
from multiprocessing import Pool
from pathlib import Path

import arch_model as A
import pure_wing_model as m
import pure_wing_v2_model as v2
import pure_wing_study as st

OUT = A.OUT
B = 0.40
G = 9.80665
WINGS = {"P09": (6.0, 15, 0.5), "P12": (6.0, 35, 1.0), "P07": (4.5, 25, 0.5)}
SECTIONS = ["NACA_0006", "NACA_0008"]
WS_LIST = [6, 8, 12, 16, 24]
LIP_CFS = [0.10, 0.15, 0.20]
LIP_ANGLES = {0.10: (3.0, 6.0), 0.15: (5.0, 8.0), 0.20: (5.0, 10.0)}
PLACEMENTS = ["full", "outer"]
TAIL_AREAS = [0.08, 0.10, 0.12]
TAIL_INCS = [-2.0, -3.0, -4.0]
TAIL_STATIONS = [1.5, 2.0]
BAND = (0.2, 0.5)
STEP = 0.005
SM_SCAN = [round(-0.10 + STEP * i, 4) for i in range(61)]
SM_ROOTS = [round(-0.10 + 0.025 * i, 4) for i in range(13)]
NCRITS = (5, 9)
PANELS = (160, 240)
MODES = v2.MODES
THRESH_MM = 3.0           # exploratory window threshold (label: exploratory)


def mkey(kind, wing, *cfg):
    return "|".join([kind, wing, *[f"{c:g}" if isinstance(c, float) else str(c) for c in cfg]])


# ------------------------------------------------------------------ stage: avl models
def stage_avl():
    t = time.time()
    models = {}
    wk = OUT / "avl"
    for wid, (ar, sw, tp) in WINGS.items():
        models[mkey("pure", wid)] = A.build_pure(ar, sw, tp, wk / f"pure_{wid}")
        for cf in LIP_CFS:
            for pl in PLACEMENTS:
                models[mkey("lip", wid, cf, pl)] = A.build_lip(ar, sw, tp, cf, pl, wk / f"lip_{wid}_{int(cf*100)}_{pl}")
        for ta in TAIL_AREAS:
            for st_ in TAIL_STATIONS:
                models[mkey("tail", wid, ta, st_)] = A.build_tail(ar, sw, tp, ta, st_, wk / f"tail_{wid}_{int(ta*100)}_{st_:g}")
    (OUT / "avl_models.json").write_text(json.dumps(models))
    worst = max((abs(M["lin_check"]["cl_avl"] - M["lin_check"]["cl_pred"]) for M in models.values()), default=0)
    print(f"{len(models)} AVL linear models in {time.time()-t:.0f}s; max |CL(check) - linear prediction| = {worst:.5f}")


def load_models():
    return json.loads((OUT / "avl_models.json").read_text())


# ------------------------------------------------------------------ stage: AVL vs XFOIL inviscid lip moment
def stage_lipcmp():
    models = load_models()
    tabs = A.ArchTables()
    rows = []
    for wid in WINGS:
        for cf in LIP_CFS:
            for pl in PLACEMENTS:
                M = models[mkey("lip", wid, cf, pl)]
                ls = [s for s in M["strips"] if s["inlip"]]
                w = sum(s["c"] * s["a"] for s in ls)
                k_avl = sum(s["K"] * s["c"] * s["a"] for s in ls) / w                 # AVL strip cm_c/4 per deg (chord-area weighted)
                d_avl = sum(s["D"] * s["c"] * s["a"] for s in ls) / w                 # AVL strip dcl per deg at alpha=0
                k_root, k_tip = ls[0]["K"], ls[-1]["K"]
                for sec in SECTIONS:
                    for ang in LIP_ANGLES[cf]:
                        for pan in PANELS:
                            inv_b = tabs.inv_curve(sec, cf, ang, pan)
                            inv_c = tabs.inv_curve(sec, 0.0, 0.0, pan)
                            for cl in (0.15, 0.25, 0.35):
                                cb, cc = inv_b.cm_at(cl), inv_c.cm_at(cl)
                                if cb is None or cc is None:
                                    continue
                                x_per_deg = (cb - cc) / ang
                                rows.append(dict(wing=wid, section=sec, cf=cf, angle=ang, placement=pl, panels=pan, cl=cl,
                                                 xfoil_inviscid_dcm_per_deg=x_per_deg, avl_dcm_per_deg_area_weighted=k_avl,
                                                 avl_dcm_root=k_root, avl_dcm_tip=k_tip, ratio_avl_over_xfoil=k_avl / x_per_deg,
                                                 avl_dcl_per_deg=d_avl))
    (OUT / "lip_moment_avl_vs_xfoil.csv").write_text("")
    _write_csv(OUT / "lip_moment_avl_vs_xfoil.csv", rows)
    # summary per lip (placement full, cl 0.25, panels 160): ratio range over wings/sections
    summ = {}
    for cf in LIP_CFS:
        for ang in LIP_ANGLES[cf]:
            for pl in PLACEMENTS:
                rs = [r for r in rows if r["cf"] == cf and r["angle"] == ang and r["placement"] == pl and r["cl"] == 0.25 and r["panels"] == 160]
                summ[f"{int(cf*100)}/{ang:g}|{pl}"] = dict(ratio_min=min(r["ratio_avl_over_xfoil"] for r in rs), ratio_max=max(r["ratio_avl_over_xfoil"] for r in rs),
                                                           ratio_median=statistics.median(r["ratio_avl_over_xfoil"] for r in rs),
                                                           xfoil_dcm_per_deg=statistics.median(r["xfoil_inviscid_dcm_per_deg"] for r in rs),
                                                           avl_dcm_per_deg=statistics.median(r["avl_dcm_per_deg_area_weighted"] for r in rs),
                                                           avl_root_over_tip=(statistics.median(r["avl_dcm_root"] for r in rs), statistics.median(r["avl_dcm_tip"] for r in rs)))
    (OUT / "lip_moment_ratio_summary.json").write_text(json.dumps(summ, indent=1))
    for k, v in summ.items():
        print(k, {a: (round(b, 4) if isinstance(b, float) else b) for a, b in v.items() if a != "avl_root_over_tip"})


def _write_csv(path, rows):
    if not rows:
        path.write_text("")
        return
    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with path.open("w", newline="", encoding="utf-8") as s:
        w = csv.DictWriter(s, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: (f"{x:.6g}" if isinstance(x, float) else x) for k, x in r.items()})


# ------------------------------------------------------------------ core worker
_S = {}


def _init():
    _S["models"] = load_models()
    _S["tabs"] = A.ArchTables()


def _variants(kind):
    v = [("avl_only", None, None)]
    forms = ["coupled", "replace"] if kind == "lip" else ["coupled"]
    for f in forms:
        for nc in NCRITS:
            for pan in PANELS:
                v.append((f, nc, pan))
    return v


def scan_curve(curve):
    """-> (sets per mode, pts per SM of the lowest-CL 'any' good root, all roots at SM_ROOTS)"""
    sets = {md: [] for md in MODES}
    pts, roots = [], {}
    for sm in SM_SCAN:
        rs = v2.roots_from_curve_v2(curve, sm)
        for md in MODES:
            if any(v2.is_good_v2(r, BAND, md) for r in rs):
                sets[md].append(sm)
        good = [r for r in rs if v2.is_good_v2(r, BAND, "any")]
        if good:
            r0 = min(good, key=lambda r: r["cl"])
            pts.append([sm, r0["cl"], r0["dg"], r0["df"], r0["jag"], r0["ti"]])
        if sm in SM_ROOTS and rs:
            roots[str(sm)] = [[r["cl"], r["dg"], r["df"], r["cov"], r["covd"], r["jag"], r["ti"]] for r in rs]
    return sets, pts, roots


def work(job):
    kind, wid, sec, cfg = job
    models, tabs = _S["models"], _S["tabs"]
    ar, sw, tp = WINGS[wid]
    recs = []
    if kind == "pure":
        M = models[mkey("pure", wid)]
        feats = [(0.0, None)]
    elif kind == "lip":
        cf, pl = cfg
        M = models[mkey("lip", wid, cf, pl)]
        feats = [(ang, ang) for ang in LIP_ANGLES[cf]]
    else:
        ta, stn = cfg
        M = models[mkey("tail", wid, ta, stn)]
        feats = [(inc, inc) for inc in TAIL_INCS]
    for u, ang in feats:
        for ws in WS_LIST:
            for form, nc, pan in _variants(kind):
                if form == "avl_only":
                    c = A.Cfg(None)
                else:
                    clean = tabs.clean(sec, nc, pan)
                    if kind == "lip":
                        c = A.Cfg(clean, tabs.bent(sec, cfg[0], ang, nc, pan), tabs.inv_curve(sec, cfg[0], ang, pan))
                    else:
                        c = A.Cfg(clean)
                curve = A.build_curve_arch(M, B, u if kind != "pure" else 0.0, c, ws, form)
                sets, pts, roots = scan_curve(curve)
                recs.append(dict(kind=kind, wing=wid, section=sec, cfg=list(cfg) if cfg else None, u=u, ws=ws, form=form, ncrit=nc, panels=pan,
                                 sets=sets, pts=pts, roots=roots))
    return recs


def stage_core(sub=None):
    t = time.time()
    jobs = []
    for wid in WINGS:
        for sec in SECTIONS:
            jobs.append(("pure", wid, sec, None))
            for cf in LIP_CFS:
                for pl in PLACEMENTS:
                    jobs.append(("lip", wid, sec, (cf, pl)))
            for ta in TAIL_AREAS:
                for stn in TAIL_STATIONS:
                    jobs.append(("tail", wid, sec, (ta, stn)))
    if sub:
        jobs = [j for j in jobs if j[0] in sub]
    with Pool(8, initializer=_init) as pool:
        res = pool.map(work, jobs, chunksize=1)
    recs = [r for x in res for r in x]
    name = "scans_arch.json" if not sub else "scans_arch_" + "_".join(sub) + ".json"
    (OUT / name).write_text(json.dumps(recs))
    print(f"core: {len(jobs)} jobs, {len(recs)} records, {time.time()-t:.0f}s")


# ------------------------------------------------------------------ windows
def model_for(models, rec):
    c = rec["cfg"]
    if rec["kind"] == "pure":
        return models[mkey("pure", rec["wing"])]
    if rec["kind"] == "lip":
        return models[mkey("lip", rec["wing"], c[0], c[1])]
    return models[mkey("tail", rec["wing"], c[0], c[1])]


def window_rows(recs, models):
    grp = defaultdict(list)
    for r in recs:
        grp[(r["kind"], r["wing"], r["section"], json.dumps(r["cfg"]), r["u"], r["ws"])].append(r)
    rows = []
    for key, rs in grp.items():
        kind, wing, sec, cfg, u, ws = key
        M = model_for(models, rs[0])
        mac_mm = M["mac"] * B * 1000
        xnp_mm = M["xnp"] * B * 1000
        c = json.loads(cfg)
        base = dict(kind=kind, wing=wing, section=sec, cfg1=(c[0] if c else None), cfg2=(c[1] if c else None), u=u, ws=ws)

        def emit(form, pan, tier, mode, sm_set, ptsets):
            w = v2.contiguous_window(sm_set, STEP)
            row = dict(base, form=form, panels=pan, tier=tier, mode=mode, n_points=w["n"], widest_n=w["widest_n"], runs=w["runs"],
                       width_pct_mac=None if w["width"] is None else 100 * w["width"], width_mm=None if w["width"] is None else 1000 * w["width"] * M["mac"] * B)
            if w["n"]:
                lo, hi = w["sm_lo"], w["sm_hi"]
                inw = [p for pts in ptsets for p in pts if lo - 1e-9 <= p[0] <= hi + 1e-9]
                row.update(sm_lo=lo, sm_hi=hi, cg_mm_lo=xnp_mm - hi * mac_mm, cg_mm_hi=xnp_mm - lo * mac_mm,
                           cl_lo=min(p[1] for p in inw), cl_hi=max(p[1] for p in inw))
            rows.append(row)
        av = [r for r in rs if r["form"] == "avl_only"][0]
        emit("avl_only", None, "avl", "any", av["sets"]["any"], [av["pts"]])
        for form in ("coupled", "replace"):
            for pan in PANELS:
                by = {r["ncrit"]: r for r in rs if r["form"] == form and r["panels"] == pan}
                if 5 not in by:
                    continue
                for mode in MODES:
                    s5, s9 = set(by[5]["sets"][mode]), set(by[9]["sets"][mode])
                    emit(form, pan, "N5", mode, s5, [by[5]["pts"]])
                    emit(form, pan, "N9", mode, s9, [by[9]["pts"]])
                    emit(form, pan, "both", mode, s5 & s9, [by[5]["pts"], by[9]["pts"]])
    return rows


def stage_windows():
    models = load_models()
    recs = json.loads((OUT / "scans_arch.json").read_text())
    rows = window_rows(recs, models)
    _write_csv(OUT / "windows.csv", rows)
    print(len(rows), "window rows")


# ------------------------------------------------------------------ analysis helpers
LIP_ORDER = [(0.10, 3.0), (0.15, 5.0), (0.10, 6.0), (0.20, 5.0), (0.15, 8.0), (0.20, 10.0)]     # by inviscid lip moment
SEC0 = "NACA_0006"


def load_windows_rows():
    models = load_models()
    recs = json.loads((OUT / "scans_arch.json").read_text())
    return models, recs, window_rows(recs, models)


def wrow(rows, **kw):
    for r in rows:
        if all(r.get(k) == v for k, v in kw.items()):
            return r
    return None


def width(r):
    return r["width_mm"] if (r and r["n_points"]) else 0.0


def index_rows(rows):
    d = {}
    for r in rows:
        d[(r["kind"], r["wing"], r["section"], r["cfg1"], r["cfg2"], r["u"], r["ws"], r["form"], r["panels"], r["tier"], r["mode"])] = r
    return d


def wd(idx, kind, wing, sec, cfg1, cfg2, u, ws, form, pan, tier, mode):
    r = idx.get((kind, wing, sec, cfg1, cfg2, u, ws, form, pan, tier, mode))
    return r["width_mm"] if (r and r["n_points"]) else 0.0


def stage_analysis():
    models, recs, rows = load_windows_rows()
    idx = index_rows(rows)
    out = {}
    # ---- A: lip threshold table
    thr = []
    for pl in PLACEMENTS:
        for cf, ang in LIP_ORDER:
            for ws in WS_LIST:
                rec = dict(placement=pl, lip=f"{int(cf*100)}/{ang:g}", ws=ws)
                for form in ("coupled", "replace"):
                    for pan in PANELS:
                        for mode in ("any", "offti", "clean"):
                            n = 0
                            w = []
                            gain = []
                            for wing in WINGS:
                                for sec in SECTIONS:
                                    x = wd(idx, "lip", wing, sec, cf, pl, ang, ws, form, pan, "both", mode)
                                    w.append(x)
                                    n += x >= THRESH_MM
                                    if mode == "any":
                                        gain.append(x - wd(idx, "pure", wing, sec, None, None, 0.0, ws, "coupled", pan, "both", "any"))
                            rec[f"n_ge3mm_{form}_p{pan}_{mode}"] = n
                            rec[f"median_mm_{form}_p{pan}_{mode}"] = statistics.median(w)
                            if mode == "any":
                                rec[f"median_gain_over_pure_mm_{form}_p{pan}"] = statistics.median(gain)
                rec["n_ge3mm_avl_only"] = sum(1 for wing in WINGS for sec in SECTIONS if wd(idx, "lip", wing, sec, cf, pl, ang, ws, "avl_only", None, "avl", "any") >= THRESH_MM)
                thr.append(rec)
    _write_csv(OUT / "lip_threshold_table.csv", thr)
    # ---- best configs per wing (section NACA_0006): mean both-Ncrit coupled(160,any) window over W/S 8 and 16
    best = {}
    for wing in WINGS:
        sec = SEC0
        cand_l = []
        for cf in LIP_CFS:
            for pl in PLACEMENTS:
                for ang in LIP_ANGLES[cf]:
                    sc = statistics.mean(wd(idx, "lip", wing, sec, cf, pl, ang, ws, "coupled", 160, "both", "any") for ws in (8, 16))
                    sc_strict = statistics.mean(wd(idx, "lip", wing, sec, cf, pl, ang, ws, "coupled", 160, "both", "offti") for ws in (8, 16))
                    cand_l.append((sc, -LIP_ORDER.index((cf, ang)), sc_strict, dict(cf=cf, placement=pl, angle=ang)))
        cand_t = []
        for ta in TAIL_AREAS:
            for stn in TAIL_STATIONS:
                for inc in TAIL_INCS:
                    sc = statistics.mean(wd(idx, "tail", wing, sec, ta, stn, inc, ws, "coupled", 160, "both", "any") for ws in (8, 16))
                    sc_strict = statistics.mean(wd(idx, "tail", wing, sec, ta, stn, inc, ws, "coupled", 160, "both", "offti") for ws in (8, 16))
                    cand_t.append((sc, -abs(inc) - ta * 10, sc_strict, dict(area=ta, station=stn, inc=inc)))
        bl, bt = max(cand_l, key=lambda t: (t[0], t[1])), max(cand_t, key=lambda t: (t[0], t[1]))
        best[wing] = dict(section=sec, lip=bl[3], lip_score_mm=bl[0], lip_strict_score_mm=bl[2], tail=bt[3], tail_score_mm=bt[0], tail_strict_score_mm=bt[2],
                          pure_score_mm=statistics.mean(wd(idx, "pure", wing, sec, None, None, 0.0, ws, "coupled", 160, "both", "any") for ws in (8, 16)))
    out["best"] = best
    (OUT / "best_configs.json").write_text(json.dumps(best, indent=1))
    # ---- headline answers on lips
    ans = {}
    for form in ("coupled", "replace"):
        for mode in ("any", "offti", "clean"):
            sm_lip = {}
            for ws in WS_LIST:
                ok = []
                for pl in PLACEMENTS:
                    for cf, ang in LIP_ORDER:
                        r = [t for t in thr if t["placement"] == pl and t["lip"] == f"{int(cf*100)}/{ang:g}" and t["ws"] == ws][0]
                        k = f"n_ge3mm_{form}_p160_{mode}"
                        k2 = f"n_ge3mm_{form}_p240_{mode}"
                        ok.append((pl, f"{int(cf*100)}/{ang:g}", r[k], r[k2]))
                sm_lip[ws] = ok
            ans[f"{form}|{mode}"] = {ws: [o for o in v if o[2] >= 4 and o[3] >= 4] for ws, v in sm_lip.items()}
    out["lips_ge3mm_in_at_least_4_of_6_cases_both_panels"] = ans
    (OUT / "analysis.json").write_text(json.dumps(out, indent=1, default=str))
    for w, b in best.items():
        print(w, {k: (round(v, 2) if isinstance(v, float) else v) for k, v in b.items()})
    for key in ("coupled|any", "coupled|offti", "coupled|clean", "replace|any"):
        print(key)
        for ws, v in ans[key].items():
            print("  ws", ws, [(o[0], o[1], o[2], o[3]) for o in v][:6])


# ------------------------------------------------------------------ nominal points and table D
def rec_index(recs):
    d = {}
    for r in recs:
        d[(r["kind"], r["wing"], r["section"], json.dumps(r["cfg"]), r["u"], r["ws"], r["form"], r["ncrit"], r["panels"])] = r
    return d


def pt_at(pts, sm):
    for p in pts:
        if abs(p[0] - sm) < 1e-9:
            return p
    return None


def cl_slope_per_mm(pts, sm, mac_mm):
    """|dCL/dx_cg| per mm from neighbouring scan roots (central difference if both exist)."""
    a, b = pt_at(pts, round(sm - STEP, 4)), pt_at(pts, round(sm + STEP, 4))
    if a and b:
        return abs((b[1] - a[1]) / (2 * STEP)) / mac_mm
    c = pt_at(pts, sm)
    n = b or a
    if c and n:
        return abs((n[1] - c[1]) / STEP) / mac_mm
    return None


def cfg_models(models, kind, wing, cfg):
    if kind == "pure":
        return models[mkey("pure", wing)]
    return models[mkey(kind, wing, cfg[0], cfg[1])]


def nominal(models, recidx, tabs, ur, kind, wing, sec, cfg, u, ws):
    """Nominal trim for one configuration at one W/S: CG at the centre of the both-Ncrit coupled window (else N5/N9/AVL window)."""
    M = cfg_models(models, kind, wing, cfg)
    mac_mm = M["mac"] * B * 1000
    xnp_mm = M["xnp"] * B * 1000
    cj = json.dumps(cfg) if cfg else "null"

    def w_(form, pan, tier, mode):
        return ur.get((kind, wing, sec, (cfg[0] if cfg else None), (cfg[1] if cfg else None), u, ws, form, pan, tier, mode))

    out = dict(kind=kind, wing=wing, section=sec, cfg=cfg, u=u, ws=ws, mass_g=1000 * ws * M["area"] * B * B / G, mac_mm=mac_mm, xnp_mm=xnp_mm,
               x_lemac_mm=M["x_lemac"] * B * 1000)
    ra_ = w_("avl_only", None, "avl", "any")
    out["win_avl_mm"] = ra_["width_mm"] if ra_ and ra_["n_points"] else 0.0
    for tier in ("N5", "N9", "both"):
        for mode in ("any", "offti", "clean"):
            r = w_("coupled", 160, tier, mode)
            out[f"win_{tier}_{mode}_mm"] = r["width_mm"] if r and r["n_points"] else 0.0
            r2 = w_("coupled", 240, tier, mode)
            out[f"win240_{tier}_{mode}_mm"] = r2["width_mm"] if r2 and r2["n_points"] else 0.0
        if kind == "lip":
            r3 = w_("replace", 160, tier, "any")
            out[f"win2b_{tier}_any_mm"] = r3["width_mm"] if r3 and r3["n_points"] else 0.0
    centre = None
    for tier in ("both", "N5", "N9"):
        r = w_("coupled", 160, tier, "any")
        if r and r["n_points"]:
            centre = 0.5 * (r["sm_lo"] + r["sm_hi"])
            out["centre_from"] = tier
            break
    if centre is None and ra_ and ra_["n_points"]:
        centre = 0.5 * (ra_["sm_lo"] + ra_["sm_hi"])
        out["centre_from"] = "avl_only"
    if centre is None:
        out["centre_from"] = None
        return out
    centre = round(round(centre / STEP) * STEP, 4)
    out.update(sm_nominal=centre, cg_mm=xnp_mm - centre * mac_mm, cg_pct_mac=100 * ((xnp_mm - centre * mac_mm) - M["x_lemac"] * B * 1000) / mac_mm)
    for nc in NCRITS:
        rec = recidx.get((kind, wing, sec, cj, u, ws, "coupled", nc, 160))
        p = pt_at(rec["pts"], centre) if rec else None
        if not p:
            out[f"N{nc}"] = None
            continue
        clean = tabs.clean(sec, nc, 160)
        c = A.Cfg(clean, tabs.bent(sec, cfg[0], u, nc, 160), tabs.inv_curve(sec, cfg[0], u, 160)) if kind == "lip" else A.Cfg(clean)
        uu = u if kind != "pure" else 0.0
        ts = A.trim_state(M, B, uu, c, p[1], ws)
        out[f"N{nc}"] = dict(cl=p[1], alpha_deg=ts["alpha_deg"], v=ts["V"], re_root=ts["re_root"], re_tip=ts["re_tip"], re_mac=ts["re_mac"], ld=ts["ld"],
                            margin_glide=-p[2], margin_fixed=-p[3], jag=p[4], ti=p[5], cl_per_mm=cl_slope_per_mm(rec["pts"], centre, mac_mm))
    sm = centre
    if kind != "pure":
        cm0u = (M["d_cm"] - M["a_cm"] / M["a_cl"] * M["d_cl"])
        out["avl_only_cl"] = u * cm0u / sm if sm > 0 else None
        out["avl_only_cl_per_deg"] = cm0u / sm if sm > 0 else None
        out["avl_only_cl_per_mm"] = (u * cm0u / sm ** 2) / mac_mm if sm > 0 else None
    else:
        out["avl_only_cl"] = 0.0
    out["avl_sm"] = sm
    return out


def row_index(rows):
    return {(r["kind"], r["wing"], r["section"], r["cfg1"], r["cfg2"], r["u"], r["ws"], r["form"], r["panels"], r["tier"], r["mode"]): r for r in rows}


def stage_tables():
    models, recs, rows = load_windows_rows()
    recidx = rec_index(recs)
    ridx = row_index(rows)
    tabs = A.ArchTables()
    best = json.loads((OUT / "best_configs.json").read_text())
    out = []
    for wing in WINGS:
        b = best[wing]
        sec = b["section"]
        lcf = (b["lip"]["cf"], b["lip"]["placement"])
        tcf = (b["tail"]["area"], b["tail"]["station"])
        for ws in (8, 16):
            for kind, cfg, u in (("pure", None, 0.0), ("lip", lcf, b["lip"]["angle"]), ("tail", tcf, b["tail"]["inc"])):
                out.append(nominal(models, recidx, tabs, ridx, kind, wing, sec, cfg, u, ws))
    for o in out:
        if o["kind"] == "pure" or o.get("sm_nominal") is None:
            continue
        cfg = o["cfg"]
        others = [a for a in LIP_ANGLES[cfg[0]] if a != o["u"]] if o["kind"] == "lip" else [i for i in TAIL_INCS if i != o["u"]]
        sens = {}
        for nc in NCRITS:
            rec1 = recidx.get((o["kind"], o["wing"], o["section"], json.dumps(cfg), o["u"], o["ws"], "coupled", nc, 160))
            vals = []
            for u2 in others:
                rec2 = recidx.get((o["kind"], o["wing"], o["section"], json.dumps(cfg), u2, o["ws"], "coupled", nc, 160))
                if not rec1 or not rec2:
                    continue
                common = [(abs(p[0] - o["sm_nominal"]), p[0], (p[1] - q[1]) / (o["u"] - u2)) for p in rec1["pts"] for q in rec2["pts"] if abs(p[0] - q[0]) < 1e-9]
                if common:
                    d, smc, val = min(common)
                    vals.append((val, smc))
            sens[f"N{nc}"] = dict(value=statistics.mean(v[0] for v in vals), at_sm=vals[0][1]) if vals else None
        o["cl_per_deg_feature"] = sens
    (OUT / "nominal_points.json").write_text(json.dumps(out, indent=1, default=str))
    _write_csv(OUT / "table_D_flat.csv", [flatten(o) for o in out])
    write_table_d(out)
    print("table D written;", len(out), "points")
    return out


def flatten(o):
    r = {k: v for k, v in o.items() if not isinstance(v, (dict, list, tuple))}
    r["cfg"] = json.dumps(o.get("cfg"))
    for nc in ("N5", "N9"):
        d = o.get(nc)
        if d:
            for k, v in d.items():
                r[f"{nc}_{k}"] = v
    if o.get("cl_per_deg_feature"):
        for k, v in o["cl_per_deg_feature"].items():
            r[f"dCL_per_deg_{k}"] = v
    return r


def fmt(x, n=2):
    return "-" if x is None else (f"{x:.{n}f}" if isinstance(x, (int, float)) else str(x))


def write_table_d(pts):
    L = ["TABLE D: architecture comparison at the candidate wings (b = 0.40 m, NACA 0006). Level 1 coupling; lip/tail = the best-window configuration per wing",
         "(best_configs.json); nominal CG = centre of the both-Ncrit coupled window (panels 160, any evidence); window width = (n-1)*0.5% MAC.", ""]
    for wing in WINGS:
        L.append(f"=== {wing} (AR {WINGS[wing][0]}, c/4 sweep {WINGS[wing][1]} deg, taper {WINGS[wing][2]}) ===")
        for ws in (8, 16):
            L.append(f" W/S {ws} N/m2")
            for o in [p for p in pts if p["wing"] == wing and p["ws"] == ws]:
                if o["kind"] == "pure":
                    name = "pure"
                elif o["kind"] == "lip":
                    name = f"lip {int(round(o['cfg'][0]*100))}% {o['cfg'][1]} {o['u']:g} deg"
                else:
                    name = f"tail {int(round(o['cfg'][0]*100))}% @{o['cfg'][1]:g}cr inc {o['u']:g}"
                L.append(f"  {name}: mass {o['mass_g']:.0f} g; window mm AVL-only {fmt(o['win_avl_mm'],1)} | N5 {fmt(o['win_N5_any_mm'],1)} | N9 {fmt(o['win_N9_any_mm'],1)} | both {fmt(o['win_both_any_mm'],1)} "
                         f"(strict off-TI both {fmt(o['win_both_offti_mm'],1)}, clean both {fmt(o['win_both_clean_mm'],1)}; panels240 both {fmt(o['win240_both_any_mm'],1)}"
                         + (f"; 2b both {fmt(o.get('win2b_both_any_mm'),1)}" if o["kind"] == "lip" else "") + ")")
                if o.get("sm_nominal") is None:
                    L.append("     no nominal trim (no window)")
                    continue
                L.append(f"     nominal CG {o['cg_mm']:.1f} mm ({o['cg_pct_mac']:.1f}% MAC, AVL SM {100*o['sm_nominal']:+.1f}%) [centre from {o['centre_from']}]")
                for nc in ("N5", "N9"):
                    d = o.get(nc)
                    L.append(f"     {nc}: " + ("no valid trim at this CG" if not d else
                             f"CL {d['cl']:.2f} alpha {d['alpha_deg']:.1f} V {d['v']:.1f} m/s Re {d['re_root']/1000:.0f}-{d['re_tip']/1000:.0f}k margins g/f {d['margin_glide']:+.3f}/{d['margin_fixed']:+.3f} "
                             f"dCL/mm {fmt(d['cl_per_mm'],3)} L/D {fmt(d['ld'],1)} (jag {d['jag']:.2f} TI {fmt(d['ti'])})"))
                if o["kind"] != "pure":
                    L.append(f"     AVL-only trim CL {fmt(o.get('avl_only_cl'))}, dCL/deg {fmt(o.get('avl_only_cl_per_deg'),3)}, dCL/mm {fmt(o.get('avl_only_cl_per_mm'),3)}; coupled dCL/deg {o.get('cl_per_deg_feature')}")
    (OUT / "table_D.txt").write_text("\n".join(L), encoding="utf-8")


# ------------------------------------------------------------------ Part C: lateral
FIN_STD = 0.06
FIN_STD_X = 1.5          # quarter chord of the fin, in root chords aft of the root LE


def lateral_cases():
    best = json.loads((OUT / "best_configs.json").read_text())
    models = load_models()
    cases = []
    for wing, (ar, sw, tp) in WINGS.items():
        b = best[wing]
        cr = models[mkey("pure", wing)]["root_chord"]
        archs = [("pure", models[mkey("pure", wing)], 0.0, {}, [("none", None), ("fin6@1.5cr", dict(area_frac=FIN_STD, x_qc=FIN_STD_X * cr))]),
                 ("lip", models[mkey("lip", wing, b["lip"]["cf"], b["lip"]["placement"])], b["lip"]["angle"],
                  dict(lip=(b["lip"]["cf"], b["lip"]["placement"])), [("none", None), ("fin6@1.5cr", dict(area_frac=FIN_STD, x_qc=FIN_STD_X * cr))]),
                 ("tail", models[mkey("tail", wing, b["tail"]["area"], b["tail"]["station"])], b["tail"]["inc"],
                  dict(tail=dict(area_frac=b["tail"]["area"], x_qc=b["tail"]["station"] * cr, incidence=b["tail"]["inc"])),
                  [("none", None), ("boomfin4@tail", dict(area_frac=0.04, x_qc=b["tail"]["station"] * cr)),
                   ("boomfin6@tail", dict(area_frac=0.06, x_qc=b["tail"]["station"] * cr))])]
        for kind, M, u, kw, fins in archs:
            for dih in (0, 3):
                for fname, fin in fins:
                    cases.append((wing, kind, M, u, kw, dih, fname, fin))
    return cases


def _lat_work(c):
    wing, kind, M, u, kw, dih, fname, fin = c
    ar, sw, tp = WINGS[wing]
    xcg = M["xnp"] - 0.05 * M["mac"]
    r = A.lateral_arch(ar, sw, tp, dih, xcg, OUT / "avl_lat" / f"{wing}_{kind}_d{dih}_{fname.replace('@', '_')}", f"{kind}", M, kind, u, fin=fin, **kw)
    row = dict(wing=wing, arch=kind, dihedral=dih, fin=fname, feature=u)
    for k, v in r["cl_beta"].items():
        row[f"cl_beta_CL{k}"] = v
    for k, v in r["cn_beta"].items():
        row[f"cn_beta_CL{k}"] = v
    for k in ("roll_per_deg_incidence_mismatch", "yaw_per_deg_incidence_mismatch", "roll_per_deg_lip_mismatch", "yaw_per_deg_lip_mismatch",
              "yaw_per_deg_fin_misalignment", "roll_per_deg_fin_misalignment", "cl_nom_actual", "alpha_nom_deg"):
        row[k] = r.get(k)
    return row


def stage_lateral():
    t = time.time()
    cases = lateral_cases()
    with Pool(8) as pool:
        rows = pool.map(_lat_work, cases, chunksize=1)
    _write_csv(OUT / "lateral.csv", rows)
    print(f"lateral: {len(rows)} cases in {time.time()-t:.0f}s")


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    fn = {"avl": stage_avl, "lipcmp": stage_lipcmp, "core": stage_core, "windows": stage_windows, "analysis": stage_analysis, "tables": stage_tables,
          "lateral": stage_lateral}
    for s in (list(fn) if what == "all" else [what]):
        fn[s]()


if __name__ == "__main__":
    main()
