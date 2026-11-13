"""v2 pure-wing study: Part 2 (extended coupled study + CG windows), Part 3 (sweep bracket),
Part 4 (lateral / fins), Part 5 (candidate sheets), figures.  Imports the unmodified model; writes only to
results/runs/rigid_glider/pure_wing_v2/.

py -B rigid_glider/pure_wing_v2_study.py shapes | core | sweep | windows | lateral | candidates | figs | all
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

import pure_wing_model as m
import pure_wing_study as st
import pure_wing_v2_model as v2

OUT = m.ROOT / "results/runs/rigid_glider/pure_wing_v2"
CFG = json.loads(Path(__file__).with_name("pure_wing_v2_config.json").read_text())
BAND = tuple(CFG["cl_band"])
STEP = CFG["sm_scan_step"]
SM_SCAN = [round(CFG["sm_scan_min"] + STEP * i, 4) for i in range(int(round((CFG["sm_scan_max"] - CFG["sm_scan_min"]) / STEP)) + 1)]
SM_DETAIL = [round(CFG["sm_scan_min"] + CFG["sm_detail_step"] * i, 4)
             for i in range(int(round((CFG["sm_scan_max"] - CFG["sm_scan_min"]) / CFG["sm_detail_step"])) + 1)]
G = 9.80665
SHAPES = {s["id"]: s for s in CFG["shapes"]}


def skey(sid):
    s = SHAPES[sid]
    return m.shape_key(s["ar"], s["sweep"], s["taper"], 0)


def write_csv(path, rows):
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


# ------------------------------------------------------------------ shapes
def stage_shapes():
    OUT.mkdir(parents=True, exist_ok=True)
    cache = {}
    for s in CFG["shapes"]:
        m.get_shape(s["ar"], s["sweep"], s["taper"], 0, cache, OUT / "avl")
        cache[skey(s["id"])]["id"] = s["id"]
    (OUT / "avl_shapes_v2.json").write_text(json.dumps(cache))
    print(len(cache), "shapes; max |Cm0|", max(abs(v["cm0"]) for v in cache.values()),
          "max CLa nonlin", max(v["cla_lin_err"] for v in cache.values()))


# ------------------------------------------------------------------ worker
_S = {}


def _init():
    _S["shapes"] = json.loads((OUT / "avl_shapes_v2.json").read_text())
    _S["tables"] = m.load_tables()


def work(args):
    sid, sweep_re_exp, spans, sections, detailed = args[:5]
    ws_list = args[5] if len(args) > 5 else CFG["ws_n_m2"]
    key = skey(sid)
    shape = _S["shapes"][key]
    sw = SHAPES[sid]["sweep"]
    rows, scans = [], []
    for sec in sections:
        for ncrit in (5, 9):
            base = _S["tables"][(sec, ncrit)]
            table = base if sweep_re_exp is None else v2.SweepSection(base, sw, re_exp=sweep_re_exp)
            for b in spans:
                mac = shape["mac"] * b
                for ws in ws_list:
                    curve = v2.build_curve_v2(shape, b, table, ws)
                    sets = {md: [] for md in v2.MODES}
                    pts = []
                    for sm in SM_SCAN:
                        rs = v2.roots_from_curve_v2(curve, sm)
                        for md in v2.MODES:
                            if any(v2.is_good_v2(r, BAND, md) for r in rs):
                                sets[md].append(sm)
                        gd = [r for r in rs if v2.is_good_v2(r, BAND, "any")]
                        if gd:
                            r0 = min(gd, key=lambda r: r["cl"])
                            pts.append([sm, r0["cl"], m.glide_speed(ws, r0["cl"]) * mac / m.NU, r0["ti"], r0["jag"]])
                    scans.append(dict(id=sid, key=key, section=sec, ncrit=ncrit, span=b, ws=ws, sets=sets, pts=pts))
                    if not detailed:
                        continue
                    ctuples = [(p["cl"], p["cm"], p["cov"]) for p in curve]
                    for sm in SM_DETAIL:
                        roots = m.trim_roots(shape, b, table, ws, sm, st.GRID, curve=ctuples)
                        cg = m.cg_from_sm(shape, b, sm)
                        S = shape["area"] * b * b
                        for r in roots:
                            cl = r["CL"]
                            ti = v2.ti_fraction_v2(shape, b, table, cl, r["V"]) if 0.1 <= cl <= 0.8 else None
                            cell = st.section_cell(shape, b, table, cl, r["V"]) or {} if not sweep_re_exp else {}
                            clb, cnb = m.lateral_at(shape, cl)
                            cdi = shape["k_cdi"] * cl ** 2
                            rows.append(dict(
                                id=sid, shape=key, ar=shape["ar"], sweep=sw, taper=shape["taper"], section=sec, ncrit=ncrit,
                                span_m=b, ws_n_m2=ws, mass_g=1000 * ws * S / G, sm_avl=sm, cg_mm=cg["xcg_mm"],
                                cg_pct_mac=100 * cg["xcg_over_mac_from_lemac"], xnp_mm=cg["xnp_mm"], trim_cl=cl,
                                alpha_deg=cl / shape["cla_per_rad"] * 180 / math.pi + (r["dalpha_deg"] or 0.0),
                                dcm_dcl_glide=r["dcm_dcl_glide"], dcm_dcl_fixed=r["dcm_dcl_fixed_speed"],
                                sm_coupled_glide=-r["dcm_dcl_glide"], sm_coupled_fixed=-r["dcm_dcl_fixed_speed"],
                                cl_beta_per_rad=clb, cn_beta_per_rad=cnb, v_m_s=r["V"], re_root=r["re_root"],
                                re_mac=r["re_mac"], re_tip=r["re_tip"], cdi=cdi, cdp=r["cdp"],
                                l_over_d=(cl / (cdi + r["cdp"]) if r["cdp"] else None), coverage=r["coverage"],
                                deriv_coverage=r["deriv_min_coverage"], jagged_fraction=r["jagged_fraction"],
                                ti_fraction=ti, stable_glide=int(r["stable_glide"]), stable_fixed=int(r["stable_fixed"]),
                                valid=int(r["valid"]), in_band=int(BAND[0] <= cl <= BAND[1]),
                                clean=int(r["valid"] and r["jagged_fraction"] <= st.JAG_CLEAN),
                                off_ti=int(ti is not None and ti < 0.5), cm_sec_mean=cell.get("cm_sec_mean"),
                                dcm_dcl_sec=cell.get("dcm_dcl_sec"), avl_only_trim_cl=0.0))
    return rows, scans


def _run(jobs, nproc=8):
    with Pool(nproc, initializer=_init) as pool:
        res = pool.map(work, jobs, chunksize=1)
    return [r for x in res for r in x[0]], [s for x in res for s in x[1]]


def stage_core():
    t = time.time()
    jobs = []
    for sid in SHAPES:
        secs = list(CFG["sections"]) + (["NACA_0004"] if sid == CFG["reference_0004_shape"] else [])
        jobs.append((sid, None, CFG["spans_m"], secs, True))
    rows, scans = _run(jobs)
    (OUT / "scans_streamwise.json").write_text(json.dumps(scans))
    write_csv(OUT / "p2_roots.csv", rows)
    print(f"core: {len(rows)} root rows, {len(scans)} curves, {time.time()-t:.0f}s")


def stage_sweep():
    t = time.time()
    for re_exp in (2, 1):
        jobs = [(sid, re_exp, [CFG["sweep_bracket_span_m"]], list(CFG["sections"]), False) for sid in SHAPES]
        _, scans = _run(jobs)
        (OUT / f"scans_sweepcorr_reexp{re_exp}.json").write_text(json.dumps(scans))
    print(f"sweep bracket done {time.time()-t:.0f}s")


# ------------------------------------------------------------------ windows
TIERS = ("N5", "N9", "both")
SMCLS = ("SM>=0", "all")


def compute_windows(scans, shapes):
    """One row per (case, mode, tier, sm class). width = (n-1)*step of the widest CONTIGUOUS run of scan SM points;
    widest_n == 1 means 'between 0 and 2 steps' (reported as width 0)."""
    idx = defaultdict(dict)
    for s in scans:
        idx[(s["id"], s["section"], s["span"], s["ws"])][s["ncrit"]] = s
    rows = []
    for (sid, sec, b, ws), by in idx.items():
        shape = shapes[skey(sid)]
        mac = shape["mac"] * b
        if 5 not in by or 9 not in by:
            continue
        for md in v2.MODES:
            sets = {5: set(by[5]["sets"][md]), 9: set(by[9]["sets"][md])}
            tierset = {"N5": sets[5], "N9": sets[9], "both": sets[5] & sets[9]}
            for tier, ss in tierset.items():
                for cls in SMCLS:
                    pts = [x for x in ss if cls == "all" or x >= -1e-9]
                    w = v2.contiguous_window(pts, STEP)
                    if not w["n"]:
                        rows.append(dict(id=sid, section=sec, span_m=b, ws=ws, mode=md, tier=tier, sm_class=cls, n_points=0,
                                         widest_n=0, runs=0, width_pct_mac=None, width_mm=None, sm_lo=None, sm_hi=None))
                        continue
                    lo, hi = w["sm_lo"], w["sm_hi"]
                    inwin = [p for nc in (5, 9) if tier in ("both", f"N{nc}") for p in by[nc]["pts"] if lo - 1e-9 <= p[0] <= hi + 1e-9]
                    rows.append(dict(id=sid, section=sec, span_m=b, ws=ws, mode=md, tier=tier, sm_class=cls, n_points=w["n"],
                                     widest_n=w["widest_n"], runs=w["runs"], width_pct_mac=100 * w["width"],
                                     width_mm=1000 * w["width"] * mac, sm_lo=lo, sm_hi=hi,
                                     cg_mm_aft_lo=1000 * (shape["xnp"] * b - hi * mac), cg_mm_aft_hi=1000 * (shape["xnp"] * b - lo * mac),
                                     re_mac_min=min(p[2] for p in inwin), re_mac_max=max(p[2] for p in inwin),
                                     cl_min=min(p[1] for p in inwin), cl_max=max(p[1] for p in inwin),
                                     ti_mean=statistics.mean(p[3] for p in inwin if p[3] is not None) if any(p[3] is not None for p in inwin) else None))
    return rows


def stage_windows():
    shapes = json.loads((OUT / "avl_shapes_v2.json").read_text())
    for tag, fn in (("streamwise", "scans_streamwise.json"), ("sweepcorr_reexp2", "scans_sweepcorr_reexp2.json"),
                    ("sweepcorr_reexp1", "scans_sweepcorr_reexp1.json")):
        scans = json.loads((OUT / fn).read_text())
        rows = compute_windows(scans, shapes)
        write_csv(OUT / f"p2_windows_{tag}.csv", rows)
        print(tag, len(rows), "window rows")


# ------------------------------------------------------------------ Part 4: lateral
def lateral_configs():
    cfgs = []
    for d in CFG["dihedral_deg"]:
        cfgs.append(dict(name=f"bare_dih{d}", dih=d))
    for a in CFG["tip_fin_area_fracs_each"]:
        for mode in ("above", "sym"):
            cfgs.append(dict(name=f"tipfin_{mode}_{int(round(a*100))}pct", dih=0, tip_fin=dict(area_frac=a, mode=mode)))
    for x in CFG["centre_fin_x_qc_over_root_chord"]:
        for a in [CFG["centre_fin_area_frac"]] + CFG["centre_fin_extra_area_fracs"]:
            cfgs.append(dict(name=f"centrefin_{int(round(a*100))}pct_x{x:g}cr", dih=0, centre_fin=dict(area_frac=a, x_qc_over_root_chord=x)))
    return cfgs


def stage_lateral():
    shapes = json.loads((OUT / "avl_shapes_v2.json").read_text())
    rows = []
    for sid, sp in SHAPES.items():
        sh = shapes[skey(sid)]
        xcg = sh["xnp"] - 0.05 * sh["mac"]           # CG at SM = +5 % of the bare-wing MAC
        base = None
        for c in lateral_configs():
            r = v2.lateral_case(sp["ar"], sp["sweep"], sp["taper"], c["dih"], xcg, OUT / "avl_lat" / sid, c["name"],
                                tip_fin=c.get("tip_fin"), centre_fin=c.get("centre_fin"), cls=CFG["lateral_cls"])
            if c["name"] == "bare_dih0":
                base = r
            for cl in CFG["lateral_cls"]:
                rows.append(dict(id=sid, ar=sp["ar"], sweep=sp["sweep"], taper=sp["taper"], config=c["name"], cl=cl,
                                 cl_beta=r["cl_beta"][str(cl)], cn_beta=r["cn_beta"][str(cl)], xnp_b1m=r["xnp"],
                                 dxnp_pct_mac=100 * (r["xnp"] - base["xnp"]) / base["mac"], cla_per_rad=r["cla_per_rad"],
                                 cdi_over_cl2=r["cdi_over_cl2"]))
    write_csv(OUT / "p4_lateral.csv", rows)
    # summary per config over the 12 shapes
    summ = {}
    lo, hi = CFG["cn_beta_target"]
    for c in lateral_configs():
        for cl in (0.3, 0.4):
            rs = [r for r in rows if r["config"] == c["name"] and r["cl"] == cl]
            cn = [r["cn_beta"] for r in rs]
            summ[f"{c['name']}|CL{cl}"] = dict(cn_median=statistics.median(cn), cn_min=min(cn), cn_max=max(cn),
                                               cl_beta_median=statistics.median(r["cl_beta"] for r in rs),
                                               n_in_target=sum(1 for x in cn if lo <= x <= hi), n=len(cn),
                                               dxnp_pct_mac_median=statistics.median(r["dxnp_pct_mac"] for r in rs))
    (OUT / "p4_lateral_summary.json").write_text(json.dumps(summ, indent=1))
    for k, v in summ.items():
        if k.endswith("CL0.4"):
            print(k, {a: round(b, 4) for a, b in v.items()})


# ------------------------------------------------------------------ Part 5 support: fine W/S profile at 0.40 m
PROFILE_WS = [4, 6, 8, 10, 12, 14, 16, 20]


def stage_profile():
    t = time.time()
    shapes = json.loads((OUT / "avl_shapes_v2.json").read_text())
    for tag, re_exp in (("streamwise", None), ("sweepcorr_reexp2", 2)):
        jobs = [(sid, re_exp, [0.40], list(CFG["sections"]), False, PROFILE_WS) for sid in SHAPES]
        _, scans = _run(jobs)
        (OUT / f"scans_profile_{tag}.json").write_text(json.dumps(scans))
        write_csv(OUT / f"p5_profile_windows_{tag}.csv", compute_windows(scans, shapes))
    print(f"profile done {time.time()-t:.0f}s")


# ------------------------------------------------------------------ analysis (Part 2 summary, Part 3 bracket)
def _load_windows(tag):
    rows = list(csv.DictReader((OUT / f"p2_windows_{tag}.csv").open(encoding="utf-8")))
    for r in rows:
        for k in list(r):
            if k not in ("id", "section", "mode", "tier", "sm_class"):
                try:
                    r[k] = float(r[k])
                except (TypeError, ValueError):
                    r[k] = None
    return rows


def _wstats(rs):
    hit = [r for r in rs if r["n_points"]]
    w = [r["width_mm"] for r in hit]
    wp = [r["width_pct_mac"] for r in hit]
    return dict(cases=len(rs), with_hit=len(hit), hit_fraction=len(hit) / len(rs) if rs else None,
                single_hit=sum(1 for r in hit if r["widest_n"] == 1), width_gt_0=sum(1 for x in w if x > 0),
                width_gt_2mm=sum(1 for x in w if x > 2), median_width_mm=statistics.median(w) if w else None,
                max_width_mm=max(w) if w else None, median_width_pct_mac=statistics.median(wp) if wp else None,
                max_width_pct_mac=max(wp) if wp else None)


def stage_analysis():
    R = _load_windows("streamwise")
    S = {"scan_step": STEP, "width_definition": "(n-1)*step of widest contiguous SM run; widest_n==1 means 0..2 steps (reported 0)",
         "cl_band": BAND}
    S["by_mode_tier_class"] = {f"{md}|{t}|{c}": _wstats([r for r in R if r["mode"] == md and r["tier"] == t and r["sm_class"] == c])
                              for md in v2.MODES for t in TIERS for c in SMCLS}
    S["by_ws"] = {f"{md}|{t}|ws{int(ws)}": _wstats([r for r in R if r["mode"] == md and r["tier"] == t and r["sm_class"] == "all" and r["ws"] == ws])
                  for md in v2.MODES for t in TIERS for ws in CFG["ws_n_m2"]}
    S["by_section"] = {f"{md}|{t}|{sec}": _wstats([r for r in R if r["mode"] == md and r["tier"] == t and r["sm_class"] == "all" and r["section"] == sec])
                       for md in v2.MODES for t in TIERS for sec in CFG["sections"] + ["NACA_0004"]}
    S["by_span"] = {f"{md}|{t}|b{b}": _wstats([r for r in R if r["mode"] == md and r["tier"] == t and r["sm_class"] == "all" and r["span_m"] == b])
                    for md in v2.MODES for t in TIERS for b in CFG["spans_m"]}
    S["widest_windows"] = {}
    for md in v2.MODES:
        for t in TIERS:
            rs = sorted([r for r in R if r["mode"] == md and r["tier"] == t and r["sm_class"] == "all" and r["n_points"]], key=lambda r: -(r["width_mm"] or 0))[:6]
            S["widest_windows"][f"{md}|{t}"] = [dict(id=r["id"], section=r["section"], span_m=r["span_m"], ws=r["ws"], width_mm=r["width_mm"],
                                                      width_pct_mac=r["width_pct_mac"], sm_range=[r["sm_lo"], r["sm_hi"]],
                                                      re_mac_range=[r["re_mac_min"], r["re_mac_max"]], cl_range=[r["cl_min"], r["cl_max"]],
                                                      ti_mean=r["ti_mean"]) for r in rs]
    # SM extent: how often does the widest 'any/both' window include CG aft of the NP (SM<0)?
    rs = [r for r in R if r["mode"] == "any" and r["tier"] == "both" and r["sm_class"] == "all" and r["n_points"]]
    S["any_both_window_includes_sm_below_0"] = dict(n=len(rs), n_lo_below_0=sum(1 for r in rs if r["sm_lo"] < -1e-9))
    # ---- Part 3 paired comparison at span 0.40
    P3 = {}
    base = {(r["id"], r["section"], r["ws"], r["mode"], r["tier"], r["sm_class"]): r for r in R if r["span_m"] == CFG["sweep_bracket_span_m"]}
    for tag in ("sweepcorr_reexp2", "sweepcorr_reexp1"):
        W = _load_windows(tag)
        sw = {(r["id"], r["section"], r["ws"], r["mode"], r["tier"], r["sm_class"]): r for r in W}
        for md in ("any", "clean", "offti"):
            for t in TIERS:
                keys = [k for k in base if k[3] == md and k[4] == t and k[5] == "all" and k in sw]
                b_hit = sum(1 for k in keys if base[k]["n_points"])
                s_hit = sum(1 for k in keys if sw[k]["n_points"])
                both_hit = [k for k in keys if base[k]["n_points"] and sw[k]["n_points"]]
                dw = [sw[k]["width_mm"] - base[k]["width_mm"] for k in both_hit]
                dcl = [0.5 * (sw[k]["cl_min"] + sw[k]["cl_max"]) - 0.5 * (base[k]["cl_min"] + base[k]["cl_max"]) for k in both_hit]
                dsm = [0.5 * (sw[k]["sm_lo"] + sw[k]["sm_hi"]) - 0.5 * (base[k]["sm_lo"] + base[k]["sm_hi"]) for k in both_hit]
                P3[f"{tag}|{md}|{t}"] = dict(cases=len(keys), hits_streamwise=b_hit, hits_sweepcorr=s_hit, hits_both=len(both_hit),
                                              max_width_mm_streamwise=max([base[k]["width_mm"] for k in keys if base[k]["n_points"]], default=None),
                                              max_width_mm_sweepcorr=max([sw[k]["width_mm"] for k in keys if sw[k]["n_points"]], default=None),
                                              median_dwidth_mm=statistics.median(dw) if dw else None,
                                              median_dcl_window_centre=statistics.median(dcl) if dcl else None,
                                              max_abs_dcl=max((abs(x) for x in dcl), default=None),
                                              median_dsm_window_centre=statistics.median(dsm) if dsm else None)
    S["p3_sweep_bracket"] = P3
    (OUT / "p2_p3_summary.json").write_text(json.dumps(S, indent=1))
    for k in ("any|both|SM>=0", "any|both|all", "clean|both|all", "offti|both|all", "clean_offti|both|all", "any|N5|all", "any|N9|all"):
        d = S["by_mode_tier_class"][k]
        print(k, {a: (round(b, 2) if isinstance(b, float) else b) for a, b in d.items()})
    for k, v in P3.items():
        if "|any|" in k or "|offti|both" in k:
            print(k, {a: (round(b, 3) if isinstance(b, float) else b) for a, b in v.items()})


# ------------------------------------------------------------------ Part 5: candidates
SHEET_WS = [6, 8, 12]
SHEET_SM = [-0.05, 0.0, 0.025, 0.05, 0.10]
RECOMMENDED = dict(dih=3, centre_fin=dict(area_frac=0.06, x_qc_over_root_chord=1.5))


def pick_candidates():
    rows = list(csv.DictReader((OUT / "p5_profile_windows_streamwise.csv").open(encoding="utf-8")))
    sc = defaultdict(list)
    for r in rows:
        if r["mode"] == "any" and r["tier"] == "both" and r["sm_class"] == "all" and float(r["ws"]) in (6, 8, 10, 12):
            sc[(r["id"], r["section"])].append(float(r["width_mm"]) if r["width_mm"] else 0.0)
    rank = sorted(((statistics.mean(v), k) for k, v in sc.items()), reverse=True)
    picks, why = [], []
    def take(pred, label):
        for sc_, (sid, sec) in rank:
            if (sid, sec) not in [(p["id"], p["section"]) for p in picks] and pred(SHAPES[sid]):
                picks.append(dict(id=sid, section=sec, score_mm=sc_, role=label))
                return
    take(lambda s: True, "widest window (any evidence, both Ncrit, mean over W/S 6-12)")
    take(lambda s: s["taper"] == 1.0, "best constant-chord (simplest to build)")
    take(lambda s: s["ar"] == 4.5, "best AR 4.5 (larger chord, higher Re)")
    return picks, rank


def _sheet_roots(sh, b, table, ws, sm):
    out = []
    for r in m.trim_roots(sh, b, table, ws, sm, st.GRID):
        cl = r["CL"]
        cdi = sh["k_cdi"] * cl ** 2
        ti = v2.ti_fraction_v2(sh, b, table, cl, r["V"])
        out.append(dict(trim_cl=cl, alpha_deg=cl / sh["cla_per_rad"] * 180 / math.pi + (r["dalpha_deg"] or 0.0), v_m_s=r["V"],
                        coupled_margin_glide=-r["dcm_dcl_glide"], coupled_margin_fixed=-r["dcm_dcl_fixed_speed"],
                        stable=bool(r["stable_glide"] and r["stable_fixed"]), re_root=r["re_root"], re_mac=r["re_mac"], re_tip=r["re_tip"],
                        coverage=r["coverage"], jagged_fraction=r["jagged_fraction"], ti_fraction=ti, cdi=cdi, cdp=r["cdp"],
                        l_over_d=(cl / (cdi + r["cdp"]) if r["cdp"] else None), in_cl_band=BAND[0] <= cl <= BAND[1]))
    return out


def stage_candidates():
    from geometry import Wing
    shapes = json.loads((OUT / "avl_shapes_v2.json").read_text())
    tables = m.load_tables()
    picks, rank = pick_candidates()
    b = CFG["sweep_bracket_span_m"]
    sheets = []
    for pk in picks:
        sp = SHAPES[pk["id"]]
        sh = shapes[skey(pk["id"])]
        w = Wing(span_m=b, aspect_ratio=sp["ar"], taper=sp["taper"], quarter_chord_sweep_deg=sp["sweep"], washout_deg=0.0,
                 dihedral_deg=0.0, section=pk["section"], cg_x_m=0.0)
        mac = sh["mac"] * b
        geom = dict(span_mm=1000 * b, ar=sp["ar"], taper=sp["taper"], area_cm2=1e4 * sh["area"] * b * b, root_chord_mm=1000 * w.root_chord,
                    tip_chord_mm=1000 * w.tip_chord, mac_mm=1000 * mac, mac_y_mm=1000 * w.mac_y, x_lemac_mm=1000 * sh["x_lemac"] * b,
                    qc_sweep_deg=sp["sweep"], le_sweep_deg=math.degrees(math.atan(w.tip_x_le / (b / 2))), section=pk["section"],
                    xnp_mm=1000 * sh["xnp"] * b, cla_per_rad=sh["cla_per_rad"], e_oswald=sh["e_oswald"])
        cgs = []
        for sm in SHEET_SM:
            cg = m.cg_from_sm(sh, b, sm)
            cgs.append(dict(sm=sm, cg_mm=cg["xcg_mm"], cg_pct_mac=100 * cg["xcg_over_mac_from_lemac"]))
        trims = []
        for ws in SHEET_WS:
            for nc in (5, 9):
                base = tables[(pk["section"], nc)]
                for variant, tb in (("streamwise", base), ("sweep_bracket", v2.SweepSection(base, sp["sweep"], 2))):
                    for sm in SHEET_SM:
                        for r in _sheet_roots(sh, b, tb, ws, sm):
                            trims.append(dict(ws=ws, mass_g=1000 * ws * sh["area"] * b * b / G, ncrit=nc, variant=variant, sm=sm, **r))
        # lateral: bare, recommended, and alternatives (CG at SM 5 %)
        xcg = sh["xnp"] - 0.05 * sh["mac"]
        lat = {}
        for name, dih, tf, cf in (("bare_dih0", 0, None, None), ("dih3", 3, None, None),
                                   ("dih3+centrefin6pct@1.5cr (RECOMMENDED)", 3, None, RECOMMENDED["centre_fin"]),
                                   ("dih0+centrefin6pct@1.5cr", 0, None, RECOMMENDED["centre_fin"]),
                                   ("dih3+centrefin3pct@1.5cr", 3, None, dict(area_frac=0.03, x_qc_over_root_chord=1.5)),
                                   ("dih3+tipfin_sym4pct", 3, dict(area_frac=0.04, mode="sym"), None)):
            r = v2.lateral_case(sp["ar"], sp["sweep"], sp["taper"], dih, xcg, OUT / "avl_lat" / pk["id"], "cand_" + name.split(" ")[0].replace("@", "_").replace("%", "p"),
                                tip_fin=tf, centre_fin=cf, cls=CFG["lateral_cls"])
            lat[name] = dict(cl_beta=r["cl_beta"], cn_beta=r["cn_beta"], xnp_mm=1000 * r["xnp"] * b, dxnp_pct_mac=100 * (r["xnp"] - sh["xnp"]) / sh["mac"])
        sheets.append(dict(**pk, geometry=geom, cg=cgs, trims=trims, lateral=lat))
    (OUT / "p5_candidates.json").write_text(json.dumps(dict(ranking_top=[(round(a, 3), k) for a, k in rank[:15]], candidates=sheets), indent=1, default=float))
    write_csv(OUT / "p5_candidate_trims.csv", [dict(id=c["id"], section=c["section"], **t) for c in sheets for t in c["trims"]])
    print("candidates:", [(c["id"], c["section"], c["role"]) for c in sheets])
    return sheets


def write_sheets_text():
    D = json.loads((OUT / "p5_candidates.json").read_text())
    L = ["CANDIDATE SHEETS (span 0.40 m, straight taper, no twist, dihedral 0 for the pitch study). Pitch numbers: Level 1 coupling; only roots with",
         "coverage>=0.9 listed. ev = evidence flags: jag = jagged-area fraction, TI = transition-interval area fraction (>=0.5 = on a transition feature).",
         "CG in mm aft of root LE; SM = AVL static margin (Xnp-Xcg)/MAC. 'sw' = simple-sweep lookup bracket (Re_n = Re cos^2 L)."]
    for c in D["candidates"]:
        g = c["geometry"]
        L += ["", f"=== {c['id']} {c['section']}  ({c['role']}) ===",
              f"span {g['span_mm']:.0f} mm  AR {g['ar']}  taper {g['taper']}  area {g['area_cm2']:.1f} cm2  root {g['root_chord_mm']:.1f} mm  tip {g['tip_chord_mm']:.1f} mm  "
              f"MAC {g['mac_mm']:.1f} mm at y={g['mac_y_mm']:.0f} mm, LE of MAC at x={g['x_lemac_mm']:.1f} mm  c/4 sweep {g['qc_sweep_deg']} deg  LE sweep {g['le_sweep_deg']:.1f} deg",
              f"AVL neutral point {g['xnp_mm']:.1f} mm ({100*(g['xnp_mm']-g['x_lemac_mm'])/g['mac_mm']:.1f}% MAC)  CLa {g['cla_per_rad']:.2f}/rad  e {g['e_oswald']:.2f}",
              "CG: " + "; ".join(f"SM {100*x['sm']:+.0f}% -> {x['cg_mm']:.1f} mm ({x['cg_pct_mac']:.1f}% MAC)" for x in c["cg"])]
        PW = {tag: list(csv.DictReader((OUT / f"p5_profile_windows_{tag}.csv").open(encoding="utf-8"))) for tag in ("streamwise", "sweepcorr_reexp2")}
        for ws in SHEET_WS:
            ts = [t for t in c["trims"] if t["ws"] == ws]
            L.append(f" W/S {ws} N/m2 (mass {ts[0]['mass_g']:.1f} g):")
            for tag, P in PW.items():
                for md in ("any", "offti"):
                    for tier in ("N5", "N9", "both"):
                        r = [x for x in P if x["id"] == c["id"] and x["section"] == c["section"] and x["mode"] == md and x["tier"] == tier
                             and x["sm_class"] == "all" and float(x["ws"]) == ws]
                        if r and r[0]["n_points"] != "0" and r[0]["sm_lo"]:
                            r = r[0]
                            L.append(f"   window[{tag[:2]},{md},{tier}]: SM {100*float(r['sm_lo']):+.1f}..{100*float(r['sm_hi']):+.1f}%  width {float(r['width_mm']):.2f} mm ({float(r['width_pct_mac']):.1f}% MAC, {r['widest_n']} pts)  "
                                     f"CG {float(r['cg_mm_aft_lo']):.1f}..{float(r['cg_mm_aft_hi']):.1f} mm  CL {float(r['cl_min']):.2f}-{float(r['cl_max']):.2f}  Re_MAC {float(r['re_mac_min'])/1000:.0f}-{float(r['re_mac_max'])/1000:.0f}k")
                        else:
                            L.append(f"   window[{tag[:2]},{md},{tier}]: none")
            for sm in SHEET_SM:
                for nc in (5, 9):
                    cell = []
                    for var in ("streamwise", "sweep_bracket"):
                        rs = [t for t in ts if t["sm"] == sm and t["ncrit"] == nc and t["variant"] == var and t["coverage"] >= 0.9]
                        cell.append(var[:2] + ": " + (" | ".join(
                            f"CL {t['trim_cl']:.3f} a {t['alpha_deg']:.1f}d V {t['v_m_s']:.1f} marg g/f {t['coupled_margin_glide']:+.3f}/{t['coupled_margin_fixed']:+.3f} "
                            f"{'STABLE' if t['stable'] else 'unstable'} Re {t['re_root']/1000:.0f}-{t['re_tip']/1000:.0f}k L/D {t['l_over_d']:.1f} ev jag {t['jagged_fraction']:.2f} TI {t['ti_fraction']:.2f}"
                            for t in rs) if rs else "no valid trim"))
                    L.append(f"   SM {100*sm:+4.0f}% N{nc}  " + "   ||   ".join(cell))
        L.append(" Lateral at CL 0.2/0.3/0.4/0.5 (CG at SM +5%):")
        for k, v in c["lateral"].items():
            L.append(f"   {k}: Cl_beta " + "/".join(f"{x:.3f}" for x in v["cl_beta"].values()) + "  Cn_beta " + "/".join(f"{x:.4f}" for x in v["cn_beta"].values())
                     + f"  dXnp {v['dxnp_pct_mac']:+.2f}% MAC")
    (OUT / "p5_candidate_sheets.txt").write_text("\n".join(L), encoding="utf-8")


def _profile_pts(P, pk, tier, ws_list, mode="any"):
    pts = []
    for ws in ws_list:
        r = [x for x in P if x["id"] == pk["id"] and x["section"] == pk["section"] and x["mode"] == mode and x["tier"] == tier
             and x["sm_class"] == "all" and float(x["ws"]) == ws]
        pts.append((ws, float(r[0]["width_mm"]) if r and r[0]["width_mm"] else 0.0))
    return pts


def stage_figs():
    shapes = json.loads((OUT / "avl_shapes_v2.json").read_text())
    tables = m.load_tables()
    PAL = st.PAL
    R = _load_windows("streamwise")
    ser = []
    for i, (md, lab) in enumerate((("any", "any evidence"), ("offti", "off transition interval"), ("clean", "clean (jag<=0.1)"))):
        for t in ("both", "N5"):
            pts = []
            for ws in CFG["ws_n_m2"]:
                w = [r["width_mm"] for r in R if r["mode"] == md and r["tier"] == t and r["sm_class"] == "all" and r["ws"] == ws and r["n_points"]]
                pts.append((ws, max(w) if w else 0.0))
            ser.append(dict(label=f"max window, {t}, {lab}", pts=pts, color=PAL[i], dash=(t == "N5"), marker="circle"))
    st.svg_chart(OUT / "fig_p2_max_window_vs_ws.svg", "Widest stable-trim CG window vs wing loading (12 shapes x 3 sections x 4 spans)", "wing loading W/S (N/m2)",
                 "widest window (mm)", ser, "Trim CL 0.2-0.5, coupled Level 1. Solid: both Ncrit; dashed: Ncrit 5 alone. Width=(n-1)*0.005*MAC; windows fall as W/S (hence Re) rises.")
    picks, _ = pick_candidates()
    ser = []
    for tag, dash in (("streamwise", False), ("sweepcorr_reexp2", True)):
        P = list(csv.DictReader((OUT / f"p5_profile_windows_{tag}.csv").open(encoding="utf-8")))
        for i, pk in enumerate(picks):
            ser.append(dict(label=f"{pk['id']} {pk['section'][-4:]} both Ncrit, {'sweep-corrected' if dash else 'streamwise'}",
                            pts=_profile_pts(P, pk, "both", PROFILE_WS), color=PAL[i], dash=dash, marker="circle"))
    P = list(csv.DictReader((OUT / "p5_profile_windows_streamwise.csv").open(encoding="utf-8")))
    for tier, col in (("N5", "#888888"), ("N9", "#bbbbbb")):
        ser.append(dict(label=f"{picks[0]['id']} {tier} alone (streamwise)", pts=_profile_pts(P, picks[0], tier, PROFILE_WS), color=col, marker="x"))
    st.svg_chart(OUT / "fig_p5_window_vs_ws_candidates.svg", "CG window vs W/S for the candidates (span 0.40 m)", "wing loading W/S (N/m2)", "stable-trim CG window (mm)", ser,
                 "Any-evidence windows, trim CL 0.2-0.5. Strict-evidence (clean or off-transition) both-Ncrit windows are ~0 mm for these candidates. Dashed = simple-sweep bracket.")
    sms = [round(-0.10 + 0.0025 * i, 4) for i in range(101)]
    for pk in picks:
        sp, sh = SHAPES[pk["id"]], shapes[skey(pk["id"])]
        b, ws = 0.40, 8
        ser = [dict(label="AVL-only (trims at CL=0)", pts=[(m.cg_from_sm(sh, b, sm)["xcg_mm"], 0.0) for sm in sms], color="#555555")]
        k = 1
        for var in ("streamwise", "sweep_bracket"):
            for nc in (5, 9):
                base = tables[(pk["section"], nc)]
                tb = base if var == "streamwise" else v2.SweepSection(base, sp["sweep"], 2)
                curve = st.build_curve(sh, b, tb, ws)
                ctuples = [(p["cl"], p["cm"], p["cov"]) for p in curve]
                stab, uns = [], []
                for sm in sms:
                    for r in m.trim_roots(sh, b, tb, ws, sm, st.GRID, curve=ctuples):
                        if r["coverage"] < 0.9:
                            continue
                        (stab if r["stable_glide"] and r["stable_fixed"] else uns).append((m.cg_from_sm(sh, b, sm)["xcg_mm"], r["CL"]))
                col = PAL[k % len(PAL)]
                ser.append(dict(label=f"N{nc} {var}: stable", pts=stab, color=col, marker="circle", line=False))
                ser.append(dict(label=f"N{nc} {var}: unstable", pts=uns, color=col, marker="x", line=False))
                k += 1
        st.svg_chart(OUT / f"fig_p5_trim_cl_vs_cg_{pk['id']}_{pk['section'][-4:]}.svg", f"Trim CL vs CG, {pk['id']} {pk['section']}, b=0.40 m, W/S={ws}",
                     "CG aft of root leading edge (mm)", "trim CL", ser,
                     "AVL-only vs coupled Level 1 (both Ncrit), streamwise lookup vs simple-sweep bracket. Only roots with coverage>=0.9; circles stable, x unstable.", ylim=(0, 1.2))
    S4 = json.loads((OUT / "p4_lateral_summary.json").read_text())
    names = [c["name"] for c in lateral_configs()]
    for qty, lo, med, hi, yl, fn, hl in (("Cn_beta", "cn_min", "cn_median", "cn_max", "Cn_beta (per rad)", "fig_p4_cn_beta_fin_options.svg", [(0.01, "target low 0.01"), (0.03, "target high 0.03")]),
                                          ("Cl_beta", None, "cl_beta_median", None, "Cl_beta (per rad), median of 12 shapes", "fig_p4_cl_beta_fin_options.svg", [])):
        ser = []
        for i, nme in enumerate(names):
            d = S4[f"{nme}|CL0.4"]
            pts = [(i + 1, d[lo]), (i + 1, d[med]), (i + 1, d[hi])] if lo else [(i + 1, d[med])]
            ser.append(dict(label=f"{i+1}: {nme}", pts=pts, color=PAL[i % len(PAL)], marker="circle"))
        st.svg_chart(OUT / fn, f"{qty} at CL 0.4 for dihedral / fin options (12 shapes)", "option number (see legend)", yl, ser,
                     "AVL, CG at SM +5%. Marker line spans min..max over the 12 planforms (median for Cl_beta). Exploratory target Cn_beta 0.01-0.03 /rad.", hlines=hl)
    print("figures written")


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    fn = {"shapes": stage_shapes, "core": stage_core, "sweep": stage_sweep, "windows": stage_windows, "lateral": stage_lateral, "profile": stage_profile, "analysis": stage_analysis,
          "candidates": lambda: (stage_candidates(), write_sheets_text()), "figs": stage_figs}
    for s in (list(fn) if what == "all" else [what]):
        fn[s]()


if __name__ == "__main__":
    main()
