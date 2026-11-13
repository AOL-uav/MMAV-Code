"""Part C + D: bounded pure-wing study, scale tables, figures (SVG), summary.

py -B rigid_glider/pure_wing_study.py shapes | core | l2 | scale | figs | summary | all
Everything is written under results/runs/rigid_glider/pure_wing/.  Not an optimisation:
a fixed list of ~46 shapes x 4 sections x 4 spans x 5 W/S x Ncrit {5,9}, CG sweep from AVL SM.
"""

from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import csv
import json
import math
import time
from collections import Counter, defaultdict
from html import escape
from multiprocessing import Pool
from pathlib import Path

import pure_wing_model as m

OUT = m.OUT
CFG = json.loads((Path(__file__).with_name("pure_wing_config.json")).read_text())
ST = CFG["study"]
G = CFG["flight"]["g_m_s2"]
BAND = tuple(ST["exploratory_cl_band"])
COV_MIN = ST["coverage_min"]
JAG_CLEAN = ST["jagged_clean_max"]
GRID = [round(0.02 + 0.01 * i, 3) for i in range(119)]            # 0.02 .. 1.20
SM_DISCRETE = [round(ST["sm_min"] + ST["sm_step"] * i, 4)
               for i in range(int(round((ST["sm_max"] - ST["sm_min"]) / ST["sm_step"])) + 1)]
SM_SCAN = [round(-0.05 + 0.005 * i, 4) for i in range(51)]
DCL = 0.02


def shape_list() -> list[tuple]:
    shapes = [(ar, sw, tp, 0) for ar in (3, 4.5, 6) for sw in (10, 20, 30, 35) for tp in (0.4, 0.7, 1.0)]
    for ar, sw, tp in ST["dihedral_bases"]:
        for di in (4, 8):
            shapes.append((ar, sw, tp, di))
    return shapes


# ------------------------------------------------------------------ shapes
def stage_shapes():
    cache = {}
    t = time.time()
    for s in shape_list():
        m.get_shape(*s, cache)
    (OUT / "avl_shapes.json").write_text(json.dumps(cache))
    l2 = {}
    for s in ST["representative_shapes"]:
        l2[m.shape_key(*s)] = m.build_l2(*s, OUT / "avl")
    (OUT / "avl_l2.json").write_text(json.dumps(l2))
    worst = {k: (v["cla_lin_err"], v["strip_f_dev"], v["xnp_spread"], abs(v["cm0"])) for k, v in cache.items()}
    print(f"{len(cache)} shapes, {len(l2)} L2 matrices in {time.time()-t:.1f}s;",
          "max CLa nonlin", max(v[0] for v in worst.values()), "max strip f dev", max(v[1] for v in worst.values()),
          "max |Cm0|", max(v[3] for v in worst.values()))


# ------------------------------------------------------------------ worker helpers
_S = {}


def _init():
    _S["shapes"] = json.loads((OUT / "avl_shapes.json").read_text())
    _S["tables"] = m.load_tables()
    if (OUT / "avl_l2.json").is_file():
        _S["l2"] = json.loads((OUT / "avl_l2.json").read_text())


def section_cell(shape, b, table, CL, V):
    """Area-weighted section cm and dcm/dcl (fixed Re) at the operating cell."""
    wsum = cmw = dw = clw = 0.0
    for s in shape["strips"]:
        c, A = s["c"] * b, s["a"] * b * b
        re = V * c / m.NU
        cl = CL * s["f"]
        r0 = table.lookup(re, cl)
        rp, rm = table.lookup(re, cl + 0.02), table.lookup(re, cl - 0.02)
        if r0 is None:
            continue
        wsum += A
        cmw += r0["cm"] * A
        clw += cl * A
        if rp is not None and rm is not None:
            dw += (rp["cm"] - rm["cm"]) / 0.04 * A
        else:
            dw = float("nan")
    if not wsum:
        return None
    return dict(cm_sec_mean=cmw / wsum, dcm_dcl_sec=dw / wsum, cl_sec_mean=clw / wsum)


def build_curve(shape, b, table, ws):
    pts = []
    for CL in GRID:
        cm, r = m.cm_extra_L1(shape, b, table, CL, ws=ws)
        pts.append([CL, cm, r["coverage"], r["jagged_fraction"], r["V"]])
    n = len(pts)
    curve = []
    for i, (CL, cm, cov, jag, V) in enumerate(pts):
        if 2 <= i < n - 2:
            dg = (pts[i + 2][1] - pts[i - 2][1]) / (4 * 0.01)
            covd = min(pts[i + 2][2], pts[i - 2][2], cov)
        else:
            dg, covd = None, 0.0
        if cov > 0 and 0.02 <= CL <= 1.15:
            fp = m.level1(shape, b, table, CL + DCL, V=V)
            fm = m.level1(shape, b, table, CL - DCL, V=V)
            df = (fp["cm_visc"] - fm["cm_visc"]) / (2 * DCL)
            covd = min(covd, fp["coverage"], fm["coverage"])
        else:
            df = None
        curve.append(dict(cl=CL, cm=cm, cov=cov, jag=jag, dg=dg, df=df, covd=covd))
    return curve


def roots_from_curve(curve, sm):
    out = []
    for a, c in zip(curve, curve[1:]):
        if a["cov"] <= 0 or c["cov"] <= 0:
            continue
        g0, g1 = a["cm"] - sm * a["cl"], c["cm"] - sm * c["cl"]
        if g0 * g1 < 0 or g0 == 0:
            t = g0 / (g0 - g1) if g0 != g1 else 0.0
            cl = a["cl"] + t * (c["cl"] - a["cl"])
            if a["dg"] is None or c["dg"] is None or a["df"] is None or c["df"] is None:
                dg = df = None
            else:
                dg = a["dg"] + t * (c["dg"] - a["dg"]) - sm
                df = a["df"] + t * (c["df"] - a["df"]) - sm
            out.append(dict(cl=cl, dg=dg, df=df, cov=min(a["cov"], c["cov"]), covd=min(a["covd"], c["covd"]),
                            jag=max(a["jag"], c["jag"])))
    return out


def is_good(r, clean):
    if r["dg"] is None or not (BAND[0] <= r["cl"] <= BAND[1]):
        return False
    if min(r["cov"], r["covd"]) < COV_MIN:
        return False
    if clean and r["jag"] > JAG_CLEAN:
        return False
    return r["dg"] < 0 and r["df"] < 0


def diagnose(curve, sm):
    band = [p for p in curve if BAND[0] <= p["cl"] <= BAND[1]]
    cov = [p for p in band if p["cov"] >= COV_MIN]
    if not cov:
        return "uncovered: <90% of area inside database Re/cl in the whole band"
    F = [p["cm"] / p["cl"] for p in cov]
    if max(F) <= 0:
        return "section cm nose-down (<=0) at all covered band CL"
    roots = [r for r in roots_from_curve(curve, sm) if BAND[0] <= r["cl"] <= BAND[1] and r["cov"] >= COV_MIN]
    if not roots:
        return "section cm positive but too weak/absent at this SM (no root in band)" if max(F) < sm else \
               "no root in band (cm_extra/CL crosses SM outside band or on uncovered CL)"
    if any(is_good(r, False) for r in roots):
        return "STABLE" if any(is_good(r, True) for r in roots) else "stable only on jagged (steep/multibranch) data"
    return "roots exist but unstable (dCm/dCL >= 0: positive cm coincides with destabilising dcm/dcl)"


def work_shape(key):
    shape = _S["shapes"][key]
    rows, scans = [], []
    diag = Counter()
    for sec in CFG["sections"]:
        for ncrit in ST["ncrit"]:
            table = _S["tables"][(sec, ncrit)]
            for b in ST["span_m"]:
                for ws in ST["wing_loading_n_m2"]:
                    curve = build_curve(shape, b, table, ws)
                    rec = dict(key=key, section=sec, ncrit=ncrit, span=b, ws=ws, any=[], clean=[], anysm_pos=0)
                    for sm in SM_SCAN:
                        rs = roots_from_curve(curve, sm)
                        if any(is_good(r, False) for r in rs):
                            rec["any"].append(sm)
                        if any(is_good(r, True) for r in rs):
                            rec["clean"].append(sm)
                    scans.append(rec)
                    for sm in (0.025, 0.05, 0.1):
                        diag[(sec, ncrit, sm, diagnose(curve, sm))] += 1
                    for sm in SM_DISCRETE:
                        roots = m.trim_roots(shape, b, table, ws, sm, GRID, curve=[(p["cl"], p["cm"], p["cov"]) for p in curve])
                        cg = m.cg_from_sm(shape, b, sm)
                        S = shape["area"] * b * b
                        for r in roots:
                            cl = r["CL"]
                            cell = section_cell(shape, b, table, cl, r["V"]) or {}
                            clb, cnb = m.lateral_at(shape, cl)
                            a_avl = cl / shape["cla_per_rad"] * 180 / math.pi
                            cdi = shape["k_cdi"] * cl ** 2
                            rows.append(dict(
                                shape=key, ar=shape["ar"], sweep=shape["sweep"], taper=shape["taper"], dih=shape["dih"],
                                section=sec, ncrit=ncrit, span_m=b, ws_n_m2=ws, mass_kg=ws * S / G, sm_avl=sm,
                                cg_mm=cg["xcg_mm"], cg_pct_mac=100 * cg["xcg_over_mac_from_lemac"], xnp_mm=cg["xnp_mm"],
                                trim_cl=cl, alpha_deg=a_avl + (r["dalpha_deg"] or 0.0), alpha_avl_deg=a_avl,
                                dcm_dcl_glide=r["dcm_dcl_glide"], dcm_dcl_fixed=r["dcm_dcl_fixed_speed"],
                                sm_coupled_glide=-r["dcm_dcl_glide"], sm_coupled_fixed=-r["dcm_dcl_fixed_speed"],
                                cm_alpha_avl_per_rad=-sm * shape["cla_per_rad"],
                                cm_alpha_coupled_glide_per_rad=r["dcm_dcl_glide"] * shape["cla_per_rad"],
                                cl_beta_per_rad=clb, cn_beta_per_rad=cnb, v_m_s=r["V"],
                                re_root=r["re_root"], re_mac=r["re_mac"], re_tip=r["re_tip"],
                                cdi=cdi, cdp=r["cdp"], l_over_d=(cl / (cdi + r["cdp"]) if r["cdp"] else None),
                                coverage=r["coverage"], deriv_coverage=r["deriv_min_coverage"],
                                jagged_fraction=r["jagged_fraction"], nbranch=r["nbranch"],
                                stable_glide=int(r["stable_glide"]), stable_fixed=int(r["stable_fixed"]),
                                valid=int(r["valid"]), in_band=int(BAND[0] <= cl <= BAND[1]),
                                clean=int(r["valid"] and r["jagged_fraction"] <= JAG_CLEAN),
                                cm_sec_mean=cell.get("cm_sec_mean"), dcm_dcl_sec=cell.get("dcm_dcl_sec"),
                                cl_sec_mean=cell.get("cl_sec_mean"), avl_only_trim_cl=m.avl_only_trim(shape, sm) if sm else None))
    return rows, scans, dict(diag)


def stage_core():
    t = time.time()
    keys = [m.shape_key(*s) for s in shape_list()]
    with Pool(8, initializer=_init) as pool:
        res = pool.map(work_shape, keys, chunksize=1)
    rows = [r for x in res for r in x[0]]
    scans = [r for x in res for r in x[1]]
    diag = Counter()
    for x in res:
        for k, v in x[2].items():
            diag[k] += v
    (OUT / "study_scans.json").write_text(json.dumps(scans))
    (OUT / "study_diagnosis.json").write_text(json.dumps([[*k, v] for k, v in sorted(diag.items())]))
    write_csv(OUT / "study_roots.csv", rows)
    print(f"core: {len(rows)} root rows, {len(scans)} curves in {time.time()-t:.0f}s")


def write_csv(path, rows):
    if not rows:
        path.write_text("")
        return
    with path.open("w", newline="", encoding="utf-8") as s:
        w = csv.DictWriter(s, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow({k: (f"{v:.6g}" if isinstance(v, float) else v) for k, v in r.items()})


def read_csv(path):
    with path.open(encoding="utf-8") as s:
        out = []
        for r in csv.DictReader(s):
            for k, v in r.items():
                if k in ("shape", "section"):
                    continue
                try:
                    r[k] = float(v)
                except (TypeError, ValueError):
                    r[k] = None
            out.append(r)
        return out


# ------------------------------------------------------------------ Level 2
REP = [m.shape_key(*s) for s in ST["representative_shapes"]]


def work_l2(key):
    shape, l2 = _S["shapes"][key], _S["l2"][key]
    out = []
    grid = [round(0.04 + 0.02 * i, 3) for i in range(56)]      # 0.04..1.14
    for sec in CFG["sections"]:
        for ncrit in ST["ncrit"]:
            table = _S["tables"][(sec, ncrit)]
            for b in ST["span_m"]:
                for ws in ST["wing_loading_n_m2"]:
                    c1, c2 = [], []
                    n_conv = n_tot = 0
                    iters = []
                    warm = None
                    for CL in grid:
                        a = m.level1(shape, b, table, CL, ws=ws)
                        fp = m.level1(shape, b, table, CL + DCL, V=a["V"])
                        fm = m.level1(shape, b, table, CL - DCL, V=a["V"])
                        c1.append(dict(cl=CL, cm=shape["cm0"] + a["cm_visc"], cov=a["coverage"], jag=a["jagged_fraction"],
                                       df=(fp["cm_visc"] - fm["cm_visc"]) / (2 * DCL), covd=min(a["coverage"], fp["coverage"], fm["coverage"])))
                        if a["coverage"] <= 0:
                            c2.append(dict(cl=CL, cm=0.0, cov=0.0, jag=0.0, df=None, covd=0.0, conv=True)); continue
                        r = m.level2(shape, l2, b, table, CL, ws, ainc0=warm)
                        warm = r["ainc"]
                        rp = m.level2(shape, l2, b, table, CL + DCL, ws, V=r["V"], ainc0=warm)
                        rm = m.level2(shape, l2, b, table, CL - DCL, ws, V=r["V"], ainc0=warm)
                        n_tot += 1
                        n_conv += int(r["converged"])
                        iters.append(r["iterations"])
                        c2.append(dict(cl=CL, cm=r["cm_extra"], cov=r["coverage"], jag=r["jagged_fraction"],
                                       df=(rp["cm_extra"] - rm["cm_extra"]) / (2 * DCL),
                                       covd=min(r["coverage"], rp["coverage"], rm["coverage"]), alpha=r["alpha_deg"],
                                       conv=r["converged"] and rp["converged"] and rm["converged"]))
                    for cv in (c1, c2):
                        for i, p in enumerate(cv):
                            if 1 <= i < len(cv) - 1:
                                p["dg"] = (cv[i + 1]["cm"] - cv[i - 1]["cm"]) / (cv[i + 1]["cl"] - cv[i - 1]["cl"])
                                p["covd"] = min(p["covd"], cv[i + 1]["cov"], cv[i - 1]["cov"])
                            else:
                                p["dg"], p["covd"] = None, 0.0
                    rec = dict(key=key, section=sec, ncrit=ncrit, span=b, ws=ws, n_conv=n_conv, n_tot=n_tot,
                               max_iter=max(iters) if iters else 0)
                    for lvl, cv in (("L1", c1), ("L2", c2)):
                        rec[lvl] = {}
                        for sm in SM_DISCRETE:
                            rs = roots_from_curve(cv, sm)
                            rec[lvl][str(sm)] = [dict(cl=r["cl"], dg=r["dg"], df=r["df"], cov=r["cov"], jag=r["jag"],
                                                      good=is_good(r, False), clean=is_good(r, True)) for r in rs]
                    # non-converged CLs near roots
                    out.append(rec)
    return out


def stage_l2():
    t = time.time()
    with Pool(8, initializer=_init) as pool:
        res = pool.map(work_l2, REP, chunksize=1)
    out = [r for x in res for r in x]
    (OUT / "l2_comparison.json").write_text(json.dumps(out))
    nc, nt = sum(r["n_conv"] for r in out), sum(r["n_tot"] for r in out)
    print(f"L2: {len(out)} curves, converged {nc}/{nt} CL points, {time.time()-t:.0f}s")


# ------------------------------------------------------------------ scale (Part D)
def scale_tables():
    sc = CFG["scale"]
    rows, bound = [], []
    lam = sc["taper"]
    bands = [0] + sc["bands"] + [1e12]
    names = ["<20k", "20-40k", "40-80k", ">80k"]
    for ar in sc["ar"]:
        mac_over_b = (2 / 3) * (2 / (ar * (1 + lam))) * (1 + lam + lam ** 2) / (1 + lam)
        for ws in ST["wing_loading_n_m2"]:
            for cl in sc["cl"]:
                V = m.glide_speed(ws, cl)
                bound.append(dict(ar=ar, ws_n_m2=ws, cl=cl, v_m_s=V,
                                  span_m_for_re_mac_10k=sc["db_re"][0] * m.NU / (V * mac_over_b),
                                  span_m_for_re_mac_20k=20000 * m.NU / (V * mac_over_b),
                                  span_m_for_re_mac_160k=sc["db_re"][1] * m.NU / (V * mac_over_b)))
                for b in sc["span_m"]:
                    S = b * b / ar
                    cr = 2 * S / (b * (1 + lam))
                    ct = lam * cr
                    frac = [0.0] * 4
                    n = 2000
                    for i in range(n):
                        eta = (i + 0.5) / n
                        c = cr + (ct - cr) * eta
                        re = V * c / m.NU
                        k = next(j for j in range(4) if bands[j] <= re < bands[j + 1])
                        frac[k] += c / (cr * (1 + lam) / 2 * n)       # area weight of strip: c / (S/(b/2)) -> mean chord
                    cov = sum(f for f, (lo, hi) in zip(frac, [(0, 0)] * 4))
                    covered = 0.0
                    for i in range(n):
                        eta = (i + 0.5) / n
                        c = cr + (ct - cr) * eta
                        if sc["db_re"][0] <= V * c / m.NU <= sc["db_re"][1]:
                            covered += c / (cr * (1 + lam) / 2 * n)
                    rows.append(dict(ar=ar, span_m=b, ws_n_m2=ws, cl=cl, v_m_s=V, mass_kg=ws * S / G,
                                     re_root=V * cr / m.NU, re_mac=V * mac_over_b * b / m.NU, re_tip=V * ct / m.NU,
                                     frac_lt20k=frac[0], frac_20_40k=frac[1], frac_40_80k=frac[2], frac_gt80k=frac[3],
                                     frac_in_database_10k_160k=covered))
    write_csv(OUT / "scale_table.csv", rows)
    write_csv(OUT / "scale_database_boundary.csv", bound)
    return rows, bound


# ------------------------------------------------------------------ lateral
def lateral_table():
    shapes = json.loads((OUT / "avl_shapes.json").read_text())
    rows = []
    for k, s in shapes.items():
        for cl in (0.2, 0.4, 0.6):
            clb, cnb = m.lateral_at(s, cl)
            rows.append(dict(shape=k, ar=s["ar"], sweep=s["sweep"], taper=s["taper"], dih=s["dih"], cl=cl,
                             cl_beta_per_rad=clb, cn_beta_per_rad=cnb, clb_slope_per_cl=s["clb_slope"],
                             cnb_slope_per_cl=s["cnb_slope"], cla_per_rad=s["cla_per_rad"], e_oswald=s["e_oswald"]))
    write_csv(OUT / "lateral_table.csv", rows)
    return rows


# ------------------------------------------------------------------ SVG
PAL = ["#1769aa", "#c55127", "#28834f", "#793da4", "#ad850d", "#555555", "#c2185b", "#00838f"]


def svg_chart(path, title, xl, yl, series, caption, logx=False, logy=False, hlines=(), xlim=None, ylim=None):
    W, H, L, R, T, B = 820, 560, 80, 30, 44, 150
    pts = [(x, y) for s in series for x, y in s["pts"] if y is not None and x is not None]
    if not pts:
        return
    tx = (lambda v: math.log10(v)) if logx else (lambda v: v)
    ty = (lambda v: math.log10(v)) if logy else (lambda v: v)
    allx = [tx(x) for x, _ in pts if not logx or x > 0]
    ally = [ty(y) for _, y in pts if not logy or y > 0] + [ty(h[0]) for h in hlines]
    x0, x1 = (tx(xlim[0]), tx(xlim[1])) if xlim else (min(allx), max(allx))
    y0, y1 = (ty(ylim[0]), ty(ylim[1])) if ylim else (min(ally), max(ally))
    if x1 == x0: x1 = x0 + 1
    if y1 == y0: y1 = y0 + 1
    px = (x1 - x0) * (0.0 if xlim else 0.04); py = (y1 - y0) * (0.0 if ylim else 0.06)
    x0, x1, y0, y1 = x0 - px, x1 + px, y0 - py, y1 + py
    pw, ph = W - L - R, H - T - B
    sx = lambda v: L + (tx(v) - x0) / (x1 - x0) * pw
    sy = lambda v: T + ph - (ty(v) - y0) / (y1 - y0) * ph
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" font-family="Arial">',
         '<rect width="100%" height="100%" fill="white"/>',
         f'<text x="{W/2}" y="24" text-anchor="middle" font-size="16">{escape(title)}</text>']
    for k in range(6):
        gx, gy = L + k * pw / 5, T + k * ph / 5
        vx, vy = x0 + k * (x1 - x0) / 5, y1 - k * (y1 - y0) / 5
        fx = 10 ** vx if logx else vx
        fy = 10 ** vy if logy else vy
        o += [f'<line x1="{gx:.1f}" y1="{T}" x2="{gx:.1f}" y2="{T+ph}" stroke="#e5e9ed"/>',
              f'<line x1="{L}" y1="{gy:.1f}" x2="{L+pw}" y2="{gy:.1f}" stroke="#e5e9ed"/>',
              f'<text x="{gx:.1f}" y="{T+ph+16}" text-anchor="middle" font-size="11">{fx:.3g}</text>',
              f'<text x="{L-6}" y="{gy+4:.1f}" text-anchor="end" font-size="11">{fy:.3g}</text>']
    o += [f'<rect x="{L}" y="{T}" width="{pw}" height="{ph}" fill="none" stroke="#222"/>',
          f'<text x="{L+pw/2}" y="{T+ph+34}" text-anchor="middle" font-size="13">{escape(xl)}</text>',
          f'<text x="18" y="{T+ph/2}" text-anchor="middle" transform="rotate(-90 18 {T+ph/2})" font-size="13">{escape(yl)}</text>']
    for hv, hl in hlines:
        if y0 <= ty(hv) <= y1:
            o.append(f'<line x1="{L}" y1="{sy(hv):.1f}" x2="{L+pw}" y2="{sy(hv):.1f}" stroke="#888" stroke-dasharray="6 4"/>')
            o.append(f'<text x="{L+pw-3}" y="{sy(hv)-3:.1f}" text-anchor="end" font-size="10" fill="#555">{escape(hl)}</text>')
    if y0 < 0 < y1 and not logy:
        o.append(f'<line x1="{L}" y1="{sy(0):.1f}" x2="{L+pw}" y2="{sy(0):.1f}" stroke="#555"/>')
    for i, s in enumerate(series):
        col = s.get("color", PAL[i % len(PAL)])
        dash = ' stroke-dasharray="6 3"' if s.get("dash") else ""
        v = [(x, y) for x, y in s["pts"] if y is not None and (not logx or x > 0) and (not logy or y > 0)]
        if s.get("line", True) and len(v) >= 2:
            o.append(f'<polyline points="{" ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in v)}" fill="none" stroke="{col}" stroke-width="2"{dash}/>')
        mk = s.get("marker")
        for x, y in v:
            if mk == "circle":
                o.append(f'<circle cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="3.5" fill="{col}"/>')
            elif mk == "hollow":
                o.append(f'<circle cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="3.5" fill="white" stroke="{col}" stroke-width="1.5"/>')
            elif mk == "x":
                o.append(f'<path d="M{sx(x)-3.5:.1f} {sy(y)-3.5:.1f}l7 7m0 -7l-7 7" stroke="{col}" stroke-width="1.5"/>')
        lx, ly = L + (i % 3) * 255, T + ph + 52 + (i // 3) * 16
        o += [f'<line x1="{lx}" y1="{ly}" x2="{lx+20}" y2="{ly}" stroke="{col}" stroke-width="2"{dash}/>',
              f'<text x="{lx+25}" y="{ly+4}" font-size="11">{escape(s["label"])}</text>']
    cap = caption
    lines = [cap[i:i + 120] for i in range(0, len(cap), 120)]
    for j, ln in enumerate(lines[:3]):
        o.append(f'<text x="{L}" y="{H-24+j*13-(len(lines)-1)*13/2:.0f}" font-size="11" fill="#333">{escape(ln)}</text>')
    o.append("</svg>")
    path.write_text("\n".join(o), encoding="utf-8")


# ------------------------------------------------------------------ figures
def fig_case(shape, sec, ncrit, b, ws, tables):
    return tables[(sec, ncrit)], b, ws


def stage_figs():
    shapes = json.loads((OUT / "avl_shapes.json").read_text())
    tables = m.load_tables()
    fc = ST["figure_case"]
    sec, b, ws = fc["section"], fc["span_m"], fc["wing_loading_n_m2"]
    sms = [round(-0.05 + 0.0025 * i, 4) for i in range(101)]
    for sk in REP:
        s = shapes[sk]
        ser1, ser2 = [], []
        ser1.append(dict(label="AVL-only (CL=0 for any CG)", pts=[(m.cg_from_sm(s, b, sm)["xcg_mm"], 0.0) for sm in sms],
                         color="#555555", marker=None))
        ser2.append(dict(label="AVL-only", pts=[(100 * sm, 0.0) for sm in sms], color="#555555"))
        for k, nc in enumerate(ST["ncrit"]):
            tb = tables[(sec, nc)]
            curve = [(cl, *[(lambda r: (s["cm0"] + r["cm_visc"], r["coverage"]))(m.level1(s, b, tb, cl, ws=ws))]) for cl in GRID]
            curve = [(a, bb[0], bb[1]) for a, bb in curve]
            pts = {"stab": [], "unst": [], "flag": []}
            pts2 = {"stab": [], "unst": [], "flag": []}
            for sm in sms:
                for r in m.trim_roots(s, b, tb, ws, sm, GRID, curve=curve):
                    kind = "flag" if not r["valid"] else ("stab" if r["stable_glide"] else "unst")
                    pts[kind].append((m.cg_from_sm(s, b, sm)["xcg_mm"], r["CL"]))
                    pts2[kind].append((100 * sm, r["CL"]))
            col = ["#1769aa", "#c55127"][k]
            for kind, mk, nm in (("stab", "circle", "stable"), ("unst", "x", "unstable"), ("flag", "hollow", "cov<0.9 (flagged)")):
                ser1.append(dict(label=f"coupled L1 Ncrit {nc}: {nm}", pts=pts[kind], color=col, marker=mk, line=False))
                ser2.append(dict(label=f"coupled L1 Ncrit {nc}: {nm}", pts=pts2[kind], color=col, marker=mk, line=False))
        cap = (f"Shape {sk}, b={b} m, W/S={ws} N/m2, {sec}. Trim CL roots of Cm=0 vs CG; AVL-only trims at CL=0 for every CG. "
               f"Circles stable (glide-line dCm/dCL<0), x unstable, hollow = coverage<0.9 (not a result).")
        svg_chart(OUT / f"fig1_trim_cl_vs_cg_{sk}.svg", f"Trim CL vs CG, {sk}", "CG aft of root leading edge (mm)", "trim CL",
                  ser1, cap, ylim=(0, 1.2))
        svg_chart(OUT / f"fig2_trim_cl_vs_sm_{sk}.svg", f"Trim CL vs static margin, {sk}", "AVL static margin (% MAC)", "trim CL",
                  ser2, cap.replace("vs CG", "vs AVL static margin"), ylim=(0, 1.2))
    # fig 3: Cm vs CL
    s = shapes[m.shape_key(*fc["shape"])]
    l2 = json.loads((OUT / "avl_l2.json").read_text())[m.shape_key(*fc["shape"])]
    sm = fc["sm"]
    ser = [dict(label=f"AVL-only (SM={sm})", pts=[(cl, -sm * cl) for cl in GRID if cl <= 0.9], color="#555555")]
    roots_txt = []
    for k, nc in enumerate(ST["ncrit"]):
        tb = tables[(sec, nc)]
        l1 = [(cl, s["cm0"] + m.level1(s, b, tb, cl, ws=ws)["cm_visc"] - sm * cl, m.level1(s, b, tb, cl, ws=ws)["coverage"]) for cl in GRID]
        col = ["#1769aa", "#c55127"][k]
        ser.append(dict(label=f"Level 1, Ncrit {nc}", pts=[(c, v) for c, v, cv in l1 if cv >= 0.5 and c <= 0.9], color=col))
        warm, l2c = None, []
        for cl in GRID:
            if cl > 0.9:
                break
            if m.level1(s, b, tb, cl, ws=ws)["coverage"] < 0.5:
                continue
            r = m.level2(s, l2, b, tb, cl, ws, ainc0=warm)
            warm = r["ainc"]
            l2c.append((cl, r["cm_extra"] - sm * cl, r["converged"]))
        ser.append(dict(label=f"Level 2, Ncrit {nc}", pts=[(c, v) for c, v, ok in l2c], color=col, dash=True))
        ser.append(dict(label=f"L2 not converged, Ncrit {nc}", pts=[(c, v) for c, v, ok in l2c if not ok], color=col, marker="x", line=False))
        for lab, curve in (("L1", l1), ("L2", l2c)):
            for (c0, v0, *_), (c1, v1, *_) in zip(curve, curve[1:]):
                if v0 * v1 < 0:
                    ser.append(dict(label=f"{lab} root N{nc}", pts=[(c0 + v0 / (v0 - v1) * (c1 - c0), 0.0)], color=col, marker="circle" if lab == "L1" else "hollow", line=False))
    svg_chart(OUT / "fig3_cm_vs_cl.svg", f"Cm vs CL, {m.shape_key(*fc['shape'])}, b={b} m, W/S={ws}, {sec}, SM={sm}",
              "CL (glide line: V and Re vary with CL)", "total Cm about CG", ser,
              "AVL-only vs coupled Level 1 (solid) vs Level 2 (dashed); roots as dots (zero crossings). Negative slope = stable.",
              ylim=(-0.06, 0.04))
    # fig 4 lateral
    lat = lateral_table()
    def pick(**kw):
        return [r for r in lat if all(r[k] == v for k, v in kw.items())]
    for qty, nm in (("cl_beta_per_rad", "Cl_beta"), ("cn_beta_per_rad", "Cn_beta")):
        ser = []
        for ar in (3, 4.5, 6):
            pts = sorted((r["sweep"], r[qty]) for r in pick(ar=ar, taper=0.7, dih=0, cl=0.4))
            ser.append(dict(label=f"AR {ar}, taper 0.7, CL=0.4", pts=pts, marker="circle"))
        svg_chart(OUT / f"fig4_{nm}_vs_sweep.svg", f"{nm} vs quarter-chord sweep (dihedral 0)", "sweep (deg)", f"{nm} (per rad)", ser,
                  "AVL stability-axis derivative at CL=0.4 for taper 0.7, dihedral 0. Negative Cl_beta restoring; positive Cn_beta weathervane.")
        ser = []
        for i, (ar, sw, tp) in enumerate(ST["dihedral_bases"]):
            pts = sorted((r["dih"], r[qty]) for r in pick(ar=ar, sweep=sw, taper=tp, cl=0.4))
            ser.append(dict(label=f"AR {ar}, sweep {sw}, taper {tp}", pts=pts, marker="circle"))
        svg_chart(OUT / f"fig4_{nm}_vs_dihedral.svg", f"{nm} vs dihedral", "dihedral (deg)", f"{nm} (per rad)", ser,
                  "AVL stability-axis derivative at CL=0.4 for the five shapes that carry dihedral variants.")
    # fig 5
    rows, bound = scale_tables()
    sel = [r for r in rows if r["ar"] == 4.5 and r["cl"] == 0.4]
    hl = [(10000, "database lower edge 10k"), (20000, "20k"), (40000, "40k"), (80000, "80k"), (160000, "database upper edge 160k")]
    ser = [dict(label=f"W/S {ws} N/m2", pts=sorted((r["span_m"], r["re_mac"]) for r in sel if r["ws_n_m2"] == ws), marker="circle")
           for ws in ST["wing_loading_n_m2"]]
    svg_chart(OUT / "fig5a_re_vs_span.svg", "MAC Reynolds number vs span (AR 4.5, taper 0.7, CL 0.4)", "span (m)", "Re at MAC", ser,
              "Re = V c/nu with V=sqrt(2 W/S/(rho CL)). Dashed: database edges and Re band limits (<20k, 20-40k, 40-80k, >80k).",
              logy=True, hlines=hl, ylim=(3000, 400000))
    ser = [dict(label=f"W/S {ws} N/m2", pts=[(b_, m.glide_speed(ws, 0.4)) for b_ in CFG["scale"]["span_m"]], marker="circle") for ws in ST["wing_loading_n_m2"]]
    svg_chart(OUT / "fig5b_speed_vs_span.svg", "Glide speed vs span at CL 0.4", "span (m)", "V (m/s)", ser,
              "At fixed CL and W/S the speed does not depend on span (V=sqrt(2 W/S/(rho CL))); span only scales chord, hence Re. Lines are flat by physics.",
              ylim=(0, 14))
    ser = [dict(label=f"AR {ar}, W/S {ws}", pts=sorted((r["span_m"], 1000 * r["mass_kg"]) for r in rows if r["ar"] == ar and r["cl"] == 0.4 and r["ws_n_m2"] == ws),
                marker="circle") for ar in (3, 6) for ws in (4, 12)]
    svg_chart(OUT / "fig5c_mass_vs_span.svg", "All-up mass implied by W/S and span", "span (m)", "mass (g)", ser,
              "mass = (W/S) b^2/(AR g); exploratory W/S values, NOT measured masses.", logy=True)
    # fig 6 section cm(cl)
    from pure_wing_polars import read_database
    visc, inv = read_database()
    for sec_ in CFG["sections"]:
        ser = []
        for i, re in enumerate((10000, 20000, 40000, 80000, 160000)):
            for nc, dash in ((5, False), (9, True)):
                rows_ = visc.get((sec_, nc, re), [])
                ser.append(dict(label=f"Re {re//1000}k Ncrit {nc}", pts=[(r["cl"], r["cm"]) for r in rows_ if r["alpha"] >= 0],
                                color=PAL[i], dash=dash, marker="circle"))
        note = " (thin-plate PROXY, not a flat plate)" if sec_ == "NACA_0004" else ""
        svg_chart(OUT / f"fig6_section_cm_{sec_}.svg", f"{sec_}{note}: section cm (c/4) vs cl", "section cl", "section cm about quarter chord", ser,
                  "Raw XFOIL points alpha 0..10 deg in alpha order (solid Ncrit 5, dashed Ncrit 9); jumps are laminar-bubble bursts; positive = nose-up.")
    print("figures written")


# ------------------------------------------------------------------ summary
def stage_summary():
    scans = json.loads((OUT / "study_scans.json").read_text())
    shapes = json.loads((OUT / "avl_shapes.json").read_text())
    rows = read_csv(OUT / "study_roots.csv")
    S = {"assumptions": m.ASSUMPTIONS, "band": BAND, "coverage_min": COV_MIN, "jagged_clean_max": JAG_CLEAN}
    S["n_curves"] = len(scans)
    S["n_shapes"] = len(shapes)
    S["n_root_rows"] = len(rows)
    # Region analysis from scans (SM grid 0.005)
    def groups(flag, smmin):
        idx = defaultdict(dict)
        for r in scans:
            idx[(r["key"], r["section"], r["span"], r["ws"])][r["ncrit"]] = set(x for x in r[flag] if x >= smmin - 1e-9)
        return idx
    for flag in ("any", "clean"):
        for smmin, tag in ((0.0, "SM>=0"), (0.025, "SM>=0.025")):
            idx = groups(flag, smmin)
            n_any5 = sum(1 for v in idx.values() if v.get(5))
            n_any9 = sum(1 for v in idx.values() if v.get(9))
            both = {k: v[5] & v[9] for k, v in idx.items() if v.get(5) is not None and v.get(9) is not None and (v[5] & v[9])}
            S[f"stable_inband_{flag}_{tag}"] = dict(cases=len(idx), with_N5=n_any5, with_N9=n_any9, with_both=len(both))
            # width stats for both
            wid = []
            for k, v in both.items():
                mac = shapes[k[0]]["mac"] * k[2]
                wid.append(dict(key=k, sm_min=min(v), sm_max=max(v), n_sm=len(v), width_sm=0.005 * (len(v) - 1) if len(v) > 1 else 0.0,
                                width_mm=1000 * mac * 0.005 * len(v), mac_mm=1000 * mac))
            wid.sort(key=lambda w: -w["n_sm"])
            S[f"top_both_{flag}_{tag}"] = [dict(shape=w["key"][0], section=w["key"][1], span_m=w["key"][2], ws=w["key"][3],
                                                sm_range=[w["sm_min"], w["sm_max"]], n_sm_points=w["n_sm"],
                                                cg_width_mm=round(w["width_mm"], 2), cg_width_pct_mac=round(0.5 * w["n_sm"], 1))
                                           for w in wid[:15]]
            # W/S extent for a (shape, section, span)
            ws_ext = defaultdict(set)
            for k in both:
                ws_ext[(k[0], k[1], k[2])].add(k[3])
            S[f"ws_extent_both_{flag}_{tag}_top"] = sorted(((list(k), sorted(v)) for k, v in ws_ext.items()), key=lambda t: -len(t[1]))[:10]
            # by section / span / ws
            for dim, pos in (("section", 1), ("span", 2), ("ws", 3)):
                c = Counter(k[pos] for k in both)
                S[f"both_by_{dim}_{flag}_{tag}"] = {str(k): v for k, v in sorted(c.items())}
    # width statistics (SM scan step 0.5 % => width resolution 0.5 % MAC) for clean roots, SM>=0.025
    import statistics
    idx = groups("clean", 0.025)
    st_ = {}
    for nm, fn in (("N5", lambda v: v.get(5, set())), ("N9", lambda v: v.get(9, set())),
                   ("both", lambda v: v.get(5, set()) & v.get(9, set()))):
        ws_ = [(k, fn(v)) for k, v in idx.items() if fn(v)]
        if ws_:
            pct = [0.5 * len(s) for _, s in ws_]
            mm = [1000 * 0.005 * len(s) * shapes[k[0]]["mac"] * k[2] for k, s in ws_]
            st_[nm] = dict(cases=len(ws_), median_width_pct_mac=statistics.median(pct), max_width_pct_mac=max(pct),
                           median_width_mm=statistics.median(mm), max_width_mm=max(mm),
                           sm_min=min(min(s) for _, s in ws_), sm_max=max(max(s) for _, s in ws_),
                           by_section={sec: sum(1 for k, _ in ws_ if k[1] == sec) for sec in CFG["sections"]})
    S["clean_stable_width_stats_SM_ge_0p025"] = st_
    # W/S extent for the both-Ncrit clean set: number of W/S values per (shape,section,span)
    ext = defaultdict(set)
    for k, v in idx.items():
        if v.get(5, set()) & v.get(9, set()):
            ext[(k[0], k[1], k[2])].add(k[3])
    S["clean_both_ws_extent_histogram"] = dict(Counter(len(v) for v in ext.values()))
    # discrete rows
    good =[r for r in rows if r["in_band"] and r["valid"] and r["stable_glide"] and r["stable_fixed"] and r["sm_avl"] >= 0]
    S["discrete_stable_inband_valid_rows_SM_ge_0"] = len(good)
    S["discrete_stable_inband_valid_clean_rows_SM_ge_0"] = len([r for r in good if r["clean"]])
    # why: section cell at good roots
    if good:
        S["good_root_section_cell"] = dict(
            mean_cm_sec=sum(r["cm_sec_mean"] for r in good) / len(good),
            frac_cm_positive=sum(1 for r in good if r["cm_sec_mean"] > 0) / len(good),
            frac_dcmdcl_sec_negative=sum(1 for r in good if r["dcm_dcl_sec"] is not None and r["dcm_dcl_sec"] < 0) / len(good),
            mean_dcm_dcl_sec=sum(r["dcm_dcl_sec"] for r in good if r["dcm_dcl_sec"] is not None and r["dcm_dcl_sec"] == r["dcm_dcl_sec"]) / max(1, len([r for r in good if r["dcm_dcl_sec"] is not None and r["dcm_dcl_sec"] == r["dcm_dcl_sec"]])))
    # diagnosis table
    diag = json.loads((OUT / "study_diagnosis.json").read_text())
    agg = defaultdict(Counter)
    for sec, nc, sm, reason, n in diag:
        agg[f"SM={sm}"][reason] += n
    S["diagnosis_by_sm"] = {k: dict(v) for k, v in agg.items()}
    agg2 = defaultdict(Counter)
    for sec, nc, sm, reason, n in diag:
        if sm == 0.05:
            agg2[f"{sec}|N{nc}"][reason] += n
    S["diagnosis_sm0p05_by_section_ncrit"] = {k: dict(v) for k, v in agg2.items()}
    # unstable roots with negative cm
    # L2 comparison
    if (OUT / "l2_comparison.json").is_file():
        l2 = json.loads((OUT / "l2_comparison.json").read_text())
        nc, nt = sum(r["n_conv"] for r in l2), sum(r["n_tot"] for r in l2)
        cnt = Counter()
        for r in l2:
            for sm in SM_DISCRETE:
                if sm < 0:
                    continue
                g1 = any(x["good"] for x in r["L1"][str(sm)])
                g2 = any(x["good"] for x in r["L2"][str(sm)])
                cnt[(r["ncrit"], g1, g2)] += 1
        byk = defaultdict(dict)
        for r in l2:
            for sm in SM_DISCRETE:
                if sm >= 0.025:
                    byk[(r["key"], r["section"], r["span"], r["ws"], sm)][r["ncrit"]] = (
                        any(x["clean"] for x in r["L1"][str(sm)]), any(x["clean"] for x in r["L2"][str(sm)]))
        cc = Counter()
        for v in byk.values():
            cc[(f"N5 L1={v[5][0]} L2={v[5][1]}", )] += 0
            cc[("both_ncrit_clean", "L1", v[5][0] and v[9][0], "L2", v[5][1] and v[9][1])] += 1
            cc[("N5_clean", "L1", v[5][0], "L2", v[5][1])] += 1
            cc[("N9_clean", "L1", v[9][0], "L2", v[9][1])] += 1
        S["level2_clean_counts"] = {" ".join(map(str, k)): v for k, v in sorted(cc.items(), key=str) if v}
        S["level2"] = dict(curves=len(l2), converged_cl_points=nc, total_cl_points=nt, frac_converged=nc / nt if nt else None,
                           max_iterations_seen=max(r["max_iter"] for r in l2),
                           stable_inband_L1_vs_L2_counts={f"N{k[0]}|L1={k[1]}|L2={k[2]}": v for k, v in sorted(cnt.items())})
    (OUT / "summary.json").write_text(json.dumps(S, indent=1, default=str))
    print(json.dumps({k: S[k] for k in S if k.startswith("stable_inband") or k.startswith("discrete") or k == "level2"}, indent=1, default=str))


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    steps = ["shapes", "core", "l2", "scale", "figs", "summary"] if what == "all" else [what]
    for st in steps:
        {"shapes": stage_shapes, "core": stage_core, "l2": stage_l2, "scale": scale_tables,
         "figs": stage_figs, "summary": stage_summary}[st]()


if __name__ == "__main__":
    main()
