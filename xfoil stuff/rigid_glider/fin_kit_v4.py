"""Fallback fin kit for swift outlines A and B (bird_outline_v4_swift): tip fins or a centre fin, sized with AVL.

   py -B fin_kit_v4.py            full run (about 55 serial AVL runs, 6-10 min), writes results/runs/rigid_glider/fin_kit_v4/
   py -B fin_kit_v4.py --render   re-render the page, csv and templates from the saved runs.json (no AVL)

Imports the geometry functions of bird_outline_v4_swift (its own outputs are NOT touched).  Standard library only.
All numbers are MODEL (inviscid AVL, outline only) unless tagged otherwise.  AVL 3.52 is run serially; the lateral eigenvalues are computed here from
the system matrix AVL writes (the AVL eigen-solver / root-locus plot crashes in this build, the matrix output works).
"""
from __future__ import annotations
import sys
sys.dont_write_bytecode = True
import argparse
import csv
import html
import json
import math
import subprocess
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import bird_outline as bo
import bird_outline_v2 as v2
import bird_outline_v4_swift as v4
import flat_test as ft
from avl import _field

ROOT = ft.ROOT
OUT = ROOT / "results/runs/rigid_glider/fin_kit_v4"
AVL_EXE = ROOT / "tools/avl/avl352.exe"
SPAN0 = 250.0                         # AVL coefficient runs at 250 mm; coefficients are scale-free
RHO = 1.225
GRAV = 9.81
FIN_AR = 1.25                         # fin height / chord (JUDGEMENT: the brief gives 1 to 1.5)
TIP_ETA = 0.95                        # tip fin mounted at this fraction of the wing from the body edge (JUDGEMENT; the tip chord is too short to carry a fin)
CHORD_CAP = 0.90                      # fin chord at most this fraction of the local wing chord (tip fins) / of the hinge position (centre fin)
HINGE_CLEAR_MM = 1.0                  # centre fin trailing edge this far ahead of the hinge line at 250 mm span (JUDGEMENT)
TAB_MM = 4.0                          # glue / tape tab along the fin base at 250 mm span, scaled with the span (JUDGEMENT)
SM_TRIM = 0.10                        # static margin the build files use
BENDS = {"A": 6.0, "B": 8.0}          # nominal gauge bends of the v4 builds (read back and checked against v4's csv in the tests)
DELTA_CN = {"floor": None, "x15": None}  # filled from the baseline: increments that bring A to 0.004 and 0.006
CN_FLOOR = 0.004
CN_15 = 1.5 * CN_FLOOR
SWEEP = {"tip": (0.5, 1.0, 2.0, 3.0, 4.0, 6.0), "centre": (1.0, 2.0, 4.0, 6.0, 8.0, 12.0), "tail": (1.0, 2.0, 3.0, 4.0, 6.0, 8.0)}      # total fin area, % of the wing area S
OPTIONS = ("tip", "centre", "tail")
BUILD_AREA_PCT = 2.0                   # S3: total fin area, % of the wing area (JUDGEMENT): fin height about 10 mm at 250 mm span, easy to cut and glue
SIZES = v2.SIZES                      # printed 250 mm (PLA plate), foam 400 mm


def outlines():
    return {"A": v4.GLIDE, "B": v4.hand_shift(v4.GLIDE, 8)}


# =============================================================================================== fin geometry
def fin_geometry(g, option, area_frac):
    """Fin(s) at span 1 (b = 1).  area_frac = total fin area / wing area S.  Returns dict(option, n, area_each, c, h, x_te, x_le, y, ar, capped)."""
    A_tot = area_frac * g["S"]
    n = 2 if option == "tip" else 1
    A = A_tot / n
    if option == "tip":
        yf = g["y_body"] + TIP_ETA * (g["half_span"] - g["y_body"])
        cloc = g["chord_fn"](yf)
        x_te = g["le_fn"](yf) + cloc
        cmax = CHORD_CAP * cloc
    elif option == "centre":
        yf = 0.0
        x_te = g["x_h"] - HINGE_CLEAR_MM / SPAN0
        cmax = CHORD_CAP * x_te
    else:                                                     # "tail": centre fin glued to the tail centreline, aft of the hinge
        yf = 0.0
        x_te = g["tail_end_x"] - HINGE_CLEAR_MM / SPAN0
        cmax = CHORD_CAP * (g["tail_end_x"] - g["x_h"])
    c = math.sqrt(A / FIN_AR)
    capped = c > cmax
    c = min(c, cmax)
    h = A / c
    return dict(option=option, n=n, area_each=A, c=c, h=h, x_te=x_te, x_le=x_te - c, y=yf, ar=h / c, capped=capped, area_total=A_tot)


def fin_deck_text(fin, k):
    """AVL surface text for the fin (k = span in m)."""
    f = lambda v: f"{v * k:.9f}"
    if fin is None:
        return ""
    L = ["SURFACE", {"tip": "TipFin", "centre": "CentreFin", "tail": "TailFin"}[fin["option"]], "5 1.0 8 -2.0"]
    if fin["option"] == "tip":
        L += ["YDUPLICATE", "0.0"]
    for z in (0.0, fin["h"]):
        L += ["SECTION", f"{f(fin['x_le'])} {f(fin['y'])} {f(z)} {f(fin['c'])} 0.0", "NACA", "0000"]
    return "\n".join(L) + "\n"


def deck_for(g, bend, fin, cg_m, k=SPAN0 / 1000.0, name="finkit"):
    v2.AVL_NCHORD, v2.AVL_NSPAN = 10, 80
    lines = v2.avl_deck2(g, k * 1000.0, bend, name=name).split("\n")
    lines[4] = f"{cg_m:.9f} 0.0 0.0"                                  # moment reference = the CG
    return "\n".join(lines).rstrip("\n") + "\n" + fin_deck_text(fin, k)


# =============================================================================================== mass file and eigenvalues
_MOM_CACHE = {}


def plate_moments(g):
    """Second moments of the sheet outline per unit area at span 1: (<y^2>, <(x-xc)^2>, xc) by point sampling."""
    key = id(g)
    if key in _MOM_CACHE:
        return _MOM_CACHE[key]
    poly = g["full_outline"]
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    n = 140
    pts = []
    for i in range(n):
        x = min(xs) + (i + 0.5) * (max(xs) - min(xs)) / n
        for j in range(n):
            y = min(ys) + (j + 0.5) * (max(ys) - min(ys)) / n
            if _inside(poly, x, y):
                pts.append((x, y))
    xc = sum(p[0] for p in pts) / len(pts)
    r = (sum(p[1] ** 2 for p in pts) / len(pts), sum((p[0] - xc) ** 2 for p in pts) / len(pts), xc)
    _MOM_CACHE[key] = r
    return r


def _inside(poly, x, y):
    c = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            c = not c
        j = i
    return c


def mass_items(g, n, fin, span_mm, size_key, fin_mass_g):
    """Mass items (kg, m) for the AVL mass file: sheet (with plate inertia), rail + ballast point masses, fins.  n = v2.size_numbers() of the bare build."""
    k = span_mm / 1000.0
    y2, x2, xc = plate_moments(g)
    ms = n["m_sheet"] * 1e-3
    items = [(ms, xc * k, 0.0, 0.0, ms * y2 * k * k, ms * x2 * k * k, ms * (y2 + x2) * k * k)]
    rail = n["rail"]["mass_g"] * 1e-3
    items.append((rail, n["rail"]["cx"] * 1e-3, 0, 0, 0, 0, 0))
    if n["ballast_g"] > 0 and n["station"] is not None:
        items.append((n["ballast_g"] * 1e-3, n["station"] * 1e-3, 0, 0, 0, 0, 0))
    if fin is not None and fin_mass_g > 0:
        m = fin_mass_g * 1e-3 / fin["n"]
        for s in ([1, -1] if fin["option"] == "tip" else [0]):
            xf = (fin["x_te"] - fin["c"] / 2) * k
            yf = s * fin["y"] * k
            zf = fin["h"] * k / 2
            items.append((m, xf, yf, zf, m * (fin["h"] * k) ** 2 / 12, 0.0, m * (fin["c"] * k) ** 2 / 12))
    return items


def mass_text(items):
    L = ["Lunit = 1.0 m", "Munit = 1.0 kg", "Tunit = 1.0 s", f"g = {GRAV}", f"rho = {RHO}", "# mass x y z Ixx Iyy Izz"]
    for m, x, y, z, ixx, iyy, izz in items:
        L.append(f"{m:.9e} {x:.9f} {y:.9f} {z:.9f} {ixx:.6e} {iyy:.6e} {izz:.6e}")
    return "\n".join(L) + "\n"


def parse_sysmat(path):
    rows = []
    for line in Path(path).read_text().splitlines():
        q = line.replace("|", " ").split()
        try:
            v = [float(x) for x in q]
        except ValueError:
            continue
        if len(v) >= 12:
            rows.append(v[:12])
    if len(rows) < 12:
        raise RuntimeError("system matrix not parsed")
    return rows[:12]


def eig_roots(a):
    """Eigenvalues of a small real square matrix: characteristic polynomial (Faddeev-LeVerrier) and Durand-Kerner roots."""
    n = len(a)
    I = [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]
    coef = [1.0]
    M = [[0.0] * n for _ in range(n)]
    for k in range(1, n + 1):
        M = [[sum(a[i][t] * M[t][j] for t in range(n)) + coef[-1] * I[i][j] for j in range(n)] for i in range(n)]
        AM = [[sum(a[i][t] * M[t][j] for t in range(n)) for j in range(n)] for i in range(n)]
        coef.append(-sum(AM[i][i] for i in range(n)) / k)
    # p(s) = s^n + coef[1] s^(n-1) + ... + coef[n]
    scale = max(abs(c) ** (1.0 / i) for i, c in enumerate(coef) if i > 0 and c != 0)
    z = [complex(0.4, 0.9) ** i * scale for i in range(n)]
    for _ in range(500):
        new = []
        for i in range(n):
            p = sum(c * z[i] ** (n - j) for j, c in enumerate(coef))
            d = 1.0
            for j in range(n):
                if j != i:
                    d *= (z[i] - z[j])
            new.append(z[i] - p / d)
        if max(abs(new[i] - z[i]) for i in range(n)) < 1e-12 * scale:
            z = new
            break
        z = new
    return z


def lateral_modes(sysmat):
    """Roll subsidence, spiral and Dutch roll from the 4x4 lateral block (v p r phi) of AVL's system matrix (state order u w q theta v p r phi ...)."""
    idx = (4, 5, 6, 7)
    a = [[sysmat[i][j] for j in idx] for i in idx]
    z = eig_roots(a)
    real = sorted([r.real for r in z if abs(r.imag) < 1e-6 * max(1.0, abs(r))])
    cplx = [r for r in z if r.imag > 1e-6 * max(1.0, abs(r))]
    out = dict(eigs=[(r.real, r.imag) for r in z])
    if len(real) == 2 and cplx:
        out["roll"], out["spiral"] = real[0], real[1]
        dr = cplx[0]
        out["dr_wn"] = abs(dr)
        out["dr_zeta"] = -dr.real / abs(dr)
        out["dr_re"] = dr.real
    else:
        out["roll"] = out["spiral"] = None
    s = out.get("spiral")
    out["t_double"] = (math.log(2) / s) if (s is not None and s > 0) else None
    return out


# =============================================================================================== AVL driver
def run_avl_case(deck, workdir: Path, name, mass=None, V=None):
    workdir.mkdir(parents=True, exist_ok=True)
    for f in ("a.ft", "a.st", "sys.out", f"{name}.mass"):
        (workdir / f).unlink(missing_ok=True)
    (workdir / f"{name}.avl").write_text(deck, encoding="ascii")
    cmds = ["PLOP", "G", ""]
    if mass:
        (workdir / f"{name}.mass").write_text(mass, encoding="ascii")          # AVL loads <deck name>.mass at start-up and applies it to the run case
    cmds += ["OPER", "A C 0.3", "X", "FT", "a.ft", "ST", "a.st", ""]
    if mass:
        cmds += ["MODE", "M", f"V {V:.4f}", "", "S", "sys.out", "", ""]
    cmds += ["QUIT"]
    script = "\n".join(cmds) + "\n"
    r = subprocess.run([str(AVL_EXE), f"{name}.avl"], input=script, text=True, cwd=workdir, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=180)
    (workdir / "avl.log").write_text(r.stdout, encoding="utf-8", errors="replace")
    stt = workdir / "a.st"
    if not (workdir / "a.ft").is_file() or not stt.is_file():
        raise RuntimeError(f"AVL gave no output; see {workdir / 'avl.log'}")
    ft_t, st_t = (workdir / "a.ft").read_text(errors="replace"), stt.read_text(errors="replace")
    res = dict(cl=_field(ft_t, "CLtot"), cm=_field(ft_t, "Cmtot"), xnp=_field(st_t, "Xnp"), cnb=_field(st_t, "Cnb"), clb=_field(st_t, "Clb"), cla=_field(st_t, "CLa"))
    if mass:
        res["modes"] = lateral_modes(parse_sysmat(workdir / "sys.out"))
    return res


# =============================================================================================== masses (v4 assumptions)
def fin_mass(fin, size_key):
    """Fin mass, g, including the glue tab: foam = areal mass of the v4 foamboard assumption; printed = PLA 100 % solid at the v4 plate thickness."""
    sz = SIZES[size_key]
    k = sz["span_mm"]
    tab = TAB_MM * k / SPAN0 / k                      # tab width in span units
    area_mm2 = fin["n"] * fin["c"] * (fin["h"] + tab) * k * k
    if size_key == "foam":
        return area_mm2 * 1e-6 * ft.FOAM_AREAL_MASS_G_M2
    return ft.mass_g(area_mm2 * sz["thickness_mm"])


def base_numbers(g, size_key, cg_span, bend):
    """v4's mass / CG / ballast arithmetic for the bare build at the trim CG."""
    res = v4.fake_res(cg_span, SM_TRIM)
    return v2.size_numbers(g, res, size_key, bend)


def cg_change(n, fin, size_key, d_np_span):
    """CG target moves aft with the NP (the margin is kept); the fin mass moves the CG too.  Returns dict(fin_g, dcg_np_mm, cg_mass_only_mm, extra_ballast_g, M_new)."""
    k = SIZES[size_key]["span_mm"]
    mf = fin_mass(fin, size_key) if fin else 0.0
    xf = (fin["x_te"] - fin["c"] / 2) * k if fin else 0.0
    M, cg_t, xs = n["M"], n["cg_target"], n["station"]
    cg_new_t = cg_t + d_np_span * k
    cg_mass_only = (M * cg_t + mf * xf) / (M + mf) - cg_t
    extra = ((M + mf) * cg_new_t - M * cg_t - mf * xf) / (xs - cg_new_t) if xs is not None else float("nan")
    return dict(fin_g=mf, dcg_np_mm=d_np_span * k, cg_mass_only_mm=cg_mass_only, extra_ballast_g=extra, M_new=M + mf + extra, M_old=M, cg_target=cg_t, cg_new_target=cg_new_t, station=xs)


# =============================================================================================== the study
def run_study(out: Path, log=print):
    work = out / "avl"
    R = {"meta": dict(fin_ar=FIN_AR, tip_eta=TIP_ETA, tab_mm=TAB_MM, sweep=SWEEP, n_avl=0, build_area_pct=BUILD_AREA_PCT)}
    cnt = lambda: R["meta"].__setitem__("n_avl", R["meta"]["n_avl"] + 1)
    P = outlines()
    G = {k: v4.build_geometry4(p) for k, p in P.items()}

    def eig_case(k, fin, sk, wd, name):
        g, bend = G[k], BENDS[k]
        szm = SIZES[sk]["span_mm"]
        n = base_numbers(g, sk, R[k]["base"]["cg_span"], bend)
        fm = fin_mass(fin, sk) if fin else 0.0
        items = mass_items(g, n, fin, szm, sk, fm)
        M = sum(i[0] for i in items)
        S = g["S"] * (szm / 1000) ** 2
        V = math.sqrt(2 * M * GRAV / (RHO * S * 0.3))
        r = run_avl_case(deck_for(g, bend, fin, cg_m=R[k]["base"]["cg_span"] * szm / 1000, k=szm / 1000), wd, name, mass=mass_text(items), V=V)
        cnt()
        d_np = r["xnp"] / (szm / 1000) - R[k]["base"]["np_span"]
        return dict(r=r, V=V, M=M, cgc=cg_change(n, fin, sk, d_np), n_base=dict(M=n["M"], cg_target=n["cg_target"], station=n["station"], ballast_g=n["ballast_g"], m_sheet=n["m_sheet"]))

    for k in "AB":
        g = G[k]
        bend = BENDS[k]
        R[k] = {}
        d0 = deck_for(g, bend, None, g["x_ref"] * SPAN0 / 1000)
        b0 = run_avl_case(d0, work / k / "bare0", "bare0")
        cnt()
        R[k]["at_xref"] = b0
        cg_span = b0["xnp"] / (SPAN0 / 1000) - SM_TRIM * g["mac"]
        cg_m = cg_span * SPAN0 / 1000
        base = run_avl_case(deck_for(g, bend, None, cg_m), work / k / "bare", "bare")
        cnt()
        base.update(cg_span=cg_span, mac=g["mac"], np_span=base["xnp"] / (SPAN0 / 1000))
        R[k]["base"] = base
        log(f"{k} base: Cn {base['cnb']:+.5f} Cl {base['clb']:+.5f} NP {base['xnp'] * 1000:.2f} mm CG {cg_m * 1000:.2f} mm cm {base['cm']:+.4f}")
        R[k]["sweep"] = {}
        for opt in OPTIONS:
            rows = []
            for f_pct in SWEEP[opt]:
                fin = fin_geometry(g, opt, f_pct / 100)
                r = run_avl_case(deck_for(g, bend, fin, cg_m), work / k / f"{opt}_{f_pct:g}", "fin")
                cnt()
                rows.append(dict(f_pct=f_pct, fin=fin, **r, d_np_mac_pct=100 * (r["xnp"] - base["xnp"]) / (g["mac"] * SPAN0 / 1000)))
                log(f"  {k} {opt} {f_pct:g}%: Cn {r['cnb']:+.5f} Cl {r['clb']:+.5f} dNP {rows[-1]['d_np_mac_pct']:+.2f}% MAC (c {fin['c'] * SPAN0:.1f} h {fin['h'] * SPAN0:.1f} mm)")
            R[k]["sweep"][opt] = rows
    cnA = R["A"]["base"]["cnb"]
    R["delta"] = dict(floor=CN_FLOOR - cnA, x15=CN_15 - cnA)
    log(f"increments {R['delta']}")
    for k in "AB":
        g = G[k]
        R[k]["chosen"] = {}
        for opt in ("tip", "tail"):
            rows = R[k]["sweep"][opt]
            cn0 = R[k]["base"]["cnb"]
            for lab, area in (("floor", interp_area(rows, cn0 + R["delta"]["floor"], cn0)), ("x15", interp_area(rows, cn0 + R["delta"]["x15"], cn0)), ("build", BUILD_AREA_PCT)):
                fin = fin_geometry(g, opt, area / 100)
                entry = dict(f_pct=area, fin=fin, per_size={}, extrapolated=area > max(SWEEP[opt]))
                for sk in ("foam", "printed"):
                    entry["per_size"][sk] = eig_case(k, fin, sk, work / k / f"{opt}_{lab}_{sk}", "chosen")
                R[k]["chosen"][f"{opt}_{lab}"] = entry
                log(f"  {k} {opt} {lab}: area {area:.2f}% Cn {entry['per_size']['foam']['r']['cnb']:+.5f}")
        # centre fin ahead of the hinge: report the best size in the sweep only
        rows = R[k]["sweep"]["centre"]
        best = max(rows, key=lambda r: r["cnb"])
        fin = fin_geometry(g, "centre", best["f_pct"] / 100)
        entry = dict(f_pct=best["f_pct"], fin=fin, per_size={}, extrapolated=False)
        for sk in ("foam", "printed"):
            entry["per_size"][sk] = eig_case(k, fin, sk, work / k / f"centre_best_{sk}", "chosen")
        R[k]["chosen"]["centre_best"] = entry
        R[k]["bare_modes"] = {sk: eig_case(k, None, sk, work / k / f"bare_{sk}", "bare") for sk in ("foam", "printed")}
    (out / "runs.json").write_text(json.dumps(R, indent=1, default=str), encoding="utf-8")
    return R


def interp_area(rows, cn_target, cn0):
    """Total fin area (% of S) at which Cn_beta reaches cn_target: linear interpolation in the sweep (Cn rises monotonically with area)."""
    pts = [(0.0, cn0)] + [(r["f_pct"], r["cnb"]) for r in rows]
    for (f0, c0), (f1, c1) in zip(pts, pts[1:]):
        if c0 <= cn_target <= c1:
            return f0 + (f1 - f0) * (cn_target - c0) / (c1 - c0)
    (f0, c0), (f1, c1) = pts[-2], pts[-1]
    return f1 + (cn_target - c1) * (f1 - f0) / (c1 - c0)          # extrapolation (flagged by the caller)


# =============================================================================================== templates
def fin_outline_mm(c, h, tab):
    """Template polygon (mm), fin upright: x LE -> TE left to right, y up.  Base at y = 0 (attachment line), glue / tape tab below it."""
    return [(0, 0), (0, h), (c, h), (c, 0), (c, -tab), (0, -tab)]


def template_page(items, title, sub):
    W, H = ft.LETTER_W, ft.LETTER_H
    mar = ft.TEMPLATE_MARGIN
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}mm" height="{H}mm" viewBox="0 0 {W} {H}">', f'<rect x="0" y="0" width="{W}" height="{H}" fill="#ffffff"/>']

    def t(x, y, s, size=3.2, w="normal"):
        o.append(f'<text x="{x:.2f}" y="{y:.2f}" font-family="Helvetica" font-size="{size}" fill="#000000" font-weight="{w}">{html.escape(s)}</text>')
    t(mar, 15, title, 5.6, "bold")
    for i, line in enumerate(sub):
        t(mar, 21 + 4.2 * i, line, 3.0)
    ys = 21 + 4.2 * len(sub) + 7
    o.append(f'<line id="scalebar" x1="{mar:.4f}" y1="{ys:.4f}" x2="{mar + 100:.4f}" y2="{ys:.4f}" stroke="#000000" stroke-width="0.5"/>')
    for i in range(11):
        o.append(f'<line x1="{mar + 10 * i:.4f}" y1="{ys - 2:.4f}" x2="{mar + 10 * i:.4f}" y2="{ys + 2:.4f}" stroke="#000000" stroke-width="0.35"/>')
    t(mar + 103, ys + 1.2, "100 mm: check after printing at 100 %", 3.0)
    x, y, rowh = mar, ys + 12, 0.0
    for it in items:
        c, h, tab = it["c"], it["h"], it["tab"]
        bw = max(c + 12, 60.0)
        bh = h + tab + 18
        if x + bw > W - mar:
            x, y, rowh = mar, y + rowh + 3, 0.0
        if y + bh > H - mar:
            raise RuntimeError("fin templates do not fit the page")
        ox, oy = x + 8, y + 12 + h                          # attachment line at y = oy
        pts = " ".join(f"{ox + px:.3f},{oy - py:.3f}" for px, py in fin_outline_mm(c, h, tab))
        o.append(f'<polygon points="{pts}" fill="#ffffff" stroke="#000000" stroke-width="0.35"/>')
        o.append(f'<line x1="{ox:.3f}" y1="{oy:.3f}" x2="{ox + c:.3f}" y2="{oy:.3f}" stroke="#c2621a" stroke-width="0.4" stroke-dasharray="1.5 1"/>')
        for k_ in range(1, int(tab // 1.5) + 1):
            yy = oy + 1.5 * k_
            if yy < oy + tab - 0.2:
                o.append(f'<line x1="{ox:.3f}" y1="{yy:.3f}" x2="{ox + c:.3f}" y2="{yy:.3f}" stroke="#000000" stroke-width="0.12"/>')
        t(x + 4, y + 3.6, it["label"], 3.0, "bold")
        t(x + 4, y + 6.8, it["sub"], 2.4)
        t(x + 4, y + 9.6, it["sub2"], 2.4)
        t(ox + c + 1.0, oy - h / 2, "TE", 2.4)
        t(ox - 5.5, oy - h / 2, "LE", 2.4)
        x += bw + 3
        rowh = max(rowh, bh)
    o.append("</svg>")
    return "\n".join(o)


TPL_LABELS = (("floor", "S1"), ("x15", "S2"), ("build", "S3"))


def build_templates(R, out: Path):
    paths = []
    for pi, (sk, order) in enumerate((("foam", "foam 400 mm build"), ("printed", "printed 250 mm build"))):
        szm = SIZES[sk]["span_mm"]
        tab = TAB_MM * szm / SPAN0
        mat = "foamboard 5 mm" if sk == "foam" else f"PLA plate {SIZES[sk]['thickness_mm']:g} mm"
        items = []
        for k in "AB":
            for opt in ("tip", "tail"):
                for lab, ln in TPL_LABELS:
                    fin = R[k]["chosen"][f"{opt}_{lab}"]["fin"]
                    c, h = fin["c"] * szm, fin["h"] * szm
                    items.append(dict(c=c, h=h, tab=tab, label=f"swift {k} {'tip fin, cut 2' if opt == 'tip' else 'tail fin, cut 1'} {ln}", sub=f"{c:.1f} x {h:.1f} mm, {c * h:.0f} mm2 each", sub2=mat))
        sub = [f"{order}. 1:1 cut templates, white paper, print at 100 %.",
               "S1 reaches Cn_beta 0.004 in AVL, S2 reaches 0.006, S3 = 2% of the wing area (sizes and numbers: index.html, MODEL).",
               "Tip fins: vertical on the wing top at 95% of the wing, trailing edge level with the wing's. Tail fin: on the tail centreline,",
               "1 mm in from the tail end. Dashed orange line = attachment line (fin base on the sheet); the hatched strip below it is the",
               "glue / tape tab, which lies flat on the sheet."]
        pp = out / f"fin_template_page{pi + 1}.svg"
        pp.write_text(template_page(items, f"Fin kit v4 templates: {order}", sub), encoding="utf-8")
        paths.append(pp)
    pdf = out / "fin_templates.pdf"
    subprocess.run([sys.executable, "-B", str(ROOT / "rigid_glider/flat_templates_pdf.py"), str(pdf)] + [str(p) for p in paths], check=True, stdout=subprocess.DEVNULL)
    return pdf, paths


# =============================================================================================== page and csv
def mode_text(m):
    """Spiral and Dutch roll as short text: 'spiral: stable' or 'doubles in 63 s'."""
    s = m["spiral"]
    sp = "stable" if s <= 0 else f"t2 {math.log(2) / s:.0f} s"
    return sp


def write_outputs(R, out: Path):
    with (out / "sweep.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["outline", "option", "total fin area % of wing area", "fin chord mm (250 span)", "fin height mm (250 span)", "fin h/c", "chord capped", "Cn_beta per rad", "Cl_beta per rad", "NP shift % MAC"])
        for k in "AB":
            for opt in OPTIONS:
                for r in R[k]["sweep"][opt]:
                    w.writerow([k, opt, r["f_pct"], f"{r['fin']['c'] * SPAN0:.2f}", f"{r['fin']['h'] * SPAN0:.2f}", f"{r['fin']['ar']:.2f}", int(r["fin"]["capped"]), f"{r['cnb']:+.5f}", f"{r['clb']:+.5f}", f"{r['d_np_mac_pct']:+.3f}"])
    with (out / "chosen_sizes.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["outline", "option_size", "build", "total fin area % of wing area", "fin chord mm", "fin height mm", "Cn_beta", "Cl_beta", "NP shift % MAC", "spiral eigenvalue 1/s", "Dutch roll damping ratio", "fin mass g",
                    "new trim CG mm", "CG shift with NP mm", "CG shift from fin mass alone mm", "extra nose ballast g", "bare all-up mass g"])
        for k in "AB":
            for key, e in R[k]["chosen"].items():
                for sk in ("foam", "printed"):
                    ps = e["per_size"][sk]
                    r, cgc, fin = ps["r"], ps["cgc"], e["fin"]
                    szm = SIZES[sk]["span_mm"]
                    w.writerow([k, key, sk, f"{e['f_pct']:.3f}", f"{fin['c'] * szm:.2f}", f"{fin['h'] * szm:.2f}", f"{r['cnb']:+.5f}", f"{r['clb']:+.5f}", f"{100 * (r['xnp'] / (szm / 1000) - R[k]['base']['np_span']) / R[k]['base']['mac']:+.3f}",
                                f"{r['modes']['spiral']:+.5f}", f"{r['modes']['dr_zeta']:+.3f}", f"{cgc['fin_g']:.3f}", f"{cgc['cg_new_target']:.2f}", f"{cgc['dcg_np_mm']:+.2f}", f"{cgc['cg_mass_only_mm']:+.2f}", f"{cgc['extra_ballast_g']:+.3f}", f"{cgc['M_old']:.2f}"])


PAGE_CSS = """:root{color-scheme:dark;--bg:#14181d;--panel:#1c2229;--line:#333c46;--ink:#dde3ea;--muted:#9aa7b4;--link:#7fb4ff;--accent:#e89a55;--thead:#262e37}
body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.5 "Segoe UI",system-ui,sans-serif}main{max-width:1240px;margin:0 auto;padding:26px 20px 60px;display:flex;flex-direction:column;gap:16px}
h1{font-size:26px;margin:0}h2{font-size:19px;margin:0;border-bottom:1px solid var(--line);padding-bottom:5px}p,ul{margin:0;max-width:110ch}ul{padding-left:22px}
table{border-collapse:collapse;background:var(--panel);font-size:13px}th,td{border:1px solid var(--line);padding:3px 7px;text-align:left;vertical-align:top}th{background:var(--thead)}td.n{text-align:right;white-space:nowrap}
.note{border-left:3px solid var(--accent);padding:2px 0 2px 12px;color:var(--muted);font-size:14px}.box{background:var(--panel);border:2px solid var(--accent);padding:10px 16px}.scroll{overflow-x:auto}code{font-size:13px}
.good{background:#1d3b2a}.bad{background:#3b1d1d}.grp td{background:#222a33;font-weight:600}.sw{display:flex;gap:18px;flex-wrap:wrap;align-items:flex-start}"""


def build_page(R, out: Path, pdf_name):
    d = R["delta"]
    P = [f"<!doctype html><html lang='en'><head><meta charset='utf-8'><title>Fin kit v4</title><style>{PAGE_CSS}</style></head><body><main>"]
    P.append("<h1>Fallback fin kit for swift A and B</h1>")
    P.append(f"<p class='note'>Tags: <b>MODEL</b> = inviscid AVL 3.52 (outline only, body ignored; {R['meta']['n_avl']} serial runs); <b>ASSUMED</b> = v4 mass assumptions (foamboard {ft.FOAM_AREAL_MASS_G_M2:g} g/m&sup2;, PLA {ft.PLA_DENSITY_G_CM3} g/cm&sup3;, inertias from the sheet outline); "
             f"<b>JUDGEMENT</b> = choices made here. Nothing was flown. <b>Fin effectiveness is uncertain:</b> AVL is inviscid, and fins this small work at chord Reynolds numbers of a few thousand, where real fins usually give less than AVL says. Templates: <a href='{pdf_name}'>{pdf_name}</a> (1:1 Letter, white).</p>")
    P.append(f"<h2>Options and sizes</h2><p>Option (a) tip fins: a pair, vertical on the wing top at {TIP_ETA:.0%} of the wing, trailing edge level with the wing's. Option (b) centre fin on the body strip, trailing edge {HINGE_CLEAR_MM:g} mm ahead of the tail hinge line. "
             f"<b>Option (b) fails in the model</b> (see the sweep): ahead of the hinge the fin's aerodynamic centre sits close to or ahead of the CG, so Cn_beta peaks and then falls as the fin grows. Added as option (c): the same centre fin glued to the tail centreline, aft of the hinge (it moves with the tail bend). "
             f"Fin height / chord = {FIN_AR} (JUDGEMENT). S1 adds Cn_beta {d['floor']:+.4f} (A reaches 0.004, MODEL), S2 adds {d['x15']:+.4f} (A at 0.006); B gets the same increments. S3 is a build size of {BUILD_AREA_PCT:g}% of the wing area (JUDGEMENT), well above S1 and S2 to leave room for the low-Reynolds-number knock-down.</p>")
    head = ["outline", "option, size", "total area, % wing", "fin c x h, mm: foam 400 / printed 250", "Cn_beta", "Cl_beta", "NP shift, % MAC", "spiral (doubling time, or stable): foam / printed", "Dutch roll damping: foam / printed",
            "fin mass, g: foam / printed", "new trim CG, mm (shift): foam / printed", "extra nose ballast, g: foam / printed"]
    rows = []
    for k in "AB":
        b = R[k]["base"]
        bm = R[k]["bare_modes"]
        cg_f, cg_p = R[k]["chosen"]["tip_floor"]["per_size"]["foam"]["cgc"]["cg_target"], R[k]["chosen"]["tip_floor"]["per_size"]["printed"]["cgc"]["cg_target"]
        rows.append(f"<tr class='grp'><td>{k}</td><td>no fin</td><td class='n'>0</td><td>-</td><td class='n'>{b['cnb']:+.4f}</td><td class='n'>{b['clb']:+.4f}</td><td class='n'>0</td><td>{mode_text(bm['foam']['r']['modes'])} / {mode_text(bm['printed']['r']['modes'])}</td>"
                    f"<td class='n'>{bm['foam']['r']['modes']['dr_zeta']:+.2f} / {bm['printed']['r']['modes']['dr_zeta']:+.2f}</td><td>-</td><td class='n'>{cg_f:.1f} / {cg_p:.1f}</td><td>-</td></tr>")
        for opt, optn in (("tip", "(a) tip fins"), ("tail", "(c) tail fin"), ("centre", "(b) centre fin")):
            labs = TPL_LABELS if opt != "centre" else (("best", "best in sweep"),)
            for lab, ln in labs:
                e = R[k]["chosen"][f"{opt}_{lab}" if opt != "centre" else "centre_best"]
                fin = e["fin"]
                pf, pp = e["per_size"]["foam"], e["per_size"]["printed"]
                r = pf["r"]
                dnp = 100 * (r["xnp"] / 0.4 - b["np_span"]) / b["mac"]
                cls = "good" if r["cnb"] >= CN_FLOOR - 1e-4 else "bad"
                flag = "*" if fin["capped"] else ""
                ex = " (extrapolated)" if e["extrapolated"] else ""
                rows.append(f"<tr><td>{k}</td><td>{optn}, {ln}{ex}</td><td class='n'>{e['f_pct']:.2f}</td><td class='n'>{fin['c'] * 400:.1f} x {fin['h'] * 400:.1f} / {fin['c'] * 250:.1f} x {fin['h'] * 250:.1f}{flag}</td><td class='n {cls}'>{r['cnb']:+.4f}</td>"
                            f"<td class='n'>{r['clb']:+.4f}</td><td class='n'>{dnp:+.2f}</td><td>{mode_text(r['modes'])} / {mode_text(pp['r']['modes'])}</td><td class='n'>{r['modes']['dr_zeta']:+.2f} / {pp['r']['modes']['dr_zeta']:+.2f}</td>"
                            f"<td class='n'>{pf['cgc']['fin_g']:.2f} / {pp['cgc']['fin_g']:.2f}</td><td class='n'>{pf['cgc']['cg_new_target']:.1f} ({pf['cgc']['dcg_np_mm']:+.2f}) / {pp['cgc']['cg_new_target']:.1f} ({pp['cgc']['dcg_np_mm']:+.2f})</td>"
                            f"<td class='n'>{pf['cgc']['extra_ballast_g']:+.2f} / {pp['cgc']['extra_ballast_g']:+.2f}</td></tr>")
    P.append("<div class='scroll'><table><tr>" + "".join(f"<th>{c}</th>" for c in head) + "</tr>" + "".join(rows) + "</table></div>")
    P.append(f"<p class='note'>Cn_beta, Cl_beta and the NP shift are scale-free; they are about the trim CG at a 10% margin, at CL 0.3, with the tail at the nominal gauge bend (A {BENDS['A']:g} deg, B {BENDS['B']:g} deg), so the no-fin values (A {R['A']['base']['cnb']:+.4f}, B {R['B']['base']['cnb']:+.4f}) differ from the v4 page "
             f"(A +0.0030, B +0.0041 at 25% MAC reference and a 5 deg bend). Spiral and Dutch roll are the lateral eigenvalues of AVL's linearised system matrix at the same condition (speed from weight at CL 0.3, mass and inertias from the v4 build, ASSUMED); t2 = time to double. "
             "'New trim CG' = the 10% margin CG at the new neutral point (the shift is the NP shift); the fin mass moves the CG aft as well, so the last column is the nose ballast to add (+) or remove (-) at the v4 nose station to land on the new CG. Fin mass includes a 4 mm glue tab (scaled with span). * chord limited by the local wing chord, so the fin is taller than h/c = 1.25.</p>")
    # spiral / side-effect summary lines
    P.append("<h2>Side effects (MODEL)</h2>" + side_effects(R))
    # sweeps
    P.append("<h2>Fin area sweep</h2><p>Cn_beta rises monotonically with area for tip fins and for the tail fin; the centre fin ahead of the hinge peaks at 1 to 2% of the wing area and falls.</p><div class='sw'>")
    for opt, optn in (("tip", "(a) tip fins (pair)"), ("tail", "(c) tail fin"), ("centre", "(b) centre fin ahead of the hinge")):
        t = f"<div><b>{optn}</b><table><tr><th>area, % wing</th><th>c x h, mm (250)</th><th>A Cn_beta</th><th>A Cl_beta</th><th>A dNP, % MAC</th><th>B Cn_beta</th><th>B Cl_beta</th><th>B dNP, % MAC</th></tr>"
        for ra, rb in zip(R["A"]["sweep"][opt], R["B"]["sweep"][opt]):
            t += (f"<tr><td class='n'>{ra['f_pct']:g}</td><td class='n'>{ra['fin']['c'] * 250:.1f} x {ra['fin']['h'] * 250:.1f}{'*' if ra['fin']['capped'] else ''}</td><td class='n'>{ra['cnb']:+.4f}</td><td class='n'>{ra['clb']:+.4f}</td><td class='n'>{ra['d_np_mac_pct']:+.2f}</td>"
                  f"<td class='n'>{rb['cnb']:+.4f}</td><td class='n'>{rb['clb']:+.4f}</td><td class='n'>{rb['d_np_mac_pct']:+.2f}</td></tr>")
        P.append(t + "</table></div>")
    P.append("</div>")
    P.append(f"<div class='box'><b>Which to try first:</b> {recommendation(R)}</div>")
    P.append("<h2>Caveats</h2><ul><li>AVL treats each fin as a flat lifting surface in inviscid flow; at chord Reynolds numbers of 1,000 to 8,000 real fins lose lift slope and can stall early, so the model Cn_beta gain is an upper estimate. S1 and S2 are model minimums, far smaller than a fin that is easy to cut and glue.</li>"
             "<li>Cl_beta is barely changed (it stays about as stable as without fins). The spiral mode is slightly less stable with fins; the table gives the numbers. Eigenvalues come from AVL's system matrix with an eigen-solver written here (AVL's own eigen routine crashes in this build); inertias are plate estimates, so the Dutch roll damping in particular is an estimate.</li>"
             "<li>Fins add mass aft of the CG; foam fin mass assumes the 5 mm foamboard areal mass of the v4 files even for fins this small. The printed numbers use PLA at the v4 plate thickness.</li></ul>")
    P.append("</main></body></html>")
    (out / "index.html").write_text("\n".join(P), encoding="utf-8")


def side_effects(R):
    L = []
    for k in "AB":
        b0 = R[k]["bare_modes"]["foam"]["r"]["modes"]["spiral"]
        out = []
        for opt, key in (("tip S2", "tip_x15"), ("tip S3", "tip_build"), ("tail S2", "tail_x15"), ("tail S3", "tail_build")):
            m = R[k]["chosen"][key]["per_size"]["foam"]["r"]["modes"]
            out.append(f"{opt} {mode_text({'spiral': m['spiral']})}")
        L.append(f"<li>{k}, foam 400: no fin spiral eigenvalue {b0:+.4f} 1/s (neutral); with fins: " + "; ".join(out) + ".</li>")
    return "<ul>" + "".join(L) + "</ul><p class='note'>A spiral that doubles in tens of seconds is mild for a glide that lasts a few seconds (JUDGEMENT); whether it matters is for the first drops to show.</p>"


def recommendation(R):
    a_t, a_c, a_tail = R["A"]["chosen"]["tip_floor"], R["A"]["chosen"]["centre_best"], R["A"]["chosen"]["tail_floor"]
    s3 = R["A"]["chosen"]["tip_build"]["per_size"]["foam"]
    return (f"(a) tip fins, S3, foam 400 first. In the model they reach Cn_beta 0.004 with {a_t['f_pct']:.2f}% of the wing area (tail fin {a_tail['f_pct']:.2f}%, similar; centre fin ahead of the hinge never reaches it, best {a_c['per_size']['foam']['r']['cnb']:+.4f}) "
            f"and keep Cl_beta and the spiral mode a little better than the tail fin at equal Cn_beta. S3 gives {s3['r']['cnb']:+.4f} for A at {s3['cgc']['fin_g']:.2f} g (CG shift {s3['cgc']['dcg_np_mm']:+.2f} mm, nose ballast {s3['cgc']['extra_ballast_g']:+.2f} g); "
            "it is cut well above the AVL minimum because real fins at this Reynolds number are weaker. The tail fin (c) is the alternative if tip fins are awkward to mount.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--render", action="store_true")
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    if a.render:
        R = json.loads((OUT / "runs.json").read_text(encoding="utf-8"))
    else:
        R = run_study(OUT)
    write_outputs(R, OUT)
    pdf, _ = build_templates(R, OUT)
    build_page(R, OUT, pdf.name)
    print("wrote", OUT)


if __name__ == "__main__":
    main()
