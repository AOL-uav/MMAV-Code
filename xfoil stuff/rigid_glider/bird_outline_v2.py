"""Bird-style flat glider outlines, version 2: SMOOTH outlines whose every feature is justified by a modelled number.

Starting from the v1 swift-like (BS) and crow-like (BC) outlines (bird_outline.py), one feature at a time is switched on and the change is tabulated in AVL numbers
(neutral point, CM0 per degree of tail bend, trim CG, static margin, Cn_beta and Cl_beta with no fin, span efficiency, tip loading), in sheet area / mass / ballast /
wing loading (arithmetic), and in two simple beam-arithmetic structural numbers (tail neck, wrist section).  Features that change no modelled quantity are labelled
APPEARANCE ONLY with their modelled cost.  Candidate outlines are selected by stated criteria, never by eye; failing candidates are listed with the reason.

Honesty rules as in v1: tags GEOMETRY / AVL / ARITHMETIC / JUDGEMENT / ASSUMED / CITED; bird proportions only where research/bird_research_notes.md gives a cited
number (aspect ratio of swift, magpie/jackdaw; swift tail span).  The notes give NO planform data (arm/hand split, leading-edge angles, trailing-edge shape, body
width, root blend, tail neck) so those are JUDGEMENT and the page says so.  Nothing is traced from an image.  AVL is inviscid, outline-only, body ignored: it says
nothing about flat-plate stall, low Reynolds number, or whether a build pulls out.  No flat-plate glide-ratio or CG-window prediction.  AVL runs are serial.

Writes ONLY to --out (default results/runs/rigid_glider/bird_outline_v2/).  Imports (never edits) bird_outline.py and flat_test.py.
    py -B rigid_glider/bird_outline_v2.py [--out DIR] [--no-pdf] [--quick]
"""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import argparse
import csv
import html
import itertools
import json
import math
import subprocess
from pathlib import Path

import bird_outline as bo
import flat_test as ft
from avl import run_avl
from geometry import Wing

ROOT = ft.ROOT
OUT_DEFAULT = ROOT / "results/runs/rigid_glider/bird_outline_v2"
SOURCES = bo.SOURCES
FAM_ORDER = bo.FAM_ORDER

# =============================================================================================== PARAMETERS
FEATURES = ("m_shape", "curved_te", "root_blend", "tip_round", "neck", "spread", "nose")
AERO_FEATURES = ("m_shape", "curved_te", "tip_round", "neck", "spread")          # change the AVL planform; root_blend and nose change only the sheet (body ignored in AVL)
FEATURE_LABEL = {
    "m_shape": "M-shaped leading edge: arm swept FORWARD to the wrist, hand swept back, rounded wrist",
    "curved_te": "curved trailing edge: bulged arm trailing edge and smooth (cubic) hand chord distribution",
    "root_blend": "rounded blend between wing root and body (no hard corner)",
    "tip_round": "hand taper with a rounded (circular-closure) tip",
    "neck": "wider tail neck at the hinge (structural)",
    "spread": "wider fan-tail spread",
    "nose": "longer body / nose ahead of the wing",
}
# on-values per family (JUDGEMENT).  Off-values are the v1 parameters.  strength s in [0, 1] interpolates linearly.
FEATURE_ON = {
    "m_shape": {"BS": dict(arm_sweep_deg=-10.0, wrist_blend_frac=0.10), "BC": dict(arm_sweep_deg=-10.0, wrist_blend_frac=0.10)},
    "curved_te": {"BS": dict(arm_te_bulge=0.10, te_smooth=1.0), "BC": dict(arm_te_bulge=0.10, te_smooth=1.0)},
    "root_blend": {"BS": dict(root_blend=1.0), "BC": dict(root_blend=1.0)},
    "tip_round": {"BS": dict(tip_round=1.0), "BC": dict(tip_round=1.0)},
    "neck": {"BS": dict(tail_root_frac=0.08), "BC": dict(tail_root_frac=0.12)},
    "spread": {"BS": dict(tail_spread_frac=0.30), "BC": dict(tail_spread_frac=0.48)},
    "nose": {"BS": dict(nose_frac=0.09), "BC": dict(nose_frac=0.11)},
}
NEW_PARAM_OFF = dict(wrist_blend_frac=0.0, arm_te_bulge=0.0, te_smooth=0.0, tip_round=0.0, root_blend=0.0)
N_ARM_SEG = 8                        # straight segments along the arm leading / trailing edge
TIP_ETA0 = 0.86                      # tip rounding starts at this fraction of the hand semi-span
DY_MIN = 0.006                       # minimum node spacing in units of span (AVL: at least one spanwise vortex per section interval)
TIP_CHORD_FLOOR = 0.03               # chord at the very tip / wrist chord when rounded (AVL needs a positive chord)
REF_BEND = 5.0                       # tail bend (deg, TE up) used for the feature table and the screening (JUDGEMENT)
BENDS = (0.0, 2.0, 3.0, 5.0, 7.5, 10.0)   # the brief's list
SELECTION_BENDS = (2.0, 3.0, 5.0, 7.5, 10.0)
TRIM_CLS = bo.TRIM_CLS
AVL_ALPHAS = bo.AVL_ALPHAS
# ---- selection criteria (parameters; stated on the page)
SM_WINDOW = (0.08, 0.15)             # static margin at CL 0.3 with the chosen bend
CN_BETA_MIN = 0.004                  # per rad, no fin (JUDGEMENT); else the smallest ventral fin that reaches it
CL_BETA_MAX = -0.005                 # per rad: Cl_beta must be at least this negative (stable sign)
BALLAST_MAX_FRAC = 0.20              # nose ballast / all-up mass, at both sizes
NECK_MIN_FRAC_SPAN = 0.04            # absolute floor on the neck width (JUDGEMENT) in addition to the stress minimum
FIN_AREAS = (0.03, 0.05)             # ventral fin areas tried (fraction of wing area)
EDGE_FIN_ANGLES = (30.0, 60.0)       # tail side edges bent up as small fins (dihedral panels), deg
EDGE_FIN_FRAC = 0.30                 # outer fraction of the fan half-width that is bent up
# ---- structure (ARITHMETIC; assumptions stated on the page)
N_LOAD = 5.0                         # JUDGEMENT: design load factor (pull-out / handling)
SIGMA_ALLOW_MPA = 20.0               # ASSUMED allowable bending stress of printed PLA across layers
SAFETY_FACTOR = 2.0
THIN_SHEET_MAC_PCT = 6.0             # thinner-sheet note: thickness that keeps the sheet under this % of MAC
HINGE_V2_MM = 0.4                    # printed hinge strip thickness (two layers; v1 used 0.2 mm).  JUDGEMENT: at 0.2 mm the beam arithmetic rejects the crow-like tail neck
PLA_STRAIN_ALLOW_PCT = 3.0           # ASSUMED outer-fibre strain a PLA hinge strip survives; reported for each bend, not a selection criterion
RAIL_HINGE_GAP_MM = 3.0              # the new rail saddle ends this far ahead of the hinge
RAIL_TICK_MARGIN_MM = 5.0
SIZES = bo.SIZES
PULLOUT_CLS = bo.PULLOUT_CLS
G_ACCEL = bo.G_ACCEL
AVL_NSPAN = 80
AVL_NCHORD = 10
AVL_SPACE = 0.0


def T(value, source, unit="", note=""):
    return bo.T(value, source, unit, note)


# =============================================================================================== parameters and features
def base_params(fam: str) -> dict:
    p = {k: v[0] for k, v in bo.FAMILIES[fam]["params"].items()}
    p.update(NEW_PARAM_OFF)
    return p


def feature_params(fam: str, on: dict) -> dict:
    """on: feature name -> strength in [0, 1].  Each feature changes exactly its own parameters."""
    p = base_params(fam)
    for name, s in on.items():
        if not s:
            continue
        for k, v in FEATURE_ON[name][fam].items():
            p[k] = p[k] + float(s) * (v - p[k])
    return p


def feature_keys(name: str, fam: str) -> set:
    return set(FEATURE_ON[name][fam])


# =============================================================================================== smooth outline
def _pchip(xs, ys):
    """Monotone cubic (Fritsch-Carlson) interpolant as a function."""
    n = len(xs)
    h = [xs[i + 1] - xs[i] for i in range(n - 1)]
    d = [(ys[i + 1] - ys[i]) / h[i] for i in range(n - 1)]
    m = [0.0] * n
    m[0], m[-1] = d[0], d[-1]
    for i in range(1, n - 1):
        m[i] = 0.0 if d[i - 1] * d[i] <= 0 else 2 * d[i - 1] * d[i] / (d[i - 1] + d[i])

    def f(x):
        i = min(max(max(k for k in range(n) if xs[k] <= x + 1e-15), 0), n - 2)
        t = (x - xs[i]) / h[i]
        h00, h10, h01, h11 = 2 * t ** 3 - 3 * t ** 2 + 1, t ** 3 - 2 * t ** 2 + t, -2 * t ** 3 + 3 * t ** 2, t ** 3 - t ** 2
        return h00 * ys[i] + h10 * h[i] * m[i] + h01 * ys[i + 1] + h11 * h[i] * m[i + 1]
    return f


def _bezier(p0, p1, p2, p3, n):
    pts = []
    for i in range(n + 1):
        t = i / n
        a, b, c, d = (1 - t) ** 3, 3 * t * (1 - t) ** 2, 3 * t * t * (1 - t), t ** 3
        pts.append((a * p0[0] + b * p1[0] + c * p2[0] + d * p3[0], a * p0[1] + b * p1[1] + c * p2[1] + d * p3[1]))
    return pts


def simplify(poly, tol=1e-9):
    """Remove (nearly) collinear vertices (needed before ear-clipping and for clean DXF); areas are unchanged."""
    pts = list(poly)
    changed = True
    while changed and len(pts) > 3:
        changed = False
        for i in range(len(pts)):
            a, b, c = pts[i - 1], pts[i], pts[(i + 1) % len(pts)]
            cr = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
            ln = math.hypot(c[0] - a[0], c[1] - a[1])
            if abs(cr) <= tol * max(ln, 1e-12) and (b[0] - a[0]) * (c[0] - b[0]) + (b[1] - a[1]) * (c[1] - b[1]) > 0:
                del pts[i]
                changed = True
                break
    return pts


def build_geometry2(p: dict, bend_deg=0.0, b=1.0):
    """Smooth outline.  Leading edge: arm line (angle arm_sweep_deg, negative = forward) to the wrist, rounded wrist (quadratic blend, half-length wrist_blend_frac x semi-span),
    then the hand line (hand_sweep_deg).  Trailing edge: root flat to the tail neck, arm trailing edge (straight + bulge arm_te_bulge x root chord), hand chord distribution
    through the station table (linear, or monotone cubic with te_smooth, with a circular-closure tip with tip_round).  Same dictionary keys as bird_outline.build_geometry."""
    wb, xn = p["body_halfwidth_frac"] * b, -p["nose_frac"] * b
    half = b / 2
    yw = p["wrist_frac"] * half
    tan_a, tan_h = math.tan(math.radians(p["arm_sweep_deg"])), math.tan(math.radians(p["hand_sweep_deg"]))
    Lt, St, wt0 = p["tail_len_frac"] * b, p["tail_spread_frac"] * b, p["tail_root_frac"] * b
    st = [tuple(s) for s in p["hand_stations"]]
    sm, tr = float(p["te_smooth"]), float(p["tip_round"])
    d_bl = float(p["wrist_blend_frac"]) * half
    xw = yw * tan_a                                                       # wrist corner (unblended)

    def le_x(y):
        if y >= yw + d_bl or y <= yw - d_bl or d_bl <= 0:
            return y * tan_a if y <= yw else xw + (y - yw) * tan_h
        t = (y - (yw - d_bl)) / (2 * d_bl)
        x0, x2 = (yw - d_bl) * tan_a, xw + d_bl * tan_h
        return (1 - t) ** 2 * x0 + 2 * t * (1 - t) * xw + t * t * x2

    etas = [e for e, f in st]
    lin = lambda e: next(st[i][1] + (e - st[i][0]) / (st[i + 1][0] - st[i][0]) * (st[i + 1][1] - st[i][1]) for i in range(len(st) - 1) if e <= st[i + 1][0] + 1e-12)
    cub = _pchip(etas, [f for e, f in st])

    def frac(e):
        f = (1 - sm) * lin(e) + sm * cub(e)
        if tr > 0 and e > TIP_ETA0:
            u = (e - TIP_ETA0) / (1 - TIP_ETA0)
            f0 = (1 - sm) * lin(TIP_ETA0) + sm * cub(TIP_ETA0)
            g_ = max(f0 * math.sqrt(max(1 - u * u, 0.0)), TIP_CHORD_FLOOR)
            f = (1 - tr) * f + tr * g_
        return f

    sub = 2 if (sm == 0 and tr == 0) else 4
    eta_nodes = set()
    for i in range(len(etas) - 1):
        for k in range(sub):
            eta_nodes.add(etas[i] + (etas[i + 1] - etas[i]) * k / sub)
    eta_nodes.add(etas[-1])
    if tr > 0:
        for u in (0.2, 0.4, 0.6, 0.8, 0.9, 0.96, 0.99, 1.0):
            eta_nodes.add(TIP_ETA0 + u * (1 - TIP_ETA0))
    eta_nodes = sorted(eta_nodes)

    def geom_for(c_root):
        xh, cw = c_root, p["cw_over_croot"] * c_root
        y0 = wt0 / 2
        ys = {0.0, y0, yw}
        for k in range(1, N_ARM_SEG):
            ys.add(y0 + (yw - y0) * k / N_ARM_SEG)
        if d_bl > 0:
            for k in range(-3, 4):
                ys.add(yw + d_bl * k / 3)
        for e in eta_nodes:
            ys.add(yw + e * (half - yw))
        ys = sorted(y for y in ys if y >= 0)
        must = {0.0, y0, yw, half}                                      # keep the structural nodes; drop nodes closer than DY_MIN to the last kept one (AVL needs >= 1 vortex per interval)
        kept = [ys[0]]
        for y in ys[1:]:
            if y in must or y - kept[-1] >= DY_MIN:
                if kept[-1] not in must and y - kept[-1] < DY_MIN:
                    kept.pop()
                kept.append(y)
        ys = kept
        les, tes = [], []
        for y in ys:
            les.append(0.0 if y == 0 else le_x(y))
            if y <= y0 + 1e-15:
                te = xh
            elif y <= yw + 1e-15:
                tau = (y - y0) / (yw - y0)
                te = xh + tau * (xw + cw - xh) + p["arm_te_bulge"] * c_root * 4 * tau * (1 - tau)
            else:
                e = (y - yw) / (half - yw)
                te = xw + (y - yw) * tan_h + frac(e) * cw
            tes.append(te)
        return xh, cw, ys, les, tes

    def area(c_root):
        xh, cw, ys, les, tes = geom_for(c_root)
        return 2 * bo._chord_integrals(ys, les, tes)[0]

    target = b * b / p["ar_target"]
    lo, hi = 0.01 * b, 2.0 * b
    for _ in range(80):
        mid = (lo + hi) / 2
        if area(mid) < target:
            lo = mid
        else:
            hi = mid
    c_root = (lo + hi) / 2
    xh, cw, ys, les, tes = geom_for(c_root)
    a_half, c2, xcint, ycint = bo._chord_integrals(ys, les, tes)
    S, mac, x_le_mac = 2 * a_half, c2 / a_half, xcint / a_half
    # ---- right-half outline: nose -> (blend) -> leading edge -> tip -> trailing edge -> neck junction -> fan tail -> centreline
    s_bl = float(p["root_blend"])
    if s_bl > 0:
        yb = wb * (1 + 3 * s_bl)
        p3 = (le_x(yb), yb)
        dx = le_x(yb + 1e-4) - le_x(yb)
        nrm = math.hypot(dx, 1e-4)
        dirx, diry = dx / nrm, 1e-4 / nrm
        L03 = math.hypot(p3[0] - xn, p3[1])
        nose_chain = _bezier((xn, 0.0), (xn, 0.55 * yb), (p3[0] - 0.45 * L03 * dirx, p3[1] - 0.45 * L03 * diry), p3, 10)
    else:
        yb = wb
        nose_chain = [(xn, 0.0), (0.5 * xn, 0.75 * wb), (le_x(wb), wb)]
    le_pts = [(x, y) for y, x in zip(ys, les) if y > yb + 1e-12]
    te_pts = [(x, y) for y, x in zip(ys, tes) if y > wt0 / 2 + 1e-12]
    bulge = p["tail_bulge_frac"] * Lt
    ex = xh + Lt + 1.5 * bulge
    R = nose_chain + le_pts + list(reversed(te_pts)) + [(xh, wt0 / 2)]
    i_junction = len(R) - 1
    R += [(xh + Lt, St / 2), (xh + Lt + bulge, St / 4), (ex, 0.0)]
    full = simplify(ft.orient_ccw(R + [(x, -y) for (x, y) in reversed(R[1:-1])]))
    main_R = R[:i_junction + 1]
    main_full = simplify(ft.orient_ccw(main_R + [(x, -y) for (x, y) in reversed(main_R[1:])]))
    wing_poly_half = [(0.0, 0.0)] + [(x, y) for y, x in zip(ys, les) if y > 0] + list(reversed(te_pts)) + [(xh, wt0 / 2), (xh, 0.0)]
    side = lambda x: wt0 / 2 + (x - xh) / Lt * (St / 2 - wt0 / 2)
    tail_half = [(xh, wt0 / 2), (xh + Lt, St / 2), (xh + Lt + bulge, St / 4), (ex, 0.0), (xh, 0.0)]
    tail_area = 2 * abs(ft.area_signed(tail_half))
    g = dict(p=p, b=b, c_root=c_root, x_h=xh, c_wrist=cw, x_wrist=xw, y_wrist=yw, half_span=half, hand=[], tip_te=[], wt0=wt0, Lt=Lt, St=St, bulge=bulge, tail_end_x=ex,
             y_nodes=ys, le_nodes=les, te_nodes=tes, S=S, S_polygon=2 * abs(ft.area_signed(wing_poly_half)), mac=mac, x_le_mac=x_le_mac, x_ref=x_le_mac + 0.25 * mac, ar=b * b / S,
             half_outline=simplify(R), full_outline=full, main_full=main_full, tail_area=tail_area, sheet_area=abs(ft.area_signed(full)), nose_x=xn, wb=wb, side=side, bend_deg=bend_deg,
             chord_wrist_local=tes[ys.index(yw)] - les[ys.index(yw)], n_nodes=len(ys), tail_centroid_x=ft.polygon_centroid(tail_half)[0] if tail_area > 0 else xh)
    return g


def polygon_is_simple(poly):
    n = len(poly)

    def ccw(a, b, c):
        return (c[1] - a[1]) * (b[0] - a[0]) - (b[1] - a[1]) * (c[0] - a[0])

    def cross(a, b, c, d):
        return ccw(a, c, d) * ccw(b, c, d) < -1e-18 and ccw(a, b, c) * ccw(a, b, d) < -1e-18
    for i in range(n):
        for j in range(i + 2, n):
            if i == 0 and j == n - 1:
                continue
            if cross(poly[i], poly[(i + 1) % n], poly[j], poly[(j + 1) % n]):
                return False
    return True


# =============================================================================================== AVL (v2 deck: many sections, tail side-edge fins, strip loads)
def tail_sections2(g, edge=None):
    """Tail sections (y, x_le, z, chord).  edge = (fraction of the fan half-width, angle deg): the outer part is bent UP about a chordwise line (small fins as dihedral panels)."""
    secs = [(y, x, 0.0, c) for y, x, c in bo.tail_sections(g)]
    if not edge:
        return secs
    frac, ang = edge
    half = g["St"] / 2
    ye = half * (1 - frac)
    out = []
    ins = False
    for i, s in enumerate(secs):
        if not ins and s[0] > ye + 1e-12:
            a, c_ = secs[i - 1], s
            t = (ye - a[0]) / (c_[0] - a[0])
            out.append((ye, a[1] + t * (c_[1] - a[1]), 0.0, a[3] + t * (c_[3] - a[3])))
            ins = True
        out.append(s)
    res = []
    for y, x, z, c in out:
        if y > ye + 1e-12:
            d = y - ye
            res.append((ye + d * math.cos(math.radians(ang)), x, d * math.sin(math.radians(ang)), c))
        else:
            res.append((y, x, z, c))
    return res


def avl_deck2(g, span_mm, bend_deg, vfin=None, edge=None, name="bird2", with_tail=True):
    k = span_mm / 1000.0
    f = lambda v: f"{v * k:.9f}"
    nspan = AVL_NSPAN
    L = [name, "0.0", "0 0 0.0", f"{g['S'] * k * k:.9f} {g['mac'] * k:.9f} {k:.9f}", f"{f(g['x_ref'])} 0.0 0.0",
         "SURFACE", "Wing", f"{AVL_NCHORD} 1.0 {nspan} {AVL_SPACE}", "YDUPLICATE", "0.0"]
    for y, x, z, c in bo.wing_sections(g):
        L += ["SECTION", f"{f(x)} {f(y)} {f(z)} {f(c)} 0.0", "NACA", "0000"]
    if with_tail:
        L += ["SURFACE", "Tail", "6 1.0 12 -2.0", "YDUPLICATE", "0.0"]
        for y, x, z, c in tail_sections2(g, edge):
            L += ["SECTION", f"{f(x)} {f(y)} {f(z)} {f(c)} {-bend_deg:.6f}", "NACA", "0000"]
    if vfin:
        L += ["SURFACE", "VentralFin", "5 1.0 8 -2.0"]
        for z in (0.0, -vfin["h"]):
            L += ["SECTION", f"{f(vfin['x_le'])} 0.0 {f(z)} {f(vfin['chord'])} 0.0", "NACA", "0000"]
    return "\n".join(L) + "\n"


def parse_strips(path: Path):
    """AVL strip-force file -> {surface name: [dict(y, c, cl, a)]}.  The file prints strip AREA to 4 decimals (0.0000 for small models), so the strip area is rebuilt as
    chord x strip width, the width from the midpoints between neighbouring strip stations."""
    out, cur, inside = {}, None, False
    for line in Path(path).read_text(errors="replace").splitlines():
        s = line.strip()
        if s.startswith("Surface #"):
            cur = " ".join(s.split()[3:])
            out[cur] = []
            inside = False
        elif "Strip Forces referred to Strip Area, Chord" in line:
            inside = True
        elif inside and cur is not None:
            q = line.split()
            if len(q) >= 15 and q[0].isdigit():
                out[cur].append(dict(y=float(q[2]), c=float(q[4]), cl=float(q[9])))
    for name, rows in out.items():
        rows.sort(key=lambda r: r["y"])
        n = len(rows)
        for i, r in enumerate(rows):
            lo = (rows[i - 1]["y"] + r["y"]) / 2 if i > 0 else r["y"] - (rows[1]["y"] - r["y"]) / 2 if n > 1 else r["y"] - 0.001
            hi = (rows[i + 1]["y"] + r["y"]) / 2 if i < n - 1 else r["y"] + (r["y"] - rows[i - 1]["y"]) / 2 if n > 1 else r["y"] + 0.001
            r["a"] = r["c"] * abs(hi - lo)
    return out


def run_case2(g, span_mm, bend, workdir: Path, name, vfin=None, edge=None, with_tail=True):
    """Like bird_outline.run_case plus: AVL induced drag and span efficiency at CL 0.3, tip loading (area-weighted local cl of the outer 20% of the semi-span divided by the
    wing CL; an INDICATOR of tip-stall tendency only), the tail lift fraction of the weight and the wrist-section bending moment (both normalised by the weight)."""
    k = span_mm / 1000.0
    w = Wing(span_m=k, aspect_ratio=g["ar"], taper=0.5, quarter_chord_sweep_deg=0.0, washout_deg=0.0, dihedral_deg=0.0, section="NACA_0000", cg_x_m=g["x_ref"] * k)
    deck = avl_deck2(g, span_mm, bend, vfin, edge, name, with_tail)
    pts = run_avl(w, name, list(AVL_ALPHAS), workdir / "scan", bo.AVL_EXE, deck_text=deck)
    cl = [p.cl for p in pts]
    cm = [p.cm for p in pts]
    kk, cm0 = bo._lsq(cl, cm)
    cla_deg = bo._lsq([p.alpha_deg for p in pts], cl)[0]
    cl0 = bo._lsq([p.alpha_deg for p in pts], cl)[1]
    a03 = (0.3 - cl0) / cla_deg
    p3 = run_avl(w, name, [a03], workdir / "cl03", bo.AVL_EXE, deck_text=deck)[0]
    xnp = sum(p.xnp_m for p in pts + [p3]) / 4 / k
    mac = g["mac"]
    strips = parse_strips(p3.strip_file)
    S_m2 = g["S"] * k * k
    wing = [s for n, ss in strips.items() if n.startswith("Wing") for s in ss if s["y"] > 0]
    half_m = g["half_span"] * k
    outer = [s for s in wing if s["y"] >= 0.8 * half_m]
    tip_loading = (sum(s["cl"] * s["a"] for s in outer) / sum(s["a"] for s in outer)) / p3.cl if outer else None
    tail = [s for n, ss in strips.items() if n.startswith("Tail") for s in ss if s["y"] > 0]
    tail_frac = 2 * sum(s["cl"] * s["a"] for s in tail) / (S_m2 * p3.cl) if tail else 0.0              # tail lift / weight at CL 0.3
    yw_m = g["y_wrist"] * k
    m_wrist = sum(s["cl"] * s["a"] * (s["y"] - yw_m) for s in wing if s["y"] > yw_m) / (S_m2 * p3.cl) if wing else 0.0     # (M / W) in metres, one wing, about the wrist
    cdi = p3.cdi
    e = p3.cl ** 2 / (math.pi * g["ar"] * cdi) if cdi > 0 else None
    res = dict(np_span=xnp, np_pct_mac=100 * (xnp - g["x_le_mac"]) / mac, cm0=cm0, dcm_dcl=kk, cla_per_rad=cla_deg * 180 / math.pi, alpha_zero_lift=-cl0 / cla_deg, alpha_cl03=a03,
               cn_beta=p3.cnb_per_rad, cl_beta=p3.clb_per_rad, cdi_cl03=cdi, span_eff=e, tip_loading=tip_loading, tail_lift_frac=tail_frac, m_wrist_over_w_m=m_wrist,
               np_from_cm_slope_span=g["x_ref"] - kk * mac)
    res["trim"] = {c: dict(x_cg_span=xnp - mac * cm0 / c, static_margin=cm0 / c) for c in TRIM_CLS}
    return res


def scale_bend(res, bend_from, bend_to, mac):
    """Linear (AVL is linear in the tail incidence to a few per cent): CM0 and the trim CG at another bend, from one run.  Used for SCREENING only."""
    f = bend_to / bend_from if bend_from else 0.0
    r = dict(res)
    r["cm0"] = res["cm0"] * f
    r["trim"] = {c: dict(x_cg_span=res["np_span"] - mac * r["cm0"] / c, static_margin=r["cm0"] / c) for c in TRIM_CLS}
    r["tail_lift_frac"] = res["tail_lift_frac"] * f
    return r


# =============================================================================================== runner with a disk cache
RUN_SPAN_MM = 250.0
CACHE_VERSION = "v2.1"


class Runner:
    """Serial AVL with an in-memory and on-disk cache.  Only the AERO features and the bend / fin / edge options enter the key (the body is ignored in the model)."""

    def __init__(self, workdir: Path, avl=True):
        self.workdir = Path(workdir)
        self.avl = avl
        self.cache = {}
        self.cache_path = self.workdir / "cache.json"
        self.n_runs = 0
        if self.cache_path.is_file():
            try:
                d = json.loads(self.cache_path.read_text())
                if d.get("version") == CACHE_VERSION:
                    self.cache = {k: self._revive(v) for k, v in d["items"].items()}
            except Exception:
                self.cache = {}

    @staticmethod
    def _revive(r):
        r = dict(r)
        r["trim"] = {float(c): v for c, v in r["trim"].items()}
        return r

    def save(self):
        self.workdir.mkdir(parents=True, exist_ok=True)
        self.cache_path.write_text(json.dumps(dict(version=CACHE_VERSION, items=self.cache)))

    def key(self, fam, on, bend, vfin, edge):
        aero = sorted((n, round(float(on.get(n, 0.0)), 6)) for n in AERO_FEATURES if on.get(n, 0.0))
        return json.dumps([fam, aero, bend, vfin, edge])

    def run(self, fam, on, bend, vfin=None, edge=None):
        k = self.key(fam, on, bend, vfin, edge)
        if k in self.cache:
            return self.cache[k]
        if not self.avl:
            raise RuntimeError("AVL disabled and result not cached: " + k)
        p = feature_params(fam, {n: s for n, s in on.items() if n in AERO_FEATURES})
        g = build_geometry2(p)
        vf = bo.vfin_for(g, vfin) if vfin else None
        tag_ = "c%05d" % (len(self.cache) + 1)
        r = run_case2(g, RUN_SPAN_MM, bend, self.workdir / tag_, tag_, vf, edge)
        self.n_runs += 1
        r = self._revive(json.loads(json.dumps(r)))
        self.cache[k] = r
        if self.n_runs % 8 == 0:
            self.save()
        return r


# =============================================================================================== rails, masses, ballast, structure (ARITHMETIC)
_RAIL_CACHE = {}


def make_rail(saddle_end_mm, cg_lo_mm, cg_hi_mm):
    """New CG rail: saddle ends at saddle_end_mm (ahead of the hinge), ticks cover [cg_lo, cg_hi] +- 5 mm.  Same hole / notch conventions as flat_test.stick_mesh."""
    key = (round(saddle_end_mm, 3), int(math.floor(cg_lo_mm)), int(math.ceil(cg_hi_mm)))
    if key in _RAIL_CACHE:
        return _RAIL_CACHE[key]
    tris, info = ft.stick_mesh({"cr": saddle_end_mm}, cg_lo_mm, cg_hi_mm)
    st = ft.mesh_stats(tris)
    L = info["layout"]
    holes = sorted(h[1] for h in info["holes"] if h[0] == "ballast")
    nose_holes = [x for x in holes if x < 0]
    ticks = [t["x"] for t in info["ticks"]]
    default_station = min(nose_holes, key=lambda x: abs(x + ft.BALLAST_STATION_FRAC_OF_NOSE * L["nose"])) if nose_holes else None
    out = dict(tris=tris, mass_g=ft.mass_g(st["volume"]), cx=st["centroid"][0], nose=L["nose"], x_lo=L["x_lo"], saddle_end=L["x_hi"], length=L["x_hi"] - L["x_lo"], holes=holes,
               nose_holes=nose_holes, aft_holes=[x for x in holes if x > 0], default_station=default_station, tick_range=(min(ticks), max(ticks)) if ticks else (None, None),
               n_ticks=len(ticks), cg_lo=cg_lo_mm, cg_hi=cg_hi_mm)
    _RAIL_CACHE[key] = out
    return out


def printed_meshes2(g, span_mm, t, hinge_t=None):
    """bird_outline.printed_meshes with the v2 hinge strip thickness: main sheet, scored-hinge strip, fan tail (three overlapping closed shells)."""
    hinge_t = HINGE_V2_MM if hinge_t is None else hinge_t
    shells, polys = bo.printed_meshes(g, span_mm, t)
    strip = polys["strip"]
    shells = dict(shells, hinge=ft.prism(strip, 0.0, hinge_t))
    return shells, polys


def sheet_mass_g(g, size_key, exact=False):
    sz = SIZES[size_key]
    k, t = sz["span_mm"], sz["thickness_mm"]
    area = g["sheet_area"] * k * k
    if size_key == "foam":
        return area * 1e-6 * ft.FOAM_AREAL_MASS_G_M2
    if exact:
        shells, _ = printed_meshes2(g, k, t)
        return ft.mass_g(sum(ft.mesh_stats(s_)["volume"] for s_ in shells.values()))
    return ft.mass_g(area * t)                                                  # screening: area x thickness x density (hinge gap and strip neglected, < 1%)


def size_numbers(g, res, size_key, bend, exact=False, thickness_mm=None):
    """Mass, bare CG, nose (or aft-hole) ballast to reach the CL 0.3 trim CG, wing loading, pull-out radii.  Rail built for THIS outline."""
    sz = SIZES[size_key]
    k = sz["span_mm"]
    t = sz["thickness_mm"] if thickness_mm is None else thickness_mm
    if size_key == "foam" and thickness_mm is not None:
        m_sheet = g["sheet_area"] * k * k * 1e-6 * ft.FOAM_AREAL_MASS_G_M2 * thickness_mm / sz["thickness_mm"]     # ASSUMED: areal mass proportional to thickness
    else:
        m_sheet = sheet_mass_g(g, size_key, exact)
    cx_sheet = ft.polygon_centroid(g["full_outline"])[0] * k
    cgs = [res["trim"][c]["x_cg_span"] * k for c in TRIM_CLS]
    target = res["trim"][0.3]["x_cg_span"] * k
    hinge = g["x_h"] * k
    rail = make_rail(hinge - RAIL_HINGE_GAP_MM, min(cgs), max(cgs))
    M0 = m_sheet + rail["mass_g"]
    cg0 = (m_sheet * cx_sheet + rail["mass_g"] * rail["cx"]) / M0
    station = None
    if target <= cg0:                                                           # nose ballast
        cands = [x for x in rail["nose_holes"] if x < target - 0.5]
        if rail["default_station"] is not None and rail["default_station"] < target - 0.5:
            station = rail["default_station"]
        elif cands:
            station = min(cands)
    else:                                                                       # CG too far forward: ballast under the wing, aft of the target
        cands = [x for x in rail["aft_holes"] if x > target + 0.5]
        station = max(cands) if cands else None
    reachable = station is not None
    mb = M0 * (target - cg0) / (station - target) if reachable else float("inf")
    M = M0 + (mb if reachable else 0.0)
    W = M * 1e-3 * G_ACCEL
    S_w = g["S"] * k * k
    ws = W / (S_w * 1e-6)
    nose_tip = g["nose_x"] * k
    return dict(size=size_key, span=k, t=t, bend=bend, m_sheet=m_sheet, rail=rail, cx_sheet=cx_sheet, M0=M0, cg0=cg0, cg_target=target, cgs=cgs, station=station,
                ballast_g=mb, reachable=reachable, M=M, W=W, ws=ws, pull={c: bo.pullout_radius(ws, c) for c in PULLOUT_CLS}, nose_tip=nose_tip, hinge=hinge,
                ballast_frac=(mb / M if reachable else float("inf")), t_pct_mac=100 * t / (g["mac"] * k), area_sheet=g["sheet_area"] * k * k, S_w=S_w,
                rail_nose_beyond_sheet_nose=rail["nose"] + nose_tip,
                tick_margin_lo=(min(cgs) - rail["tick_range"][0]) if rail["tick_range"][0] is not None else None,
                tick_margin_hi=(rail["tick_range"][1] - max(cgs)) if rail["tick_range"][1] is not None else None)


def structure_numbers(g, res, sn, span_mm=250.0, bend=0.0):
    """Beam arithmetic for the PRINTED sheet (foamboard: UNKNOWN sandwich stiffness, not computed).  Loads at load factor N_LOAD from the AVL tail lift and wrist moment at CL 0.3.
      tail neck / hinge strip: cantilever of width = neck width, thickness = hinge strip, load = tail force from the moment balance (|CM0| x MAC / (0.3 x lever) x weight; the
      AVL tail-lift fraction is not used because it changes by tens of per cent between panellings), arm = tail centroid aft of the hinge; sigma = 6 M / (w t^2).
      wrist section: outer panel as a cantilever about the wrist, width = local chord at the wrist, thickness = plate; sigma = 6 M / (c t^2)."""
    k = span_mm
    W = sn["W"]
    lever = max((g["tail_centroid_x"] - g["x_ref"]) * k, 1.0)                           # tail centroid aft of the CM reference point, mm
    tail_load = abs(res["cm0"]) * g["mac"] * k / (0.3 * lever) * W * N_LOAD             # tail force that balances CM0 at zero wing lift, scaled to the weight at CL 0.3 (moment based: the AVL tail lift is panelling-noisy)
    arm = max((g["tail_centroid_x"] - g["x_h"]) * k, 0.1)
    w_neck = g["wt0"] * k
    t_h = HINGE_V2_MM
    sig_neck = 6.0 * tail_load * arm / (w_neck * t_h ** 2)
    allow = SIGMA_ALLOW_MPA / SAFETY_FACTOR
    w_min_stress = 6.0 * tail_load * arm / (t_h ** 2 * allow)
    neck_min = max(w_min_stress, NECK_MIN_FRAC_SPAN * k)
    m_w = res["m_wrist_over_w_m"] * 1000.0 * (k / RUN_SPAN_MM) * W * N_LOAD            # N mm, one outer panel about the wrist
    c_w = g["chord_wrist_local"] * k
    t = SIZES["printed"]["thickness_mm"]
    sig_wrist = 6.0 * m_w / (c_w * t ** 2)
    strain = 100.0 * t_h * math.radians(bend) / (2.0 * bo.HINGE_WIDTH_MM)                    # outer-fibre strain of a strip of length = hinge gap bent through the bend angle
    return dict(hinge_strain_pct=strain, tail_load_N=tail_load, arm_mm=arm, neck_width_mm=w_neck, hinge_t_mm=t_h, sigma_neck_MPa=sig_neck, neck_min_stress_mm=w_min_stress, neck_min_mm=neck_min,
                neck_ok=w_neck >= neck_min, wrist_moment_Nmm=m_w, wrist_chord_mm=c_w, sigma_wrist_MPa=sig_wrist, allow_MPa=allow)


# =============================================================================================== one candidate row
def candidate_row(fam, on, bend, res, g=None, exact_mass=False):
    if g is None:
        g = build_geometry2(feature_params(fam, on))
    row = dict(family=fam, on=dict(on), features="+".join(n for n in FEATURES if on.get(n)) or "none", bend=bend, S=g["S"], ar=g["ar"], mac=g["mac"], c_root=g["c_root"], g=g)
    row.update(np_pct_mac=res["np_pct_mac"], cm0=res["cm0"], cm0_per_deg=(res["cm0"] / bend if bend else None), cn_beta=res["cn_beta"], cl_beta=res["cl_beta"],
               span_eff=res["span_eff"], cdi=res["cdi_cl03"], tip_loading=res["tip_loading"], tail_lift_frac=res["tail_lift_frac"], sm03=res["trim"][0.3]["static_margin"],
               x_cg03_span=res["trim"][0.3]["x_cg_span"], sheet_area_mm2=g["sheet_area"] * 250.0 ** 2, tail_area_frac=g["tail_area"] / g["S"], nose_x_span=g["nose_x"],
               res=res)
    row["x_cg03_mm"] = row["x_cg03_span"] * 250.0
    sizes = {sk: size_numbers(g, res, sk, bend, exact_mass) for sk in ("printed", "foam")}
    row["sizes"] = sizes
    st = structure_numbers(g, res, sizes["printed"], 250.0, bend)
    row["structure"] = st
    row.update(mass_printed_g=sizes["printed"]["M"], sheet_mass_printed_g=sizes["printed"]["m_sheet"], ballast_printed_g=sizes["printed"]["ballast_g"], ballast_frac_printed=sizes["printed"]["ballast_frac"],
               ballast_frac_foam=sizes["foam"]["ballast_frac"], ballast_foam_g=sizes["foam"]["ballast_g"], ws_printed=sizes["printed"]["ws"], ws_foam=sizes["foam"]["ws"],
               sigma_neck=st["sigma_neck_MPa"], sigma_wrist=st["sigma_wrist_MPa"], neck_ok=st["neck_ok"], reachable_all=sizes["printed"]["reachable"] and sizes["foam"]["reachable"],
               valid_outline=polygon_is_simple(g["full_outline"]))
    return row


QUANT = [   # key, label, unit, direction (up / down = better, None = context only), source tag, relative neutrality threshold, absolute floor
    ("np_pct_mac", "neutral point", "% MAC", None, "AVL", 0.03, 1.5),
    ("cm0_per_deg", "CM0 per degree of tail bend", "per deg", None, "AVL", 0.03, 0.0003),
    ("x_cg03_mm", "trim CG at CL 0.3 (250 mm span)", "mm aft of root LE", None, "AVL", 0.03, 0.3),
    ("sm03", "static margin at CL 0.3", "fraction of MAC", None, "AVL", 0.03, 0.004),
    ("cn_beta", "Cn_beta, no fin", "per rad", "up", "AVL", 0.05, 0.0004),
    ("cl_beta", "Cl_beta, no fin", "per rad", "down", "AVL", 0.05, 0.003),
    ("span_eff", "span efficiency e (CL 0.3)", "", "up", "AVL", 0.05, 0.10),
    ("cdi", "induced drag CDi (CL 0.3)", "", "down", "AVL", 0.08, 0.0006),
    ("tip_loading", "tip loading indicator (outer 20% cl / CL)", "", "down", "AVL (indicator only)", 0.02, 0.03),
    ("sheet_area_mm2", "sheet area (250 mm)", "mm2", "down", "GEOMETRY", 0.01, 5.0),
    ("sheet_mass_printed_g", "sheet mass, printed 250 mm", "g", "down", "ARITHMETIC", 0.01, 0.02),
    ("ballast_frac_printed", "nose ballast / all-up mass, printed", "fraction", "down", "ARITHMETIC", 0.02, 0.005),
    ("ballast_frac_foam", "nose ballast / all-up mass, foam 400", "fraction", "down", "ARITHMETIC", 0.02, 0.005),
    ("ws_printed", "wing loading, printed 250 mm", "N/m2", "down", "ARITHMETIC", 0.01, 0.1),
    ("sigma_neck", "tail neck hinge stress (n=5)", "MPa", "down", "ARITHMETIC", 0.10, 0.3),
    ("sigma_wrist", "wrist section stress (n=5)", "MPa", "down", "ARITHMETIC", 0.10, 0.1),
]
QMAP = {q[0]: q for q in QUANT}
# a "modelled benefit" is a HELPS verdict in one of these (stability, efficiency, loading, structure, ballast); area / mass / wing loading are costs, not benefits
BENEFIT_KEYS = ("cn_beta", "cl_beta", "span_eff", "cdi", "tip_loading", "sigma_neck", "sigma_wrist", "ballast_frac_printed", "ballast_frac_foam")


def verdict(key, base_v, new_v):
    """-> 'helps' / 'hurts' / 'neutral' / 'context' (no preferred direction)."""
    _, _, _, direction, _, rel, floor = QMAP[key]
    if base_v is None or new_v is None or not all(math.isfinite(v) for v in (base_v, new_v)):
        return "context"
    d = new_v - base_v
    if abs(d) < max(rel * abs(base_v), floor):
        return "neutral"
    if direction is None:
        return "context"
    good = d > 0 if direction == "up" else d < 0
    return "helps" if good else "hurts"


def feature_effects(base_row, new_row):
    return {key: dict(base=base_row[key], new=new_row[key], delta=(new_row[key] - base_row[key]) if base_row[key] is not None and new_row[key] is not None else None,
                      verdict=verdict(key, base_row[key], new_row[key])) for key, *_ in QUANT}


def modelled_benefits(eff):
    return [k for k in BENEFIT_KEYS if eff[k]["verdict"] == "helps"]


def modelled_costs(eff):
    return [k for k in ("sheet_area_mm2", "sheet_mass_printed_g", "ws_printed") + BENEFIT_KEYS if eff[k]["verdict"] == "hurts"]


# =============================================================================================== selection (pure logic on a table of dict rows)
APPEARANCE_COST_MAX = 0.04            # appearance-only features may add at most this fraction of the all-up mass (JUDGEMENT)
NOSE_CLEAR_SPAN = 0.012               # trim CG must be at least this fraction of the span (3 mm at 250 mm) behind the nose tip
TIP_TOL = 0.01


def criteria_failures(r, base_tip_loading):
    """Reasons a candidate row fails the selection criteria (empty list = passes).  Row keys: cn_beta, cl_beta, sm03, x_cg03_span, nose_x_span, ballast_frac_printed,
    ballast_frac_foam, reachable_all, tip_loading, neck_ok, appearance_cost_frac, valid_outline."""
    f = []
    if r["cn_beta"] < CN_BETA_MIN:
        f.append(f"Cn_beta {r['cn_beta']:+.4f} below {CN_BETA_MIN} /rad (no fin)")
    if r["cl_beta"] > CL_BETA_MAX:
        f.append(f"Cl_beta {r['cl_beta']:+.4f} not below {CL_BETA_MAX} /rad")
    if not (SM_WINDOW[0] <= r["sm03"] <= SM_WINDOW[1]):
        f.append(f"static margin at CL 0.3 {100 * r['sm03']:.1f}% outside {100 * SM_WINDOW[0]:g}-{100 * SM_WINDOW[1]:g}% MAC")
    if r["x_cg03_span"] < r["nose_x_span"] + NOSE_CLEAR_SPAN:
        f.append(f"trim CG {r['x_cg03_span'] * 250:.1f} mm is not at least {NOSE_CLEAR_SPAN * 250:.0f} mm behind the nose tip ({r['nose_x_span'] * 250:.1f} mm)")
    if not r["reachable_all"]:
        f.append("trim CG not reachable with the rail ballast holes")
    for sk in ("printed", "foam"):
        fr = r[f"ballast_frac_{sk}"]
        if fr >= BALLAST_MAX_FRAC:
            f.append(f"ballast {'unreachable' if not math.isfinite(fr) else format(100 * fr, '.0f') + '% of all-up mass'} at {sk} size (limit {100 * BALLAST_MAX_FRAC:g}%)")
    if r["tip_loading"] > base_tip_loading * (1 + TIP_TOL):
        f.append(f"tip loading {r['tip_loading']:.3f} above v1 value {base_tip_loading:.3f}")
    if not r["neck_ok"]:
        f.append("tail neck narrower than the structural minimum")
    if r["appearance_cost_frac"] > APPEARANCE_COST_MAX:
        f.append(f"appearance-only features add {100 * r['appearance_cost_frac']:.1f}% of all-up mass (limit {100 * APPEARANCE_COST_MAX:g}%)")
    if not r["valid_outline"]:
        f.append("outline self-intersects")
    return f


def margin_of(r, base_tip_loading):
    """Smallest normalised margin to the thresholds (larger = more robust)."""
    m = [(r["cn_beta"] - CN_BETA_MIN) / CN_BETA_MIN, (CL_BETA_MAX - r["cl_beta"]) / abs(CL_BETA_MAX),
         min(r["sm03"] - SM_WINDOW[0], SM_WINDOW[1] - r["sm03"]) / (SM_WINDOW[1] - SM_WINDOW[0]),
         (BALLAST_MAX_FRAC - max(r["ballast_frac_printed"], r["ballast_frac_foam"])) / BALLAST_MAX_FRAC,
         (base_tip_loading * (1 + TIP_TOL) - r["tip_loading"]) / base_tip_loading]
    return min(m)


def rank_key(r):
    """Among passers: fewest fins, then the most features that have a modelled benefit (single-feature test), then the most outline features switched on
    (Kai asked for a bird outline: JUDGEMENT), then the largest minimum normalised margin to the thresholds, then the smallest sheet area."""
    return (r["n_fins"], -r.get("n_justified", 0), -r["n_features"], -r["margin"], r["sheet_area_mm2"])


def eff_fails(r, waive=()):
    """Failures that count: a waived criterion (a tuple of message prefixes) is ignored.  Waiving is used ONLY when nothing passes everything, and is reported."""
    return [x for x in r["fails"] if not any(x.startswith(w) for w in waive)]


def rank_key_waived(r):
    return (r["n_fins"], round(max(r["ballast_frac_printed"], r["ballast_frac_foam"]), 2), -r.get("n_justified", 0), -r["n_features"], -r["margin"], r["sheet_area_mm2"])


def select(rows, waive=()):
    """rows carry 'fails' (list) plus the rank keys.  -> (passers sorted best first, failers).  With a waiver the ranking puts the waived quantity first."""
    key = rank_key_waived if waive else rank_key
    return sorted([r for r in rows if not eff_fails(r, waive)], key=key), [r for r in rows if eff_fails(r, waive)]


# =============================================================================================== analysis pipeline
LIN_KEYS = ("cm0", "tail_lift_frac", "m_wrist_over_w_m", "span_eff", "cdi_cl03", "tip_loading", "cn_beta", "cl_beta")
YAW_OPTIONS = (("none", dict()), ("ventral fin 3%", dict(vfin=0.03)), ("ventral fin 5%", dict(vfin=0.05)),
               (f"tail edges up {EDGE_FIN_ANGLES[0]:g} deg", dict(edge=(EDGE_FIN_FRAC, EDGE_FIN_ANGLES[0]))), (f"tail edges up {EDGE_FIN_ANGLES[1]:g} deg", dict(edge=(EDGE_FIN_FRAC, EDGE_FIN_ANGLES[1]))))
IMPACT_KEYS = ("cn_beta", "cl_beta", "span_eff", "tip_loading", "sm03", "np_pct_mac")


def aero_combos(quick=False):
    combos = [dict(zip(AERO_FEATURES, bits)) for bits in itertools.product((0, 1), repeat=len(AERO_FEATURES))]
    if quick:
        combos = [c for c in combos if sum(c.values()) <= 1 or all(c.values())]
    return combos


def interp_bend(res0, res5, bend):
    """AVL is linear in the tail incidence: values at another bend from the bend-0 and bend-5 runs (SCREENING; finalists are re-run exactly)."""
    f = bend / 5.0
    r = dict(res5)
    for k in LIN_KEYS:
        r[k] = res0[k] + (res5[k] - res0[k]) * f
    mac = None
    return r


def retrim(r, mac):
    r["trim"] = {c: dict(x_cg_span=r["np_span"] - mac * r["cm0"] / c, static_margin=r["cm0"] / c) for c in TRIM_CLS}
    return r


_GEOM_CACHE = {}


def geom_for(fam, on):
    key = (fam, tuple(sorted((n, round(float(s), 6)) for n, s in on.items() if s)))
    if key not in _GEOM_CACHE:
        _GEOM_CACHE[key] = build_geometry2(feature_params(fam, on))
    return _GEOM_CACHE[key]


def finish_row(row, fam, appearance, base_tip, n_fins=0):
    """Add rank keys and the criteria verdict to a candidate row."""
    on = row["on"]
    row["n_features"] = sum(1 for v in on.values() if v)
    row["n_justified"] = sum(1 for n, v in on.items() if v and n not in appearance)
    row["n_fins"] = n_fins
    off = {n: (0 if n in appearance else s) for n, s in on.items()}
    g_off = geom_for(fam, off)
    cost = 0.0
    for sk in ("printed", "foam"):
        k = SIZES[sk]["span_mm"]
        d_area = (row["g"]["sheet_area"] - g_off["sheet_area"]) * k * k
        dm = ft.mass_g(d_area * SIZES[sk]["thickness_mm"]) if sk == "printed" else d_area * 1e-6 * ft.FOAM_AREAL_MASS_G_M2
        if math.isfinite(row["sizes"][sk]["M"]):
            cost = max(cost, dm / row["sizes"][sk]["M"])
    row["appearance_cost_frac"] = max(cost, 0.0)
    row["fails"] = criteria_failures(row, base_tip)
    row["margin"] = margin_of(row, base_tip) if math.isfinite(row["ballast_frac_printed"]) and math.isfinite(row["ballast_frac_foam"]) else -9.0
    return row


def single_feature_table(fam, runner):
    """Each feature alone, at strength 1 and 0.5, against the v1 family outline, at REF_BEND.  The two body-only features reuse the base AVL result."""
    base_res = runner.run(fam, {}, REF_BEND)
    base = candidate_row(fam, {}, REF_BEND, base_res)
    rows, eff, half = {}, {}, {}
    for f in FEATURES:
        for s in (1.0, 0.5):
            on = {f: s}
            res = runner.run(fam, on, REF_BEND) if f in AERO_FEATURES else base_res
            row = candidate_row(fam, on, REF_BEND, res)
            if s == 1.0:
                rows[f] = row
                eff[f] = feature_effects(base, row)
            else:
                half[f] = row
    return base, rows, eff, half


def impact_score(eff_map):
    sc = 0.0
    for k in IMPACT_KEYS:
        e = eff_map[k]
        scale = max(abs(e["base"]), QMAP[k][6] * 5, 1e-9)
        sc += abs(e["delta"]) / scale
    return sc


def interactions(fam, runner, top):
    """Combined effect of 2 and 3 features vs the sum of their single effects (AVL at REF_BEND)."""
    base_res = runner.run(fam, {}, REF_BEND)
    out = []
    keys = ("np_pct_mac", "cm0_per_deg", "sm03", "cn_beta", "cl_beta", "span_eff", "tip_loading")
    singles = {f: runner.run(fam, {f: 1}, REF_BEND) for f in top}
    g0 = geom_for(fam, {})

    def val(res, k):
        if k == "cm0_per_deg":
            return res["cm0"] / REF_BEND
        if k == "sm03":
            return res["trim"][0.3]["static_margin"]
        return res[k]
    for r_ in (2, 3):
        for sub in itertools.combinations(top, r_):
            comb = runner.run(fam, {f: 1 for f in sub}, REF_BEND)
            for k in keys:
                b = val(base_res, k)
                d_each = [val(singles[f], k) - b for f in sub]
                d_comb = val(comb, k) - b
                out.append(dict(family=fam, combo="+".join(sub), quantity=k, base=b, sum_of_singles=sum(d_each), combined=d_comb, interaction=d_comb - sum(d_each),
                                singles="; ".join(f"{f} {d:+.5f}" for f, d in zip(sub, d_each))))
    return out


def screen_family(fam, runner, appearance, quick=False):
    base0, base5 = runner.run(fam, {}, 0.0), runner.run(fam, {}, 5.0)
    g0 = geom_for(fam, {})
    base_tip = {b: interp_bend(base0, base5, b)["tip_loading"] for b in SELECTION_BENDS}
    rows = []
    for aero in aero_combos(quick):
        r0, r5 = runner.run(fam, aero, 0.0), runner.run(fam, aero, 5.0)
        for rb in (0, 1):
            for nz in (0, 1):
                on = {n: s for n, s in dict(aero, root_blend=rb, nose=nz).items() if s}
                g = geom_for(fam, on)
                for bend in SELECTION_BENDS:
                    res = retrim(interp_bend(r0, r5, bend), g["mac"])
                    row = candidate_row(fam, on, bend, res, g)
                    finish_row(row, fam, appearance, base_tip[bend])
                    rows.append(row)
    return rows, base_tip


def exact_stage(fam, ranked, runner, appearance, want=3, max_runs=24, waive=()):
    """Re-run the best screened candidates exactly at their own bend and re-check every criterion."""
    log, finals = [], []
    n = 0
    for r in ranked:
        if n >= max_runs or len(finals) >= want:
            break
        res = runner.run(fam, r["on"], r["bend"])
        n += 1
        base_exact = runner.run(fam, {}, r["bend"])
        row = candidate_row(fam, r["on"], r["bend"], res, r["g"])
        finish_row(row, fam, appearance, base_exact["tip_loading"])
        row["stage"] = "exact re-run"
        log.append(row)
        if not eff_fails(row, waive):
            finals.append(row)
    return finals, log


def fin_stage(fam, failers, runner, appearance, n_try=8, waive=()):
    """No candidate passes without a fin: try the smallest ventral fins (3%, then 5%) on the failers whose only failure is Cn_beta."""
    only_cn = [r for r in failers if eff_fails(r, waive) and all(s.startswith("Cn_beta") for s in eff_fails(r, waive))]
    only_cn.sort(key=lambda r: (-r["n_features"], -r["cn_beta"]))
    seen, out, log = set(), [], []
    for r in only_cn:
        k = (r["features"], r["bend"])
        if k in seen:
            continue
        seen.add(k)
        if len(seen) > n_try:
            break
        for i, fa in enumerate(FIN_AREAS):
            res = runner.run(fam, r["on"], r["bend"], vfin=fa)
            base_exact = runner.run(fam, {}, r["bend"])
            row = candidate_row(fam, r["on"], r["bend"], res, r["g"])
            finish_row(row, fam, appearance, base_exact["tip_loading"], n_fins=i + 1)
            row["stage"] = f"ventral fin {100 * fa:g}%"
            row["fin"] = fa
            log.append(row)
            if not eff_fails(row, waive):
                out.append(row)
                break
    return out, log


def leave_one_out(fam, final, runner):
    """In the context of the final outline: switch each of its features off alone and tabulate the change (the honest 'kept because')."""
    res_f = runner.run(fam, final["on"], final["bend"], vfin=final.get("fin"))
    full = candidate_row(fam, final["on"], final["bend"], res_f, final["g"])
    out = {}
    for f in FEATURES:
        if not final["on"].get(f):
            continue
        on2 = {n: s for n, s in final["on"].items() if n != f}
        res2 = runner.run(fam, on2, final["bend"], vfin=final.get("fin")) if f in AERO_FEATURES else res_f
        row2 = candidate_row(fam, on2, final["bend"], res2)
        out[f] = feature_effects(row2, full)               # effect of ADDING the feature to the final-without-it
    return full, out


def kept_because(single_eff, loo_eff):
    """Words for the table: benefit (from leave-one-out, then single), costs, or APPEARANCE ONLY with its modelled cost."""
    def label(k):
        return QMAP[k][1]

    def fmt(k, e):
        d = e[k]["delta"]
        base = e[k]["base"]
        return f"{label(k)} {base:.4g} to {e[k]['new']:.4g}"
    ben = modelled_benefits(loo_eff)
    costs = [k for k in ("sheet_area_mm2", "sheet_mass_printed_g", "ws_printed") if loo_eff[k]["verdict"] == "hurts"] + [k for k in BENEFIT_KEYS if loo_eff[k]["verdict"] == "hurts"]
    if ben:
        s = "kept for a modelled benefit: " + "; ".join(fmt(k, loo_eff) for k in ben)
        if costs:
            s += ". Cost: " + "; ".join(fmt(k, loo_eff) for k in costs[:3])
        return s, "benefit"
    s = "APPEARANCE ONLY (no modelled benefit)"
    if costs:
        s += ". Modelled cost: " + "; ".join(fmt(k, loo_eff) for k in costs[:3])
    return s, "appearance"


def analyse(runner, quick=False, log=print):
    A = {}
    for fam in FAM_ORDER:
        log(f"[{fam}] single features")
        base, singles, eff, half = single_feature_table(fam, runner)
        runner.save()
        appearance = {f for f in FEATURES if not modelled_benefits(eff[f])}
        scores = sorted(((f, impact_score(eff[f])) for f in AERO_FEATURES), key=lambda t: -t[1])
        top = [f for f, _ in scores[:3]]
        log(f"[{fam}] appearance-only by single test: {sorted(appearance)}; top aero by impact: {top}")
        log(f"[{fam}] screening")
        rows, base_tip = screen_family(fam, runner, appearance, quick)
        runner.save()
        ia = interactions(fam, runner, top)
        waive = ()
        ranked, failers = select(rows)
        log(f"[{fam}] screened {len(rows)} candidates, {len(ranked)} pass screening")
        finals, exact_log = exact_stage(fam, ranked, runner, appearance)
        fin_log = []
        if not finals:
            finals, fin_log = fin_stage(fam, failers, runner, appearance)
        if not finals:                                                   # nothing passes everything: waive the ballast limit, say so, rank by ballast
            waive = ("ballast",)
            ranked_w, failers_w = select(rows, waive)
            log(f"[{fam}] nothing passes every criterion; ballast limit waived: {len(ranked_w)} candidates pass the rest")
            finals, exact_log2 = exact_stage(fam, ranked_w, runner, appearance, waive=waive)
            exact_log += exact_log2
            if not finals:
                finals, fin_log2 = fin_stage(fam, failers_w, runner, appearance, waive=waive)
                fin_log += fin_log2
        used_fin = bool(finals) and finals[0]["n_fins"] > 0
        flagged = False
        if finals:
            finals.sort(key=rank_key_waived if waive else rank_key)
            final = finals[0]
        else:
            flagged = True
            pool = exact_log + fin_log + rows
            final = min(pool, key=lambda r: (len(r["fails"]), r["n_fins"], -r["margin"]))
            log(f"[{fam}] NO candidate passes even with the ballast limit waived; best effort {final['features']} bend {final['bend']}: {final['fails']}")
        final["waived"] = list(waive)
        runner.save()
        log(f"[{fam}] final {final['features']} bend {final['bend']} fin {final.get('fin')}")
        full, loo = leave_one_out(fam, final, runner)
        A[fam] = dict(base=base, singles=singles, eff=eff, half=half, appearance=appearance, scores=scores, top=top, interactions=ia, rows=rows, base_tip=base_tip, ranked=ranked, waive=waive,
                      exact_log=exact_log, fin_log=fin_log, finals=finals, final=final, flagged=flagged, used_fin=used_fin, loo=loo, full=full)
    return A


def final_tables(A, runner, log=print):
    """Exact bends table (v1 baseline and final) and the yaw options at every bend."""
    for fam in FAM_ORDER:
        a = A[fam]
        fin = a["final"].get("fin")
        bends = {}
        for label, on in (("v1", {}), ("final", a["final"]["on"])):
            for b in BENDS:
                res = runner.run(fam, on, b, vfin=(fin if label == "final" else None))
                row = candidate_row(fam, on, b, res)
                base_exact = runner.run(fam, {}, b)
                finish_row(row, fam, a["appearance"], base_exact["tip_loading"], n_fins=(FIN_AREAS.index(fin) + 1 if (fin and label == "final") else 0))
                bends[(label, b)] = row
        yaw = []
        for label, on in (("v1", {}), ("final", a["final"]["on"])):
            for b in (BENDS if label == "final" else (REF_BEND,)):
                for oname, kw in YAW_OPTIONS:
                    res = runner.run(fam, on, b, **kw)
                    yaw.append(dict(family=fam, outline=label, bend=b, option=oname, cn_beta=res["cn_beta"], cl_beta=res["cl_beta"], np_pct_mac=res["np_pct_mac"], cm0=res["cm0"],
                                    sm03=res["trim"][0.3]["static_margin"], span_eff=res["span_eff"], cn_ok=res["cn_beta"] >= CN_BETA_MIN, cl_ok=res["cl_beta"] <= CL_BETA_MAX))
        a["bends"], a["yaw"] = bends, yaw
        runner.save()
        log(f"[{fam}] bends and yaw tables done")
    return A


def passing_bends(a):
    """Bends of the final outline whose exact row passes every criterion (the final's own bend first)."""
    out = []
    for b in SELECTION_BENDS:
        r = a["bends"][("final", b)]
        if not eff_fails(r, a["waive"]):
            out.append(b)
    if a["final"]["bend"] not in out:
        out.append(a["final"]["bend"])
    out = sorted(set(out), key=lambda b: (b != a["final"]["bend"], abs(b - a["final"]["bend"])))
    return out[:3]


# =============================================================================================== builds: STL, DXF, drawing, rails, templates
LETTER_W, LETTER_H, MARGIN = ft.LETTER_W, ft.LETTER_H, ft.TEMPLATE_MARGIN
TOP_RESERVE = 40.0                    # mm at the top of every template page for the title, instructions and the scale bar (the drawing never enters it)
OVERLAP_MM = bo.OVERLAP_MM
LABEL = {"BS": "swift-like", "BC": "crow-like"}


def _mask(o):
    W, H, M = LETTER_W, LETTER_H, MARGIN
    for x, y, w, h in ((0, 0, M, H), (W - M, 0, M, H), (0, 0, W, M), (0, H - M, W, M)):
        o.append(f'<rect x="{x:.2f}" y="{y:.2f}" width="{w:.2f}" height="{h:.2f}" fill="#ffffff"/>')


def _backed_text(o, x, y, s, size=2.6, anchor="start", color="#000", width_mm=None):
    """White backing rectangle behind a label so no outline or tick runs through the letters."""
    w = width_mm or 0.52 * size * len(s)
    x0 = x if anchor == "start" else (x - w if anchor == "end" else x - w / 2)
    o.append(f'<rect x="{x0 - 0.6:.2f}" y="{y - size:.2f}" width="{w + 1.2:.2f}" height="{size * 1.35:.2f}" fill="#ffffff"/>')
    o.append(ft._t(x, y, s, size, anchor, color))


def template_pages2(g, span_mm, label, material, thickness_mm, bends, res_by_bend):
    """1:1 US Letter portrait templates (rect / line / polygon / text only).  v1 label overlaps fixed: the title, instructions and the 100 mm scale bar live in a reserved
    top band (y < 40 mm) with the bar at the right; centreline, tick numbers, root label and CG legend sit on separate lines below the centreline; tick numbers are
    drawn only when they fit inside the margins; overlap labels have a white backing and stay below the title band."""
    k = span_mm
    half = [(x * k, y * k) for x, y in g["half_outline"]]
    xs = [p[0] for p in half]
    xmin, xmax = min(xs) - 4.0, max(xs) + 4.0
    pw = LETTER_W - 2 * MARGIN
    tiles = [xmin]
    while tiles[-1] + pw < xmax - 1e-9:
        tiles.append(tiles[-1] + pw - OVERLAP_MM)
    ycl = TOP_RESERVE + g["half_span"] * k + 2.0
    xh, wt0 = g["x_h"] * k, g["wt0"] * k
    cgs = [(b, res_by_bend[b]["trim"][0.3]["x_cg_span"] * k) for b in bends]
    pages = []
    lo_ok, hi_ok = MARGIN + 4.0, LETTER_W - MARGIN - 4.0
    for ti, t0 in enumerate(tiles):
        X = lambda x: MARGIN + (x - t0)
        Y = lambda y: ycl - y
        o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{LETTER_W}mm" height="{LETTER_H}mm" viewBox="0 0 {LETTER_W} {LETTER_H}">', '<rect width="100%" height="100%" fill="white"/>']
        o.append(f'<polygon points="{" ".join(f"{X(x):.3f},{Y(y):.3f}" for x, y in half)}" fill="#eef3f8" fill-opacity="0.6" stroke="#000" stroke-width="0.5"/>')
        o.append(ft._l(X(xh), Y(0), X(xh), Y(wt0 / 2), "#c2621a", 0.7, "3 1.5"))
        o.append(ft._l(X(g["x_wrist"] * k), Y(g["y_wrist"] * k), X((g["x_wrist"] + g["c_wrist"]) * k), Y(g["y_wrist"] * k), "#2b6cb0", 0.35, "2 1.5"))
        o.append(ft._l(X(xmin), Y(0), X(xmax), Y(0), "#555", 0.35, "3 2"))
        for xm in range(int(math.ceil(xmin)), int(xmax) + 1):
            if xm % 5:
                continue
            ln = 5.0 if xm % 10 == 0 else 2.5
            o.append(ft._l(X(xm), Y(0) - ln, X(xm), Y(0), "#000", 0.3))
            if xm % 10 == 0 and lo_ok <= X(xm) <= hi_ok:
                o.append(ft._t(X(xm), Y(0) + 5.0, f"{xm}", 2.6, "middle"))
        o.append(ft._l(X(0), Y(0) - 9, X(0), Y(0) + 9, "#c2621a", 0.6))
        if lo_ok <= X(0) <= hi_ok - 40:
            o.append(ft._t(X(0) + 1.0, Y(0) + 10.5, "0 = wing root leading edge", 2.8, color="#c2621a", weight="bold"))
        for i, (b, xm) in enumerate(cgs):
            o.append(ft._l(X(xm), Y(0) - 12, X(xm), Y(0) + 2, "#1a7f4b", 0.6))
            if lo_ok <= X(xm) <= hi_ok:
                o.append(ft._t(X(xm), Y(0) - 13.5 - 3.0 * i, f"{b:g} deg", 2.4, "middle", "#1a7f4b"))
        o.append(ft._t(max(lo_ok, X(max(xmin, t0 + 2))), Y(0) + 14.5, "centreline: fold or flip here for the other half", 2.6, color="#555"))
        o.append(ft._t(max(lo_ok, X(max(xmin, t0 + 2))), Y(0) + 18.5, "green ticks: AVL trim CG at CL 0.3 for the bend written above each (reference points)", 2.4, color="#1a7f4b"))
        xt = X(xh)
        if lo_ok <= xt <= hi_ok:
            if xt < hi_ok - 60:
                o.append(ft._t(xt + 2.0, Y(wt0 / 2) - 2.0, "hinge: bend tail UP (trailing edge up)", 2.6, color="#c2621a"))
            else:
                o.append(ft._t(xt - 2.0, Y(wt0 / 2) - 2.0, "hinge: bend tail UP (trailing edge up)", 2.6, "end", "#c2621a"))
        xw_ = X(g["x_wrist"] * k)
        if lo_ok <= xw_ <= hi_ok - 30:
            o.append(ft._t(xw_ + 1.0, Y(g["y_wrist"] * k) - 1.5, "wrist line (marker)", 2.4, color="#2b6cb0"))
        _mask(o)
        if len(tiles) > 1:
            for j, (xb, col, txt, anchor) in enumerate(([(tiles[ti + 1], "#c2621a", f"overlap: page {ti + 2} starts here", "end")] if ti + 1 < len(tiles) else [])
                                                      + ([(tiles[ti - 1] + pw, "#2b6cb0", f"overlap: page {ti} ended here", "start")] if ti > 0 else [])):
                o.append(ft._l(X(xb), TOP_RESERVE + 2, X(xb), LETTER_H - MARGIN - 2, col, 0.4, "4 2"))
                _backed_text(o, X(xb) + (-1.5 if anchor == "end" else 1.5), TOP_RESERVE + 6 + 5 * j, txt, 2.6, anchor, col)
                for yy in (TOP_RESERVE + 25, TOP_RESERVE + 100):
                    o.append(ft._l(X(xb) - 3, yy, X(xb) + 3, yy, col, 0.4))
        o.append(ft._t(12, 14, f"{label}: page {ti + 1} of {len(tiles)}", 3.8, weight="bold"))
        for j, line in enumerate(("ONE HALF of the sheet with its fan tail, 1:1, Letter portrait.", "Cut the solid outline; flip on the centreline for the other half.",
                                  "Print at 100 % (actual size). Check the 100 mm bar with a ruler.", f"{material} {thickness_mm:g} mm; span {span_mm:g} mm (thickness ASSUMED: measure yours).",
                                  f"Hinge (orange): bend tail UP by {', '.join(f'{b:g}' for b in bends)} deg; see notes page.")):
            o.append(ft._t(12, 20 + 4.2 * j, line, 2.6))
        o.extend(ft._scale_bar(LETTER_W - MARGIN - 102.0, 16.0))
        o.append("</svg>")
        pages.append("\n".join(o))
    # ---- notes page
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{LETTER_W}mm" height="{LETTER_H}mm" viewBox="0 0 {LETTER_W} {LETTER_H}">', '<rect width="100%" height="100%" fill="white"/>',
         ft._t(12, 14, f"{label}: build notes", 3.8, weight="bold"),
         ft._t(12, 20, "Tail bend: rotate the fan tail about the hinge, trailing edge UP.", 2.6),
         ft._t(12, 24.2, "Score the top face, bend up to the gauge, glue the crease.", 2.6),
         ft._t(12, 28.4, "One sheet, one bend. These are build instructions,", 2.6),
         ft._t(12, 32.6, "not a prediction that the glider pulls out.", 2.6)]
    for i, b in enumerate(bends):
        base = 120.0
        rise = base * math.tan(math.radians(b))
        yy = 90.0 + i * 55.0
        o.append(f'<polygon points="{MARGIN + 10:.2f},{yy:.2f} {MARGIN + 10 + base:.2f},{yy:.2f} {MARGIN + 10 + base:.2f},{yy - rise:.2f}" fill="#fbeedd" fill-opacity="0.6" stroke="#000" stroke-width="0.4"/>')
        o.append(ft._t(MARGIN + 12, yy + 5.5, f"gauge for a {b:g} deg bend: lay the wing along the long edge; the tail follows the slanted edge", 2.6))
        o.append(ft._t(MARGIN + 12 + base * 0.45, yy - 2.0 - 0.5 * i, f"{b:g} deg", 3.0, color="#c2621a", weight="bold"))
    o.append(ft._t(12, 250.0, "Wrist line: a marker only; the sheet is not cut or creased there.", 2.6))
    o.append(ft._t(12, 255.0, "CG: knife edge. The 0 mark of the printed rail goes at the wing root LE.", 2.6))
    o.extend(ft._scale_bar(LETTER_W - MARGIN - 102.0, 16.0))
    o.append("</svg>")
    pages.append("\n".join(o))
    return pages


def build_final(fam, a, out: Path, make_pdf_pages=True):
    """All files for one family: both sizes.  -> dict with build numbers and file lists."""
    g = a["final"]["g"]
    bends = passing_bends(a)
    rows = {b: a["bends"][("final", b)] for b in bends}
    res_b = {b: rows[b]["res"] for b in bends}
    info = dict(fam=fam, bends=bends, files=[], builds=[], svgs={}, template_pages=[])
    cgs_all = {}
    for sk in ("printed", "foam"):
        sz = SIZES[sk]
        k = sz["span_mm"]
        folder = out / f"{fam}_{sk}_span{int(k)}"
        folder.mkdir(parents=True, exist_ok=True)
        cgs = [res_b[b]["trim"][c]["x_cg_span"] * k for b in bends for c in TRIM_CLS]
        hinge = g["x_h"] * k
        rail = make_rail(hinge - RAIL_HINGE_GAP_MM, min(cgs), max(cgs))
        cgs_all[sk] = (min(cgs), max(cgs), rail)
        ft.write_stl(folder / "rail.stl", rail["tris"], f"{fam} {sk} CG rail")
        rpt = ft.edge_report(rail["tris"], nd=4)
        assert rpt[1:] == (0, 0, 0), ("rail", fam, sk, rpt)
        marks = [(f"trim CG CL0.3 bend {b:g}", res_b[b]["trim"][0.3]["x_cg_span"] * k) for b in bends]
        bo.write_dxf_bird(folder / "parts.dxf", g, k, marks)
        (folder / "drawing.svg").write_text(bo.drawing_svg_bird(g, k, f"{fam} {LABEL[fam]} v2", bends, res_b, None, sk), encoding="utf-8")
        info["svgs"][sk] = f"{folder.name}/drawing.svg"
        fl = f"{fam} {sz['material']} {k:g} mm: <a href='{folder.name}/drawing.svg'>drawing.svg</a>, <a href='{folder.name}/parts.dxf'>parts.dxf</a>, <a href='{folder.name}/rail.stl'>rail.stl</a> (new CG rail)"
        if sk == "printed":
            shells, _ = printed_meshes2(g, k, sz["thickness_mm"])
            tris = [t for s_ in shells.values() for t in s_]
            ft.write_stl(folder / "sheet.stl", tris, f"{fam} v2 bird sheet (3 shells)")
            for nm, s_ in shells.items():
                rp = ft.edge_report(s_, nd=4)
                assert rp[1:] == (0, 0, 0), (fam, nm, rp)
            fl += f", <a href='{folder.name}/sheet.stl'>sheet.stl</a> (flat, scored hinge: one {HINGE_V2_MM:g} mm strip, {bo.HINGE_WIDTH_MM:g} mm wide; three overlapping closed shells)"
        pages = template_pages2(g, k, f"{fam} {LABEL[fam]} {k:g} mm", sz["material"], sz["thickness_mm"], bends, res_b)
        names = []
        for i, svg in enumerate(pages):
            nm = f"template_page{i + 1}.svg"
            (folder / nm).write_text(svg, encoding="utf-8")
            names.append(folder / nm)
        info["template_pages"] += [p for p in names]
        fl += ", templates: " + ", ".join(f"<a href='{folder.name}/{p.name}'>{p.name}</a>" for p in names)
        info["files"].append(fl)
        for b in bends:
            nb = size_numbers(g, res_b[b], sk, b, exact=True)
            nb["rail_union"] = rail
            nb["rail_covers_all_bends"] = (rail["tick_range"][0] is not None and rail["tick_range"][0] <= min(cgs) and rail["tick_range"][1] >= max(cgs))
            nb["rail_fits_ahead_of_hinge"] = rail["saddle_end"] <= hinge - RAIL_HINGE_GAP_MM + 1e-9
            nb["structure"] = structure_numbers(g, res_b[b], nb, 250.0, b) if sk == "printed" else None
            nb["fam"] = fam
            info["builds"].append(nb)
    # thinner sheet note for the 400 mm foam size
    k = SIZES["foam"]["span_mm"]
    t_thin = THIN_SHEET_MAC_PCT / 100.0 * g["mac"] * k
    nb = size_numbers(g, res_b[bends[0]], "foam", bends[0], exact=False, thickness_mm=t_thin)
    info["thin"] = dict(t_mm=t_thin, t_pct_mac_now=100 * SIZES["foam"]["thickness_mm"] / (g["mac"] * k), mass_g=nb["M"], sheet_g=nb["m_sheet"], ballast_g=nb["ballast_g"], ws=nb["ws"], bend=bends[0])
    return info


# =============================================================================================== CSV tables
def write_csv(path: Path, header, rows):
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def f_(v, n=4):
    if v is None:
        return ""
    if isinstance(v, float):
        return "inf" if not math.isfinite(v) else f"{v:.{n}g}"
    return str(v)


def write_tables(out: Path, A, builds, panel):
    rows = []
    for fam in FAM_ORDER:
        a = A[fam]
        for f in FEATURES:
            for key, label, unit, direction, src, rel, floor in QUANT:
                e = a["eff"][f][key]
                rows.append([fam, f, key, label, unit, f_(e["base"]), f_(e["new"]), f_(e["delta"]), e["verdict"], src])
    write_csv(out / "features_single.csv", ["family", "feature", "quantity_key", "quantity", "unit", "v1_value", "with_feature_strength1", "change", "verdict", "tag"], rows)
    rows = []
    for fam in FAM_ORDER:
        a = A[fam]
        for f in FEATURES:
            for s, row in ((0.0, a["base"]), (0.5, a["half"][f]), (1.0, a["singles"][f])):
                rows.append([fam, f, s] + [f_(row[k]) for k, *_ in QUANT])
    write_csv(out / "features_strength.csv", ["family", "feature", "strength"] + [q[0] for q in QUANT], rows)
    write_csv(out / "interactions.csv", ["family", "features", "quantity", "v1_value", "sum_of_single_changes", "combined_change", "interaction (combined - sum)", "single_changes"],
              [[r["family"], r["combo"], r["quantity"], f_(r["base"]), f_(r["sum_of_singles"]), f_(r["combined"]), f_(r["interaction"]), r["singles"]] for fam in FAM_ORDER for r in A[fam]["interactions"]])
    hdr = ["family", "outline", "bend_deg", "NP_pct_MAC", "CM0", "CM0_per_deg", "trim_CG_CL0.2_mm_250", "trim_CG_CL0.3_mm_250", "trim_CG_CL0.5_mm_250", "static_margin_CL0.3_pct", "Cn_beta", "Cl_beta", "span_eff",
           "CDi_CL0.3", "tip_loading", "tail_lift_over_weight", "ballast_g_250", "ballast_frac_250", "ballast_g_400", "ballast_frac_400", "all_up_g_250", "wing_loading_N_m2_250", "neck_stress_MPa", "neck_width_mm",
           "neck_min_mm", "wrist_stress_MPa", "passes_all_criteria", "failures"]
    rows = []
    for fam in FAM_ORDER:
        for (label, b), r in sorted(A[fam]["bends"].items(), key=lambda t: (t[0][0] != "v1", t[0][1])):
            st = r["structure"]
            rows.append([fam, label, b, f_(r["np_pct_mac"], 4), f_(r["cm0"], 4), f_(r["cm0_per_deg"], 4), f_(r["res"]["trim"][0.2]["x_cg_span"] * 250, 4), f_(r["x_cg03_mm"], 4), f_(r["res"]["trim"][0.5]["x_cg_span"] * 250, 4),
                         f_(100 * r["sm03"], 4), f_(r["cn_beta"], 4), f_(r["cl_beta"], 4), f_(r["span_eff"], 4), f_(r["cdi"], 4), f_(r["tip_loading"], 4), f_(r["tail_lift_frac"], 4), f_(r["ballast_printed_g"], 4),
                         f_(r["ballast_frac_printed"], 4), f_(r["ballast_foam_g"], 4), f_(r["ballast_frac_foam"], 4), f_(r["mass_printed_g"], 4), f_(r["ws_printed"], 4), f_(st["sigma_neck_MPa"], 4), f_(st["neck_width_mm"], 4),
                         f_(st["neck_min_mm"], 4), f_(st["sigma_wrist_MPa"], 4), "yes" if not r["fails"] else "no", " | ".join(r["fails"])])
    write_csv(out / "bends.csv", hdr, rows)
    write_csv(out / "yaw.csv", ["family", "outline", "bend_deg", "option", "Cn_beta", "Cl_beta", "NP_pct_MAC", "CM0", "static_margin_CL0.3", "span_eff", f"Cn_beta>={CN_BETA_MIN}", f"Cl_beta<={CL_BETA_MAX}", "tag"],
              [[r["family"], r["outline"], r["bend"], r["option"], f_(r["cn_beta"]), f_(r["cl_beta"]), f_(r["np_pct_mac"], 4), f_(r["cm0"]), f_(r["sm03"]), f_(r["span_eff"]), "yes" if r["cn_ok"] else "no",
                "yes" if r["cl_ok"] else "no", "AVL, outline only, body ignored"] for fam in FAM_ORDER for r in A[fam]["yaw"]])
    hdr = ["family", "stage", "features", "bend_deg", "fin", "n_features", "Cn_beta", "Cl_beta", "static_margin_CL0.3_pct", "trim_CG_CL0.3_mm_250", "ballast_frac_250", "ballast_frac_400", "tip_loading", "v1_tip_loading_ref",
           "neck_ok", "appearance_cost_frac", "span_eff", "sheet_area_mm2_250", "passes", "failures"]
    rows = []
    for fam in FAM_ORDER:
        a = A[fam]
        for r in a["rows"]:
            rows.append([fam, "screening (linear in bend)", r["features"], r["bend"], "", r["n_features"], f_(r["cn_beta"]), f_(r["cl_beta"]), f_(100 * r["sm03"], 4), f_(r["x_cg03_mm"], 4), f_(r["ballast_frac_printed"]),
                         f_(r["ballast_frac_foam"]), f_(r["tip_loading"]), f_(a["base_tip"][r["bend"]]), r["neck_ok"], f_(r["appearance_cost_frac"]), f_(r["span_eff"]), f_(r["sheet_area_mm2"], 6), "yes" if not r["fails"] else "no", " | ".join(r["fails"])])
        for r in a["exact_log"] + a["fin_log"]:
            rows.append([fam, "exact re-run: " + r.get("stage", ""), r["features"], r["bend"], r.get("fin", ""), r["n_features"], f_(r["cn_beta"]), f_(r["cl_beta"]), f_(100 * r["sm03"], 4), f_(r["x_cg03_mm"], 4),
                         f_(r["ballast_frac_printed"]), f_(r["ballast_frac_foam"]), f_(r["tip_loading"]), "", r["neck_ok"], f_(r["appearance_cost_frac"]), f_(r["span_eff"]), f_(r["sheet_area_mm2"], 6),
                         "yes" if not eff_fails(r, a["waive"]) else "no", " | ".join(r["fails"])])
    write_csv(out / "selection.csv", hdr, rows)
    rows = []
    for fam in FAM_ORDER:
        for sk in ("printed", "foam"):
            pass
    for b in builds:
        st = b.get("structure")
        rows.append([b["fam"], b["size"], b["bend"], f_(b["m_sheet"], 4), f_(b["rail"]["mass_g"], 4), f_(b["cg0"], 4), f_(b["cg_target"], 4), f_(b["ballast_g"], 4), f_(b["station"], 4), f_(b["M"], 4), f_(b["ws"], 4),
                     f_(b["pull"][0.3], 4), f_(b["pull"][0.5], 4), f_(b["t_pct_mac"], 4), f_(b["rail_union"]["saddle_end"], 4), f_(b["hinge"], 4), "yes" if b["rail_fits_ahead_of_hinge"] else "NO",
                     f_(b["rail_union"]["tick_range"][0], 4), f_(b["rail_union"]["tick_range"][1], 4), "yes" if b["rail_covers_all_bends"] else "NO", f_(b["rail_nose_beyond_sheet_nose"], 4),
                     f_(st["sigma_neck_MPa"], 4) if st else "UNKNOWN (foam sandwich)", f_(st["sigma_wrist_MPa"], 4) if st else "UNKNOWN (foam sandwich)",
                     "ARITHMETIC (densities ASSUMED in flat_test.py; AVL trim CG)"])
    write_csv(out / "builds.csv", ["family", "size", "bend_deg", "sheet_mass_g", "rail_mass_g", "bare_CG_mm", "target_trim_CG_CL0.3_mm", "ballast_g", "ballast_station_mm", "all_up_g", "wing_loading_N_m2",
                                   "pullout_radius_m_CL0.3", "pullout_radius_m_CL0.5", "thickness_pct_MAC", "rail_saddle_end_mm", "hinge_mm", "rail_saddle_ends_ahead_of_hinge", "rail_tick_min_mm", "rail_tick_max_mm",
                                   "rail_ticks_cover_trim_CGs_all_bends", "rail_nose_beyond_sheet_nose_mm", "neck_stress_MPa_n5", "wrist_stress_MPa_n5", "tag"], rows)
    write_csv(out / "panelling_check.csv", ["family", "outline", "bend_deg", "chordwise", "spanwise", "NP_pct_MAC", "CM0", "Cn_beta", "Cl_beta", "span_eff", "tail_lift_over_weight", "wrist_moment_over_weight_m", "tag"],
              [[r["family"], r["outline"], r["bend"], r["nc"], r["ns"], f_(r["np_pct_mac"], 4), f_(r["cm0"], 4), f_(r["cn_beta"], 4), f_(r["cl_beta"], 4), f_(r["span_eff"], 4), f_(r["tail_lift_frac"], 4), f_(r["m_wrist_over_w_m"], 4),
                "AVL panelling convergence check"] for r in panel])
    rows = []
    for fam in FAM_ORDER:
        g = A[fam]["final"]["g"]
        for k_, (v, src, note) in bo.FAMILIES[fam]["params"].items():
            rows.append([fam, "v1 parameter", k_, v if not isinstance(v, tuple) else "; ".join(f"({x:g}, {y:g})" for x, y in v), src, note])
        for f in FEATURES:
            for k_, v in FEATURE_ON[f][fam].items():
                rows.append([fam, "v2 feature on-value", f"{f}: {k_}", v, "JUDGEMENT", f"off value = v1 value {bo.FAMILIES[fam]['params'].get(k_, (NEW_PARAM_OFF.get(k_), ''))[0]}"])
    write_csv(out / "dimensions.csv", ["family", "kind", "parameter", "value", "source_tag", "note"], rows)


def panelling_check(runner_dir: Path, fam, on, label, bend):
    out = []
    keep = (AVL_NCHORD, AVL_NSPAN)
    g = build_geometry2(feature_params(fam, on))
    try:
        for nc, ns in ((8, 64), (10, 80), (16, 128)):
            globals()["AVL_NCHORD"], globals()["AVL_NSPAN"] = nc, ns
            r = run_case2(g, RUN_SPAN_MM, bend, runner_dir / f"panel_{fam}_{label}_{nc}", f"p{nc}")
            out.append(dict(family=fam, outline=label, bend=bend, nc=nc, ns=ns, **{k: r[k] for k in ("np_pct_mac", "cm0", "cn_beta", "cl_beta", "span_eff", "tail_lift_frac", "m_wrist_over_w_m")}))
    finally:
        globals()["AVL_NCHORD"], globals()["AVL_NSPAN"] = keep
    return out


# =============================================================================================== feature table text and HTML page
def describe(eff, keys=None, verdicts=("helps", "hurts")):
    parts = []
    for key, label, unit, direction, src, rel, floor in QUANT:
        if keys and key not in keys:
            continue
        e = eff[key]
        if e["verdict"] in verdicts:
            parts.append(f"{e['verdict']} {label}: {e['base']:.4g} to {e['new']:.4g}")
    return parts


def feature_rows(A):
    rows = []
    for fam in FAM_ORDER:
        a = A[fam]
        final = a["final"]
        for f in FEATURES:
            eff = a["eff"][f]
            says = describe(eff)
            neutral = [QMAP[k][1] for k in QMAP if eff[k]["verdict"] == "neutral"]
            on = bool(final["on"].get(f))
            if on:
                text, kind = kept_because(eff, a["loo"][f])
            else:
                ben = modelled_benefits(eff)
                hurt = [k for k in list(BENEFIT_KEYS) + ["sheet_area_mm2", "sheet_mass_printed_g"] if eff[k]["verdict"] == "hurts"]
                if hurt:
                    text = "NOT in the final: " + "; ".join(f"{QMAP[k][1]} {eff[k]['base']:.4g} to {eff[k]['new']:.4g}" for k in hurt[:3]) + (" (helps " + ", ".join(QMAP[k][1] for k in ben) + ")" if ben else " and helps nothing the model measures")
                elif ben:
                    text = "NOT in the final: helps " + ", ".join(QMAP[k][1] for k in ben) + " but a combination without it ranked higher or the final failed no criterion without it"
                else:
                    text = "NOT in the final: no modelled benefit and no modelled cost (appearance only); left out because the final is not required to carry it"
                kind = "off"
            rows.append(dict(family=fam, feature=f, label=FEATURE_LABEL[f], on=on, says=says, neutral=neutral, text=text, kind=kind))
    return rows


def tag(s):
    return f' <span class="tag">[{s}]</span>'


def table(head, body, cls="t"):
    return "<div class='scroll'><table class='%s'><tr>%s</tr>%s</table></div>" % (cls, "".join(f"<th>{h}</th>" for h in head), "".join("<tr>" + "".join(f"<td>{c}</td>" for c in r) + "</tr>" for r in body))


def n_(v, d=3, sign=False):
    if v is None:
        return "&ndash;"
    if isinstance(v, float) and not math.isfinite(v):
        return "unreachable"
    return f"{v:+.{d}f}" if sign else f"{v:.{d}f}"


def page_html(out: Path, A, info, files, pdf_name, panel, quick):
    h = html.escape
    P = [f"<!doctype html><html lang='en'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'><title>Bird-style flat glider outlines, version 2</title>"
         f"<style>{ft.CSS}\n.scroll{{overflow-x:auto}}table.t td,table.t th{{padding:4px 7px;font-size:13.5px;vertical-align:top}}.warn{{background:#2c2418;border:2px solid #e89a55;padding:12px 16px}}"
         f"figure img{{max-width:100%}}.cols{{display:grid;grid-template-columns:repeat(auto-fit,minmax(380px,1fr));gap:12px}}</style></head><body><main>"]
    P.append("<h1>Bird-style flat glider outlines, version 2: smooth outlines, every feature backed by a modelled number</h1>")
    P.append("<p class='note'>Design numbers from an outline-only computer model (AVL) and from arithmetic. Nothing here is a flight result or a prediction that a glider glides or pulls out. Tags: GEOMETRY (exact from the outline), AVL (outline-only vortex-lattice model, inviscid, body ignored), ARITHMETIC (simple sum, inputs stated), JUDGEMENT (our choice), ASSUMED (a stated guess), CITED (a number from the bird notes). AVL says nothing about flat-plate stall, the low Reynolds number or whether a build pulls out."
             + (" <b>QUICK MODE: reduced search, do not use these results.</b>" if quick else "") + "</p>")
    # ---- 1. proposals
    P.append("<section><h2>1. The two proposed outlines</h2><p class='warn'><b>Proposals for Kai, not decisions.</b> They are the best outlines the search found under the criteria in section 3. Where a criterion could not be met, it is stated here and in the table below.</p><div class='cols'>")
    for fam in FAM_ORDER:
        for sk in ("printed", "foam"):
            nm = info[fam]["svgs"][sk]
            P.append(f"<figure><a href='{nm}'><img src='{nm}' alt='{fam} {sk}'></a><figcaption>{fam} {LABEL[fam]}, {SIZES[sk]['material']} {SIZES[sk]['span_mm']:g} mm. Click to enlarge.</figcaption></figure>")
    P.append("</div>")
    hdr = ["", "BS swift-like", "BC crow-like"]
    rows = []

    def both(label, fn, src=""):
        rows.append([label + (tag(src) if src else "")] + [fn(A[fam]) for fam in FAM_ORDER])
    both("Features switched on", lambda a: ", ".join(f for f in FEATURES if a["final"]["on"].get(f)) or "none (the v1 outline)")
    both("Tail bend (trailing edge up), deg", lambda a: f"{a['final']['bend']:g} (also passing: {', '.join(format(b, 'g') for b in passing_bends(a) if b != a['final']['bend']) or 'none'})", "JUDGEMENT")
    both("Fin", lambda a: "none" if not a["final"].get("fin") else f"ventral fin {100 * a['final']['fin']:g}% of wing area")
    both("Criteria not met", lambda a: ("; ".join(a["final"]["fails"]) or "none") if a["flagged"] or a["waive"] else "none")
    both("Span 250 / 400 mm: wing area, mm&sup2;", lambda a: f"{a['final']['g']['S'] * 250 ** 2:,.0f} / {a['final']['g']['S'] * 400 ** 2:,.0f}", "GEOMETRY")
    both("Aspect ratio b&sup2;/S", lambda a: f"{a['final']['g']['ar']:.2f}", "GEOMETRY")
    both("MAC, mm (250 / 400)", lambda a: f"{a['final']['g']['mac'] * 250:.1f} / {a['final']['g']['mac'] * 400:.1f}", "GEOMETRY")
    both("Root chord, wrist chord, mm (250)", lambda a: f"{a['final']['g']['c_root'] * 250:.1f}, {a['final']['g']['c_wrist'] * 250:.1f}", "GEOMETRY")
    both("Hinge, mm aft of root LE (250 / 400)", lambda a: f"{a['final']['g']['x_h'] * 250:.1f} / {a['final']['g']['x_h'] * 400:.1f}", "GEOMETRY")
    both("Tail neck width / spread / length, mm (250)", lambda a: f"{a['final']['g']['wt0'] * 250:.1f} / {a['final']['g']['St'] * 250:.1f} / {a['final']['g']['Lt'] * 250:.1f}", "GEOMETRY")
    both("Nose ahead of root LE, mm (250)", lambda a: f"{-a['final']['g']['nose_x'] * 250:.1f}", "GEOMETRY")
    both("Neutral point, % MAC", lambda a: f"{a['final']['np_pct_mac']:.1f}", "AVL")
    both("Static margin at CL 0.3, % MAC", lambda a: f"{100 * a['final']['sm03']:.1f}", "AVL")
    both("Trim CG at CL 0.3, mm aft of root LE (250 / 400)", lambda a: f"{a['final']['x_cg03_span'] * 250:.1f} / {a['final']['x_cg03_span'] * 400:.1f}", "AVL")
    both("Cn_beta, Cl_beta /rad (no fin unless stated)", lambda a: f"{a['final']['cn_beta']:+.4f}, {a['final']['cl_beta']:+.4f}", "AVL")
    both("Span efficiency e, tip loading indicator", lambda a: f"{a['final']['span_eff']:.2f}, {a['final']['tip_loading']:.2f} (v1: {a['bends'][('v1', a['final']['bend'])]['tip_loading']:.2f})", "AVL")
    both("All-up mass g, ballast g: printed 250", lambda a: f"{a['final']['sizes']['printed']['M']:.1f}, {a['final']['sizes']['printed']['ballast_g']:.1f} ({100 * a['final']['ballast_frac_printed']:.0f}%)", "ARITHMETIC")
    both("All-up mass g, ballast g: foamboard 400", lambda a: f"{a['final']['sizes']['foam']['M']:.1f}, {a['final']['sizes']['foam']['ballast_g']:.1f} ({100 * a['final']['ballast_frac_foam']:.0f}%)", "ARITHMETIC")
    both("Wing loading N/m&sup2; (250 / 400)", lambda a: f"{a['final']['ws_printed']:.1f} / {a['final']['ws_foam']:.1f}", "ARITHMETIC")
    P.append(table(hdr, rows))
    P.append("</section>")
    # ---- 2. feature table
    P.append("<section><h2>2. Feature / what the model says it does / kept because</h2><p>Each feature is switched on alone (strength 1) against the v1 outline at a " + f"{REF_BEND:g} deg tail bend. &quot;Helps&quot; and &quot;hurts&quot; mean the quantity moves by more than the neutrality threshold (3% to 5% for AVL numbers, 1% for sheet arithmetic; AVL changes smaller than the panelling noise in section 7 are called neutral). The last column is judged in the context of the final outline (switch the feature off alone and compare). Full numbers: <a href='features_single.csv'>features_single.csv</a>, <a href='features_strength.csv'>features_strength.csv</a>.</p>")
    body = []
    for r in feature_rows(A):
        body.append([r["family"], f"<b>{r['feature']}</b><br>{h(r['label'])}", ("<br>".join(h(x) for x in r["says"]) or "changes nothing the model measures") + (f"<br><span class='muted'>neutral: {len(r['neutral'])} quantities</span>" if r["neutral"] else ""),
                     ("<b>in the final.</b> " if r["on"] else "") + h(r["text"])])
    P.append(table(["family", "feature", "what the model says (feature alone, bend 5)", "kept because"], body))
    P.append("</section>")
    # ---- 3. selection
    P.append("<section><h2>3. How the final outlines were chosen (criteria, not looks)</h2><ol>"
             f"<li>Cn_beta at least {CN_BETA_MIN} per rad with no fin; if no outline reaches it, the smallest ventral fin (3% then 5% of wing area) that does.{tag('JUDGEMENT')}</li>"
             f"<li>Cl_beta at most {CL_BETA_MAX} per rad (the stable sign, with a floor).{tag('JUDGEMENT')}</li>"
             f"<li>Static margin at CL 0.3 between {100 * SM_WINDOW[0]:g} and {100 * SM_WINDOW[1]:g}% of MAC with the chosen bend.{tag('JUDGEMENT')}</li>"
             f"<li>Trim CG at CL 0.3 at least {NOSE_CLEAR_SPAN * 250:.0f} mm (250 mm span; same fraction at 400) behind the nose tip, and reachable with the new rail's ballast holes.{tag('JUDGEMENT')}</li>"
             f"<li>Nose ballast below {100 * BALLAST_MAX_FRAC:g}% of the all-up mass at both sizes.{tag('JUDGEMENT')}</li>"
             "<li>Tip loading indicator not above the v1 value at the same bend (1% tolerance). The indicator is the area-weighted local lift coefficient of the outer 20% of the semi-span divided by the wing CL; it is NOT a stall prediction.</li>"
             f"<li>Tail neck at least the larger of the width that keeps the hinge-strip stress below {SIGMA_ALLOW_MPA:g}/{SAFETY_FACTOR:g} MPa at load factor {N_LOAD:g} and {100 * NECK_MIN_FRAC_SPAN:g}% of the span (beam arithmetic, section 6).{tag('ARITHMETIC')}</li>"
             f"<li>Features with no modelled benefit (appearance only) may add at most {100 * APPEARANCE_COST_MAX:g}% to the all-up mass.{tag('JUDGEMENT')}</li></ol>"
             "<p>Ranking among outlines that pass: fewest fins, then the most outline features switched on (Kai asked for a bird-like outline: a JUDGEMENT tie-break that comes only after every criterion is met), then the largest smallest-margin to the thresholds, then the smallest sheet area. "
             "Screening used AVL at bends 0 and 5 and the model's linearity in tail incidence to reach bends 2, 3, 7.5 and 10; the best screened outlines were re-run exactly at their own bend and re-checked.</p>")
    for fam in FAM_ORDER:
        a = A[fam]
        from collections import Counter
        c = Counter(x.split(" ")[0] if not x.startswith("trim") else "trim CG" for r in a["rows"] for x in r["fails"])
        n_rows = len(a["rows"])
        P.append(f"<h3>{fam} {LABEL[fam]}: {n_rows} candidates (feature combinations x {len(SELECTION_BENDS)} bends)</h3><p>{len(a['ranked'])} passed screening with nothing waived. Failures counted per criterion (a candidate can fail several): "
                 + ", ".join(f"{k}: {v}" for k, v in c.most_common()) + ".</p>")
        if a["waive"]:
            P.append(f"<p class='warn'><b>No candidate met every criterion.</b> The {', '.join(a['waive'])} limit was waived to find the closest outline; the final fails it as shown in section 1. Reason, in the model: the bent fan tail and the wide planform put the sheet's own CG far behind the CG the static margin needs, so the difference must be ballast. "
                     f"For the final at 250 mm the bare CG is {a['final']['sizes']['printed']['cg0']:.1f} mm and the trim CG {a['final']['sizes']['printed']['cg_target']:.1f} mm aft of the root LE; to stay under {100 * BALLAST_MAX_FRAC:g}% ballast at the rail's ballast hole ({a['final']['sizes']['printed']['station']:g} mm) the bare CG would have to be at or ahead of {a['final']['sizes']['printed']['cg_target'] + BALLAST_MAX_FRAC / (1 - BALLAST_MAX_FRAC) * (a['final']['sizes']['printed']['cg_target'] - a['final']['sizes']['printed']['station']):.1f} mm{tag('ARITHMETIC')}.</p>")
        near = sorted(a["rows"], key=lambda r: (len(r["fails"]), -r["margin"]))[:6]
        v1rows = [a["bends"][("v1", b)] for b in BENDS if b > 0]
        P.append("<p>Closest failing candidates and the v1 outline at each bend (exact AVL), with the reasons:</p>")
        body = [[r["features"], f"{r['bend']:g}", "screening", h(" | ".join(r["fails"]) or "passes")] for r in near if r["fails"]]
        body += [["none (v1 outline)", f"{r['bend']:g}", "exact", h(" | ".join(r["fails"]) or "passes")] for r in v1rows]
        P.append(table(["features on", "bend", "stage", "criteria failed"], body))
    P.append("<p>Every candidate with its verdict: <a href='selection.csv'>selection.csv</a>.</p></section>")
    # ---- 4. interactions
    P.append("<section><h2>4. Interactions of the features that matter most</h2><p>Ranked by the size of their combined change in Cn_beta, Cl_beta, span efficiency, tip loading, static margin and neutral point. Combined change minus the sum of the single changes is the interaction (AVL, bend 5).</p>")
    for fam in FAM_ORDER:
        a = A[fam]
        P.append(f"<h3>{fam}: top features {', '.join(a['top'])}</h3>")
        body = []
        for r in a["interactions"]:
            if r["quantity"] in ("cn_beta", "cl_beta", "sm03", "span_eff", "np_pct_mac"):
                body.append([r["combo"], r["quantity"], f"{r['base']:.4g}", f"{r['sum_of_singles']:+.4g}", f"{r['combined']:+.4g}", f"{r['interaction']:+.4g}"])
        P.append(table(["features", "quantity", "v1", "sum of single changes", "combined change", "interaction"], body))
    P.append("<p>All quantities: <a href='interactions.csv'>interactions.csv</a>.</p></section>")
    # ---- 5. bends and yaw
    P.append("<section><h2>5. The final outlines at every tail bend, and the yaw options</h2><p>Bends 0, 2, 3, 5, 7.5, 10 deg, trailing edge up. Exact AVL at every bend" + tag("AVL") + ". Bend 0 has no nose-up moment at zero lift.</p>")
    for fam in FAM_ORDER:
        a = A[fam]
        body = []
        for lab in ("v1", "final"):
            for b in BENDS:
                r = a["bends"][(lab, b)]
                body.append([lab, f"{b:g}", n_(r["np_pct_mac"], 1), n_(r["cm0"], 4, True), n_(r["x_cg03_mm"], 1), n_(100 * r["sm03"], 1), n_(r["cn_beta"], 4, True), n_(r["cl_beta"], 4, True), n_(r["span_eff"], 2), n_(r["tip_loading"], 2),
                             n_(r["ballast_frac_printed"] * 100, 0) + "% / " + n_(r["ballast_frac_foam"] * 100, 0) + "%", "yes" if not eff_fails(r, a["waive"]) else "no"])
        P.append(f"<h3>{fam} {LABEL[fam]}</h3>" + table(["outline", "bend", "NP % MAC", "CM0", "trim CG CL 0.3, mm (250)", "SM CL 0.3, %", "Cn_beta", "Cl_beta", "span eff.", "tip loading", "ballast 250 / 400", "passes"], body))
        P.append(f"<h3>{fam}: yaw options (no fin / ventral fin / tail side edges bent up as small fins)</h3>")
        body = [[r["outline"], f"{r['bend']:g}", r["option"], n_(r["cn_beta"], 4, True), n_(r["cl_beta"], 4, True), n_(r["np_pct_mac"], 1), n_(r["cm0"], 4, True), "yes" if r["cn_ok"] else "no", "yes" if r["cl_ok"] else "no"] for r in a["yaw"]]
        P.append(table(["outline", "bend", "option", "Cn_beta", "Cl_beta", "NP % MAC", "CM0", f"Cn_beta >= {CN_BETA_MIN}", f"Cl_beta <= {CL_BETA_MAX}"], body))
    for fam in FAM_ORDER:
        a_ = A[fam]
        fb = a_["final"]["bend"]
        alt = [r for r in a_["yaw"] if r["outline"] == "final" and r["bend"] == fb and r["option"].startswith("tail edges") and r["cn_ok"]]
        if alt:
            P.append(f"<p><b>{fam}: a fin-free alternative.</b> Bending the outer {100 * EDGE_FIN_FRAC:g}% of each side of the fan up (no extra part, no added area) reaches Cn_beta of " + ", ".join(f"{r['cn_beta']:+.4f} at {r['option'].replace('tail edges up ', '')}" for r in alt)
                     + f" at the final bend of {fb:g} deg, with Cl_beta " + ", ".join(f"{r['cl_beta']:+.4f}" for r in alt) + ". The selection rule only walked the ventral-fin ladder (3% then 5% of wing area), so this is listed, not chosen: whether to fold the edges is for Kai" + tag("AVL") + ".</p>")
    P.append(f"<p>Tail-edge fins are modelled as dihedral panels: the outer {100 * EDGE_FIN_FRAC:g}% of each half of the fan, bent up about a chordwise line. Ventral fin chord is 0.6 tail lengths under the tail. Full table: <a href='yaw.csv'>yaw.csv</a>, <a href='bends.csv'>bends.csv</a>.</p></section>")
    # ---- 6. builds
    P.append("<section><h2>6. Builds, masses, rails and structure</h2>")
    body = []
    for fam in FAM_ORDER:
        for b in info[fam]["builds"]:
            st = b.get("structure")
            body.append([f"{fam} {b['size']} {b['span']:g}", f"{b['bend']:g}", n_(b["m_sheet"], 2), n_(b["rail"]["mass_g"], 2), n_(b["cg0"], 1), n_(b["cg_target"], 1), n_(b["ballast_g"], 2) + (f" at {b['station']:g} mm" if b["station"] is not None else ""),
                         n_(b["M"], 2), n_(b["ws"], 1), f"{b['pull'][0.3]:.1f} / {b['pull'][0.5]:.1f}", f"{b['rail_union']['saddle_end']:.1f} vs hinge {b['hinge']:.1f}", f"{b['rail_union']['tick_range'][0]:.0f} to {b['rail_union']['tick_range'][1]:.0f}",
                         "yes" if b["rail_covers_all_bends"] else "NO", n_(b["rail_nose_beyond_sheet_nose"], 1),
                         (f"{st['sigma_neck_MPa']:.1f} (min neck {st['neck_min_mm']:.1f} mm, have {st['neck_width_mm']:.1f}); wrist {st['sigma_wrist_MPa']:.1f}; hinge strain {st['hinge_strain_pct']:.1f}% (limit {PLA_STRAIN_ALLOW_PCT:g}% ASSUMED)" if st else "UNKNOWN")])
    P.append(table(["build", "bend", "sheet g", "rail g", "bare CG mm", "trim CG mm", "ballast g", "all-up g", "W/S N/m&sup2;", "pull-out R m (CL 0.3 / 0.5)", "rail saddle end, mm", "rail ticks, mm", "ticks cover all trim CGs?", "rail nose beyond sheet nose, mm", "stress MPa at n=5 and hinge strain % (printed only)"], body))
    P.append("<p>Sheet mass: printed = mesh volume x PLA density, foamboard = area x areal mass (both ASSUMED in flat_test.py). New rails are made for each outline: the saddle ends " + f"{RAIL_HINGE_GAP_MM:g} mm ahead of the hinge so it cannot stiffen the bend, the ticks run {RAIL_TICK_MARGIN_MM:g} mm beyond the lowest and highest trim CG of all proposed bends and CL 0.2 to 0.5, and the hole, notch and tick conventions are those of the flat-test rails (<code>flat_test.stick_mesh</code>). Ballast goes in the rail hole nearest 75% of the nose length, or the most forward hole that is ahead of the target. Pull-out radius R = 2 (W/S) / (rho g CL), steady circular pull-up, load factor much greater than 1, air density 1.225 ASSUMED; it says nothing about whether the glider pulls out" + tag("ARITHMETIC") + ".</p>")
    P.append("<h3>Beam arithmetic for the printed sheet</h3><p>Loads at load factor " + f"{N_LOAD:g}" + tag("JUDGEMENT") + f". <b>Tail neck</b>: the {HINGE_V2_MM:g} mm hinge strip (two layers; v1 used {bo.HINGE_THICK_MM:g} mm, which the beam arithmetic rejects for the crow-like tail at load factor {N_LOAD:g}) across the neck width is a cantilever carrying the tail force that balances CM0 at zero wing lift (|CM0| x MAC / (0.3 x lever) x weight, lever = tail centroid aft of the CM reference point; the AVL tail-lift fraction is not used because it moves by tens of per cent with the panelling, see section 7) at the tail centroid, sigma = 6 M / (w t&sup2;). <b>Wrist</b>: the outer panel is a cantilever about the wrist, with moment from the AVL strip loads, over the local chord at the wrist and the {SIZES['printed']['thickness_mm']:g} mm plate, sigma = 6 M / (c t&sup2;). Allowable {SIGMA_ALLOW_MPA:g} MPa across print layers" + tag("ASSUMED") + f" with a safety factor {SAFETY_FACTOR:g}. The tail neck minimum is the larger of the stress-based width and {100 * NECK_MIN_FRAC_SPAN:g}% of the span. Foamboard stiffness: UNKNOWN (sandwich), not computed. This is a screening number, not a strength test; the real hinge may be worse (layer adhesion, a crease that has been bent more than once).</p>")
    for fam in FAM_ORDER:
        t = info[fam]["thin"]
        P.append(f"<p><b>Thinner sheet for the 400 mm foamboard, {fam}:</b> the 5 mm board is {t['t_pct_mac_now']:.1f}% of the MAC. A sheet of about {t['t_mm']:.1f} mm would be {THIN_SHEET_MAC_PCT:g}% of the MAC (JUDGEMENT target). If the areal mass scaled with thickness (ASSUMED, not true for a board with paper skins), the sheet would weigh {t['sheet_g']:.1f} g and the all-up mass would be {t['mass_g']:.1f} g with {t['ballast_g']:.1f} g of ballast at bend {t['bend']:g}.</p>")
    P.append("</section>")
    # ---- 7. provenance
    P.append("<section><h2>7. What came from the bird notes, what did not, and how reliable the AVL numbers are</h2><div class='box'><h3>From the bird notes (CITED)</h3><ul>"
             "<li>BS aspect ratio 9.5 (common swift b&sup2;/S, paper value 9.8) [R2]. BS tail spread 0.23 of the span (0.09 m tail span over 0.39 m wing span at 7 m/s; the bird furls it to 0.04 m at 11 m/s) [R2].</li>"
             "<li>BC aspect ratio 5.0 (magpie 4.97, jackdaw about 6) [R9, R2].</li></ul>"
             "<h3>No planform data in the notes (all JUDGEMENT)</h3><p>The notes give no arm/hand split, no leading-edge angles, no chord distribution, no trailing-edge shape, no body or neck width, no tail length and no root blend. The M-shaped leading edge, the curved trailing edge, the tip rounding, the wrist blend radius, the neck width and the root blend are therefore our own parameters, chosen only to be switched on and off and costed. <b>Nothing is traced from a bird image.</b> The notes also say birds were not shown to be passively stable, that the tail is not shown to stabilise a bird, and that sweep costs lift-to-drag ratio on simple plates; none of that is relied on here.</p></div>"
             "<h3>AVL panelling check</h3><p>The tail sits in the wing's own wake plane, so AVL results depend a little on the panelling. The same outline at three panel counts (this run uses 10 chordwise x 80 spanwise per half wing):</p>")
    P.append(table(["family", "outline", "bend", "chordwise x spanwise", "NP % MAC", "CM0", "Cn_beta", "Cl_beta", "span eff.", "tail lift / weight", "wrist moment / weight, m"],
                   [[r["family"], r["outline"], f"{r['bend']:g}", f"{r['nc']} x {r['ns']}", n_(r["np_pct_mac"], 1), n_(r["cm0"], 4, True), n_(r["cn_beta"], 4, True), n_(r["cl_beta"], 4, True), n_(r["span_eff"], 2), n_(r["tail_lift_frac"], 3, True), n_(r["m_wrist_over_w_m"], 4)] for r in panel]))
    P.append("<p>Cn_beta, Cl_beta and CM0 agree within a few per cent across the panellings; the neutral point, span efficiency and the AVL tail-lift fraction do not (the tail lies in the wing's own wake plane, so its load depends on how the lattices line up). Read span efficiency and the tip loading indicator as trends only; the neutrality thresholds for them in section 2 are set above the spread seen here. The v2 deck is finer than the v1 deck, so v1 numbers quoted here are re-runs in the v2 deck; they differ from the v1 page by a few per cent.</p></section>")
    # ---- 8. files
    P.append("<section><h2>8. Files</h2><ul>" + "".join(f"<li>{x}</li>" for x in files) + "</ul>"
             + (f"<p>Print templates, 1:1 US Letter, white, all builds: <a href='{pdf_name}'>{pdf_name}</a> (made with <code>flat_templates_pdf.py</code>). Every page carries a 100 mm scale bar at the top right; check it with a ruler before cutting.</p>" if pdf_name else "")
             + "<p>Tables: <a href='features_single.csv'>features_single.csv</a>, <a href='features_strength.csv'>features_strength.csv</a>, <a href='interactions.csv'>interactions.csv</a>, <a href='bends.csv'>bends.csv</a>, <a href='yaw.csv'>yaw.csv</a>, <a href='selection.csv'>selection.csv</a>, <a href='builds.csv'>builds.csv</a>, <a href='panelling_check.csv'>panelling_check.csv</a>, <a href='dimensions.csv'>dimensions.csv</a>. Re-run: <code>py -B rigid_glider/bird_outline_v2.py</code>.</p></section>")
    P.append("<section><h2>9. What is not known</h2><ul><li>Whether a flat-plate sheet at these Reynolds numbers (about 12,000 to 31,000) pulls out of a dive or glides. No glide ratio and no CG window is predicted anywhere on this page.</li>"
             "<li>Whether the bent tail still works as the dive speed builds up. Whether a printed hinge strip holds; the stress numbers are screening arithmetic.</li>"
             "<li>Whether the tip loading indicator means anything for a flat plate.</li><li>The body strip is ignored in the AVL model; the nose and the root blend therefore change only the sheet area, the mass and the ballast, not the aerodynamics.</li></ul></section>")
    P.append("</main></body></html>")
    (out / "index.html").write_text("\n".join(P), encoding="utf-8")


# =============================================================================================== main
def run(out: Path = OUT_DEFAULT, avl=True, pdf=True, quick=False, log=print):
    out.mkdir(parents=True, exist_ok=True)
    runner = Runner(out / "avl_work", avl)
    A = analyse(runner, quick, log)
    final_tables(A, runner, log)
    panel = []
    for fam in FAM_ORDER:
        b = A[fam]["final"]["bend"]
        if avl:
            panel += panelling_check(out / "avl_work", fam, {}, "v1", b) + panelling_check(out / "avl_work", fam, A[fam]["final"]["on"], "final", b)
    info, files, builds, pages = {}, [], [], []
    for fam in FAM_ORDER:
        info[fam] = build_final(fam, A[fam], out)
        files += info[fam]["files"]
        builds += info[fam]["builds"]
        pages += info[fam]["template_pages"]
    pdf_name = None
    if pdf and pages:
        pdf_name = "bird_v2_templates.pdf"
        subprocess.run([sys.executable, "-B", str(ROOT / "rigid_glider/flat_templates_pdf.py"), str(out / pdf_name)] + [str(p) for p in pages], check=True)
    write_tables(out, A, builds, panel)
    page_html(out, A, info, files, pdf_name, panel, quick)
    runner.save()
    return dict(A=A, info=info, builds=builds, pdf=pdf_name, panel=panel, runner=runner)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=OUT_DEFAULT)
    ap.add_argument("--no-pdf", action="store_true")
    ap.add_argument("--quick", action="store_true", help="reduced search for a smoke test")
    a = ap.parse_args()
    R = run(a.out, True, not a.no_pdf, a.quick)
    for fam in FAM_ORDER:
        f = R["A"][fam]["final"]
        print(fam, "final:", f["features"], "bend", f["bend"], "fin", f.get("fin"), "waived", f.get("waived"), "fails", f["fails"])
    print("PDF:", R["pdf"], "->", a.out)


if __name__ == "__main__":
    main()
