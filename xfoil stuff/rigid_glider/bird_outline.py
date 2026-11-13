"""First bird-style flat glider outlines: a one-piece flat sheet seen from above as a gliding bird (arm panel + swept hand panel + body blended
into a short fan tail), with the tail panel bent trailing-edge-UP along ONE straight hinge line so the sheet has a nose-up moment at zero lift
(it must pull out of a nose-down release by itself).  One sheet, one bend.

Honesty rules (as in the flat-test kit): every number is tagged GEOMETRY / AVL / ARITHMETIC / JUDGEMENT / ASSUMED / CITED.  Bird proportions are used
ONLY where research/bird_research_notes.md gives a cited number; everything else is JUDGEMENT.  The notes say gliding birds are NOT passively stable and
that the bird tail is not shown to be what stabilises a bird: nothing here claims either.  AVL is inviscid, outline-only, body ignored: it says nothing
about flat-plate stall, low Reynolds number, or whether a build actually pulls out.  No flat-plate glide-ratio or CG-window prediction is made.

Writes ONLY to --out (default results/runs/rigid_glider/bird_outline/).  Imports (never edits) flat_test.py for constants, mesh and template helpers.
AVL runs are serial.     py -B rigid_glider/bird_outline.py [--out DIR] [--no-avl] [--no-pdf]
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

import flat_test as ft
from avl import run_avl
from geometry import Wing

ROOT = ft.ROOT
AVL_EXE = ft.AVL_EXE
OUT_DEFAULT = ROOT / "results/runs/rigid_glider/bird_outline"
SOURCES = ft.SOURCES + ("CITED",)

# =============================================================================================== PARAMETERS
SIZES = {   # printed 250 mm (PLA plate) and foamboard 400 mm (thickness and areal mass ASSUMED in flat_test.py); rail = the existing CG rail of the same size
    "printed": dict(span_mm=250.0, thickness_mm=ft.THICKNESS_MM, material="PLA plate", rail_span=250.0, rail_name="F45_span250 (104 mm)"),
    "foam": dict(span_mm=400.0, thickness_mm=ft.FOAM_THICKNESS_MM, material="foamboard", rail_span=400.0, rail_name="B45_span400 (166 mm)"),
}
RAIL_AR = 4.5                        # the existing rails were generated for the AR 4.5 wings (flat_test.wing_geometry(span, 4.5))
TAIL_BENDS = (0.0, 5.0, 10.0, 15.0)  # deg, trailing edge UP (tail set nose-down relative to the wing); the brief's list
TRIM_CLS = (0.2, 0.3, 0.4, 0.5)
REF_BEND = 10.0                      # tail bend used for the outline scan and the yaw-fix evaluation (JUDGEMENT)
AVL_ALPHAS = (0.0, 4.0, 8.0)         # deg; AVL is linear in alpha for this planar model
CN_BETA_NEAR_ZERO = 0.003            # per rad: |Cn_beta| below this is called "near zero" (JUDGEMENT)
SCAN_LEVELS = (0.8, 1.0, 1.25)       # multipliers on the family's tail length and tail spread
SCAN_SWEEP_DELTAS = (-10.0, 0.0, 10.0)   # deg added to the family's hand sweep
VFIN_AREA_FRAC = 0.03                # ventral fin area / wing area (JUDGEMENT); chord 0.6 tail lengths, under the tail
TIP_BEND_ETA = 0.70                  # tip bend line at this fraction of the hand semi-span (chordwise line), JUDGEMENT
TIP_BEND_ANGLES = (-60.0, -30.0, 30.0, 60.0)   # deg; negative = tip bent DOWN, positive = UP
HINGE_THICK_MM = 0.2                 # printed hinge strip (one layer), JUDGEMENT
HINGE_WIDTH_MM = 1.0                 # printed hinge gap between wing and tail, JUDGEMENT
HINGE_OVERLAP_MM = 0.4               # the strip overlaps wing and tail shells by this much (a slicer joins them)
SM_BAND = (0.05, 0.15)               # static margin at CL 0.3 used to PROPOSE builds (JUDGEMENT): not a requirement
N_BUILD_BENDS = 2
PULLOUT_CLS = (0.3, 0.5)
G_ACCEL = 9.80665
OVERLAP_MM = 15.0                    # tile overlap on multi-page templates
BODY_WORD = "ignored in the AVL model"

# Family proportions.  (value, source, note)
FAMILIES = {
    "BS": dict(label="swift-like", params=dict(
        ar_target=(9.5, "CITED", "common swift b^2/S = 9.5 (paper value 9.8) [R2, bird notes section 1.1]"),
        body_halfwidth_frac=(0.030, "JUDGEMENT", "body strip half-width / span; not in the notes"),
        nose_frac=(0.05, "JUDGEMENT", "nose ahead of the wing root leading edge / span"),
        wrist_frac=(0.40, "JUDGEMENT", "arm/hand split as a fraction of the semi-span; no cited number"),
        arm_sweep_deg=(5.0, "JUDGEMENT", "leading-edge sweep of the arm panel"),
        hand_sweep_deg=(50.0, "JUDGEMENT", "leading-edge sweep of the hand panel; context only: a 60 deg swift hand-wing model in a water tunnel [R21, secondary]"),
        cw_over_croot=(0.85, "JUDGEMENT", "chord at the wrist / chord at the centreline"),
        hand_stations=(((0.0, 1.0), (0.40, 0.78), (0.75, 0.50), (0.93, 0.25), (1.0, 0.06)), "JUDGEMENT", "(fraction of hand semi-span, chord / wrist chord): scythe tip"),
        tail_len_frac=(0.12, "JUDGEMENT", "tail length behind the wing trailing edge / span"),
        tail_spread_frac=(0.23, "CITED", "swift tail span 0.09 m at 7 m/s over wing span 0.39 m [R2]: ARITHMETIC 0.09/0.39 = 0.23 (the bird furls it to 0.04 m at 11 m/s)"),
        tail_root_frac=(0.05, "JUDGEMENT", "tail root (hinge) width / span"),
        tail_bulge_frac=(0.15, "JUDGEMENT", "rounding of the tail end, fraction of the tail length"),
    )),
    "BC": dict(label="crow/pigeon-like", params=dict(
        ar_target=(5.0, "CITED", "magpie wing aspect ratio 4.97, jackdaw 6 [R9, R2 via bird notes table 1.1]"),
        body_halfwidth_frac=(0.040, "JUDGEMENT", "body strip half-width / span; not in the notes"),
        nose_frac=(0.07, "JUDGEMENT", "nose ahead of the wing root leading edge / span"),
        wrist_frac=(0.50, "JUDGEMENT", "arm/hand split as a fraction of the semi-span; no cited number"),
        arm_sweep_deg=(3.0, "JUDGEMENT", "leading-edge sweep of the arm panel"),
        hand_sweep_deg=(20.0, "JUDGEMENT", "leading-edge sweep of the hand panel; the notes say sweep costs L/D on plates and helps yaw only through the OUTER wing [R17, R39]"),
        cw_over_croot=(0.95, "JUDGEMENT", "chord at the wrist / chord at the centreline"),
        hand_stations=(((0.0, 1.0), (0.50, 0.90), (0.80, 0.72), (0.95, 0.50), (1.0, 0.25)), "JUDGEMENT", "(fraction of hand semi-span, chord / wrist chord): rounded tip"),
        tail_len_frac=(0.20, "JUDGEMENT", "tail length behind the wing trailing edge / span"),
        tail_spread_frac=(0.38, "JUDGEMENT", "fan tail spread / span; no cited number for a corvid or pigeon"),
        tail_root_frac=(0.08, "JUDGEMENT", "tail root (hinge) width / span"),
        tail_bulge_frac=(0.10, "JUDGEMENT", "rounding of the tail end, fraction of the tail length"),
    )),
}
FAM_ORDER = ("BS", "BC")


def T(value, source, unit="", note=""):
    assert source in SOURCES, source
    return {"value": value, "unit": unit, "source": source, "note": note}


def pv(fam, name):
    return FAMILIES[fam]["params"][name][0]


# =============================================================================================== geometry (dimensionless: span b = 1; x aft from the wing root leading edge)
def _chord_integrals(ys, les, tes):
    """Exact integrals over a piecewise-linear chord: (area of the half wing, integral c^2 dy, integral x_le*c dy, integral y*c dy)."""
    a = c2 = xc = yc = 0.0
    for i in range(len(ys) - 1):
        dy = ys[i + 1] - ys[i]
        c0, c1 = tes[i] - les[i], tes[i + 1] - les[i + 1]
        a += dy * (c0 + c1) / 2
        c2 += dy * (c0 * c0 + c0 * c1 + c1 * c1) / 3
        # linear x_le and c: integral of x_le*c dy by Simpson (exact for the quadratic integrand)
        xm, cm = (les[i] + les[i + 1]) / 2, (c0 + c1) / 2
        xc += dy / 6 * (les[i] * c0 + 4 * xm * cm + les[i + 1] * c1)
        ym = (ys[i] + ys[i + 1]) / 2
        yc += dy / 6 * (ys[i] * c0 + 4 * ym * cm + ys[i + 1] * c1)
    return a, c2, xc, yc


def build_geometry(p: dict, bend_deg=0.0, b=1.0):
    """p: parameter values (name -> value).  Returns the half outline, full outline, wing/tail polygons and the reference quantities, in units of the span."""
    wb, xn = p["body_halfwidth_frac"] * b, -p["nose_frac"] * b
    yw = p["wrist_frac"] * b / 2
    tan_a, tan_h = math.tan(math.radians(p["arm_sweep_deg"])), math.tan(math.radians(p["hand_sweep_deg"]))
    Lt, St, wt0 = p["tail_len_frac"] * b, p["tail_spread_frac"] * b, p["tail_root_frac"] * b
    st = p["hand_stations"]
    half = b / 2

    def nodes(c_root):
        xh = c_root
        cw = p["cw_over_croot"] * c_root
        xw = yw * tan_a
        hand = []
        for eta, frac in st:
            y = yw + eta * (half - yw)
            xle = xw + (y - yw) * tan_h
            hand.append((y, xle, frac * cw))
        return xh, cw, xw, hand

    def wing_area(c_root):
        xh, cw, xw, hand = nodes(c_root)
        ys, les, tes = wing_profile(p, c_root, xh, cw, xw, hand, wt0)
        return 2 * _chord_integrals(ys, les, tes)[0]

    target = b * b / p["ar_target"]
    lo, hi = 0.01 * b, 2.0 * b
    for _ in range(80):
        mid = (lo + hi) / 2
        if wing_area(mid) < target:
            lo = mid
        else:
            hi = mid
    c_root = (lo + hi) / 2
    xh, cw, xw, hand = nodes(c_root)
    ys, les, tes = wing_profile(p, c_root, xh, cw, xw, hand, wt0)
    a_half, c2, xcint, ycint = _chord_integrals(ys, les, tes)
    S = 2 * a_half
    mac = c2 / a_half
    x_le_mac = xcint / a_half
    # ---- outline, right half, nose -> ... -> tail end (centreline points first and last)
    bulge = p["tail_bulge_frac"] * Lt
    ex = (xh + Lt + 1.5 * bulge)
    R = [(xn, 0.0), (0.5 * xn, 0.75 * wb), (wb * tan_a, wb), (xw, yw)]
    R += [(x, y) for (y, x, c) in hand[1:]]                                      # LE out to the tip (hand[0] is the wrist itself)
    tip_te = [(x + c, y) for (y, x, c) in hand]
    R += list(reversed(tip_te))                                                  # TE back in to the wrist TE (hand[0] TE)
    R += [(xh, wt0 / 2)]                                                         # wing TE root junction (hinge end)
    R += [(xh + Lt, St / 2), (xh + Lt + bulge, St / 4), (ex, 0.0)]
    i_junction = len(R) - 4
    full = R + [(x, -y) for (x, y) in reversed(R[1:-1])]
    full = ft.orient_ccw(full)
    main_R = R[:i_junction + 1]
    main_full = ft.orient_ccw(main_R + [(x, -y) for (x, y) in reversed(main_R[1:])])
    wing_poly_half = [(0.0, 0.0)] + [(x, y) for (x, y) in R[3:4]] + [(x, y) for (y, x, c) in hand[1:]] + list(reversed(tip_te)) + [(xh, wt0 / 2), (xh, 0.0)]
    # tail polygon (full) starting at the hinge line
    side = lambda x: wt0 / 2 + (x - xh) / Lt * (St / 2 - wt0 / 2)
    tail_half = [(xh, wt0 / 2), (xh + Lt, St / 2), (xh + Lt + bulge, St / 4), (ex, 0.0), (xh, 0.0)]
    tail_area = 2 * abs(ft.area_signed(tail_half))
    wing_area_poly = 2 * abs(ft.area_signed(wing_poly_half))
    g = dict(p=p, b=b, c_root=c_root, x_h=xh, c_wrist=cw, x_wrist=xw, y_wrist=yw, half_span=half, hand=hand, tip_te=tip_te, wt0=wt0, Lt=Lt, St=St, bulge=bulge, tail_end_x=ex,
             y_nodes=ys, le_nodes=les, te_nodes=tes, S=S, S_polygon=wing_area_poly, mac=mac, x_le_mac=x_le_mac, x_ref=x_le_mac + 0.25 * mac, ar=b * b / S,
             half_outline=R, full_outline=full, main_full=main_full, tail_area=tail_area, sheet_area=abs(ft.area_signed(full)), nose_x=xn, wb=wb, side=side, bend_deg=bend_deg)
    return g


def wing_profile(p, c_root, xh, cw, xw, hand, wt0):
    """Piecewise-linear LE / TE of the half wing: nodes at the centreline, the tail-root junction, the wrist and every hand station."""
    tan_a = math.tan(math.radians(p["arm_sweep_deg"]))
    ys = [0.0, wt0 / 2, hand[0][0]] + [y for (y, x, c) in hand[1:]]
    les = [0.0, (wt0 / 2) * tan_a, xw] + [x for (y, x, c) in hand[1:]]
    tes = [xh, xh, xw + cw] + [x + c for (y, x, c) in hand[1:]]
    return ys, les, tes


def polygon_area_mm2(poly, span_mm):
    return abs(ft.area_signed(poly)) * span_mm * span_mm


# =============================================================================================== AVL
def wing_sections(g, tipbend=None):
    """AVL wing sections (y, x_le, z, chord) for the right half; tipbend = (eta_b, angle_deg): the hand panel beyond the bend line is folded about a chordwise line."""
    ys, les, tes = g["y_nodes"], g["le_nodes"], g["te_nodes"]
    secs = [(y, x, 0.0, t - x) for y, x, t in zip(ys, les, tes)]
    if not tipbend:
        return secs
    eta_b, ang = tipbend
    yb = g["y_wrist"] + eta_b * (g["half_span"] - g["y_wrist"])
    out = []
    inserted = False
    for i, s in enumerate(secs):
        if not inserted and s[0] > yb + 1e-12:
            a, c = secs[i - 1], s
            t = (yb - a[0]) / (c[0] - a[0])
            out.append((yb, a[1] + t * (c[1] - a[1]), 0.0, a[3] + t * (c[3] - a[3])))
            inserted = True
        out.append(s)
    res = []
    for y, x, z, c in out:
        if y > yb + 1e-12:
            d = y - yb
            res.append((yb + d * math.cos(math.radians(ang)), x, d * math.sin(math.radians(ang)), c))
        else:
            res.append((y, x, z, c))
    return res


def tail_sections(g):
    """AVL tail sections (y, x_le, chord): chord strips of the fan, LE = the hinge line across the root and the slanted side edge outside it."""
    xh, Lt, St, wt0, bl = g["x_h"], g["Lt"], g["St"], g["wt0"], g["bulge"]
    te_pts = [(0.0, g["tail_end_x"]), (St / 4, xh + Lt + bl), (St / 2, xh + Lt)]
    te = lambda y: next(te_pts[i][1] + (y - te_pts[i][0]) / (te_pts[i + 1][0] - te_pts[i][0]) * (te_pts[i + 1][1] - te_pts[i][1]) for i in range(2) if y <= te_pts[i + 1][0] + 1e-12)
    le = lambda y: xh if y <= wt0 / 2 else xh + (y - wt0 / 2) / (St / 2 - wt0 / 2) * Lt
    ys = sorted({0.0, wt0 / 2, St / 4, St / 2})
    return [(y, le(y), max(te(y) - le(y), 0.004 * Lt)) for y in ys]


def avl_deck(g, span_mm, bend_deg, vfin=None, tipbend=None, name="bird", with_tail=True):
    """AVL input text (m).  Wing planform + tail as a separate surface with incidence = -bend; the body strip is ignored.  vfin: ventral fin on the centreline under the tail."""
    k = span_mm / 1000.0
    f = lambda v: f"{v * k:.9f}"
    L = [name, "0.0", "0 0 0.0", f"{g['S'] * k * k:.9f} {g['mac'] * k:.9f} {k:.9f}", f"{f(g['x_ref'])} 0.0 0.0",
         "SURFACE", "Wing", "20 1.0 32 -2.0", "YDUPLICATE", "0.0"]
    for y, x, z, c in wing_sections(g, tipbend):
        L += ["SECTION", f"{f(x)} {f(y)} {f(z)} {f(c)} 0.0", "NACA", "0000"]
    if with_tail:
        L += ["SURFACE", "Tail", "6 1.0 10 -2.0", "YDUPLICATE", "0.0"]
        for y, x, c in tail_sections(g):
            L += ["SECTION", f"{f(x)} {f(y)} 0.0 {f(c)} {-bend_deg:.6f}", "NACA", "0000"]
    if vfin:
        L += ["SURFACE", "VentralFin", "5 1.0 8 -2.0"]
        for z in (0.0, -vfin["h"]):
            L += ["SECTION", f"{f(vfin['x_le'])} 0.0 {f(z)} {f(vfin['chord'])} 0.0", "NACA", "0000"]
    return "\n".join(L) + "\n"


def vfin_for(g, area_frac=VFIN_AREA_FRAC):
    chord = 0.6 * g["Lt"]
    return dict(x_le=g["x_h"] + 0.1 * g["Lt"], chord=chord, h=area_frac * g["S"] / chord, area_frac=area_frac)


def _lsq(xs, ys):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    s = sum((x - mx) ** 2 for x in xs)
    sl = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / s
    return sl, my - sl * mx


def run_case(g, span_mm, bend, workdir: Path, name, vfin=None, tipbend=None, with_tail=True):
    """-> dict: neutral point (fraction of span aft of the wing root LE and % MAC), CM0 and dCM/dCL about the reference, lift slope, trim CG at CL 0.2-0.5 with static
    margin, Cn_beta and Cl_beta (per rad, at CL 0.3, no fin unless vfin).  Two serial AVL calls (a scan and the CL 0.3 point)."""
    w = Wing(span_m=span_mm / 1000, aspect_ratio=g["ar"], taper=0.5, quarter_chord_sweep_deg=0.0, washout_deg=0.0, dihedral_deg=0.0, section="NACA_0000", cg_x_m=g["x_ref"] * span_mm / 1000)
    deck = avl_deck(g, span_mm, bend, vfin, tipbend, name, with_tail)
    pts = run_avl(w, name, list(AVL_ALPHAS), workdir / "scan", AVL_EXE, deck_text=deck)
    cl = [p.cl for p in pts]
    cm = [p.cm for p in pts]
    k, cm0 = _lsq(cl, cm)
    cla_deg = _lsq([p.alpha_deg for p in pts], cl)[0]
    cl0 = _lsq([p.alpha_deg for p in pts], cl)[1]
    a03 = (0.3 - cl0) / cla_deg
    p3 = run_avl(w, name, [a03], workdir / "cl03", AVL_EXE, deck_text=deck)[0]
    xnp = sum(p.xnp_m for p in pts + [p3]) / 4 / (span_mm / 1000)          # fraction of span aft of the wing root LE
    mac, xle_mac, xref = g["mac"], g["x_le_mac"], g["x_ref"]
    res = dict(np_span=xnp, np_pct_mac=100 * (xnp - xle_mac) / mac, cm0=cm0, dcm_dcl=k, cla_per_rad=cla_deg * 180 / math.pi, alpha_zero_lift=-cl0 / cla_deg, cl_at_cn=p3.cl,
               alpha_cl03=a03, cn_beta=p3.cnb_per_rad, cl_beta=p3.clb_per_rad)
    # moment about a point x: Cm_x = Cm_ref + CL (x - x_ref)/mac.  Cm about the neutral point is the constant cm0; trim at CL: x_cg = x_np - mac * cm0 / CL.
    res["np_from_cm_slope_span"] = xref - k * mac                            # NP implied by dCM/dCL: should equal the AVL neutral point
    res["trim"] = {c: dict(x_cg_span=xnp - mac * cm0 / c, static_margin=cm0 / c) for c in TRIM_CLS}
    return res


# =============================================================================================== scan, yaw fixes, pull-out
def tail_area_frac(g):
    return g["tail_area"] / g["S"]


def scan(fam, span_mm, workdir: Path):
    rows = []
    base = {k: v[0] for k, v in FAMILIES[fam]["params"].items()}
    i = 0
    for lm in SCAN_LEVELS:
        for sm in SCAN_LEVELS:
            for hd in SCAN_SWEEP_DELTAS:
                p = dict(base, tail_len_frac=base["tail_len_frac"] * lm, tail_spread_frac=base["tail_spread_frac"] * sm, hand_sweep_deg=base["hand_sweep_deg"] + hd)
                g = build_geometry(p)
                r = run_case(g, span_mm, REF_BEND, workdir / f"{fam}_scan_{i:02d}", f"{fam}scan{i}")
                af = tail_area_frac(g)
                rows.append(dict(family=fam, tail_len_mult=lm, tail_spread_mult=sm, hand_sweep_deg=p["hand_sweep_deg"], tail_area_frac=af, cm0=r["cm0"], np_pct_mac=r["np_pct_mac"],
                                 sm_cl03=r["trim"][0.3]["static_margin"], cn_beta=r["cn_beta"], cl_beta=r["cl_beta"], cm0_per_tail_area=r["cm0"] / af, sm_per_tail_area=r["trim"][0.3]["static_margin"] / af,
                                 ar=g["ar"], tail_area_over_wing_area=af))
                i += 1
    return rows


def main_effects(rows):
    """Average of each metric over the scan at each level of each factor (an honest summary of 'which choice buys what')."""
    out = []
    for fac, key in (("tail length x", "tail_len_mult"), ("tail spread x", "tail_spread_mult"), ("hand sweep deg", "hand_sweep_deg")):
        for lv in sorted({r[key] for r in rows}):
            rs = [r for r in rows if r[key] == lv]
            m = lambda k: sum(r[k] for r in rs) / len(rs)
            out.append(dict(factor=fac, level=lv, n=len(rs), cm0=m("cm0"), sm_cl03=m("sm_cl03"), tail_area_frac=m("tail_area_frac"), cm0_per_tail_area=m("cm0_per_tail_area"),
                            sm_per_tail_area=m("sm_per_tail_area"), cn_beta=m("cn_beta"), np_pct_mac=m("np_pct_mac")))
    return out


def yaw_fixes(fam, g, span_mm, workdir: Path, base):
    rows = [dict(family=fam, fix="none (baseline)", detail="no fin, flat wing", **{k: base[k] for k in ("cn_beta", "cl_beta", "np_pct_mac", "cm0")})]
    vf = vfin_for(g)
    r = run_case(g, span_mm, REF_BEND, workdir / f"{fam}_vfin", f"{fam}vfin", vfin=vf)
    rows.append(dict(family=fam, fix="ventral fin", detail=f"{100 * VFIN_AREA_FRAC:g}% of wing area, chord {vf['chord'] * span_mm:.1f} mm, height {vf['h'] * span_mm:.1f} mm below the sheet, under the tail", cn_beta=r["cn_beta"], cl_beta=r["cl_beta"], np_pct_mac=r["np_pct_mac"], cm0=r["cm0"]))
    for ang in TIP_BEND_ANGLES:
        r = run_case(g, span_mm, REF_BEND, workdir / f"{fam}_tip{int(ang):+d}", f"{fam}tip{int(abs(ang))}", tipbend=(TIP_BEND_ETA, ang))
        rows.append(dict(family=fam, fix=f"tip bent {'down' if ang < 0 else 'up'}", detail=f"outer {100 * (1 - TIP_BEND_ETA):.0f}% of the hand panel bent {abs(ang):g} deg about a chordwise line", cn_beta=r["cn_beta"], cl_beta=r["cl_beta"], np_pct_mac=r["np_pct_mac"], cm0=r["cm0"]))
    return rows


def pullout_radius(ws_n_m2, cl, rho=ft.RHO_AIR):
    """Minimum turn radius of a steady circular pull-up at constant CL, n >> 1:  R = 2 (W/S) / (rho g CL).  ARITHMETIC."""
    return 2.0 * ws_n_m2 / (rho * G_ACCEL * cl)


# =============================================================================================== all AVL tables, build proposals, masses
def family_geometry(fam):
    return build_geometry({k: v[0] for k, v in FAMILIES[fam]["params"].items()})


def avl_all(geoms, workdir: Path):
    """Serial AVL for every family x tail bend.  -> dict[(fam, bend)] = run_case result (dimensionless; scale by the span for mm)."""
    out = {}
    for fam in FAM_ORDER:
        for bend in TAIL_BENDS:
            out[(fam, bend)] = run_case(geoms[fam], SIZES["printed"]["span_mm"], bend, workdir / f"{fam}_bend{int(bend)}", f"{fam}b{int(bend)}")
    return out


def propose_bends(fam, res, geoms):
    """PROPOSAL rule (JUDGEMENT, stated on the page): bends above 0 whose CL 0.3 trim CG lies at or behind the wing root leading edge (a CG ahead of it
    cannot be reached with the existing rails), ranked by how close the CL 0.3 static margin is to the middle of SM_BAND; up to N_BUILD_BENDS are proposed."""
    mid = sum(SM_BAND) / 2
    ok = [b for b in TAIL_BENDS if b > 0 and res[(fam, b)]["trim"][0.3]["x_cg_span"] >= 0.0]
    ok.sort(key=lambda b: abs(res[(fam, b)]["trim"][0.3]["static_margin"] - mid))
    if not ok:                                                              # nothing reachable: propose the smallest bend and say so
        ok = [min(b for b in TAIL_BENDS if b > 0)]
    return sorted(ok[:N_BUILD_BENDS])


KIT = ROOT / "results/runs/rigid_glider/flat_test_v2"
RAIL_SPEC = {"printed": "F45_span250", "foam": "B45_span400"}


def rail_info(size_key, geom_span_for_cg):
    """The existing CG rail of this size: mass, centroid, ballast station, saddle end, tick range.  Read from the flat-test kit's geometry.json (read only) when it
    exists, otherwise regenerated with flat_test.stick_mesh; never copied by hand."""
    sz = SIZES[size_key]
    gj = KIT / RAIL_SPEC[size_key] / "geometry.json"
    if gj.is_file():
        G = json.loads(gj.read_text())
        v = lambda k: G[k]["value"]
        nose = v("stick_nose_ahead_of_root_le_mm")
        holes = sorted(x for x in v("ballast_holes_x_mm") if x < 0)
        station = min(holes, key=lambda x: abs(x + ft.BALLAST_STATION_FRAC_OF_NOSE * nose))
        tr = v("stick_tick_range_mm")
        return dict(mass_g=ft.mass_g(v("stick_volume_mm3")), cx=v("stick_centroid_x_mm"), station=station, saddle_end=v("stick_length_mm") - nose, nose=nose, length=v("stick_length_mm"),
                    tick_range=(tr[0], tr[1]), name=sz["rail_name"], source=str(gj.relative_to(ROOT)).replace("\\", "/"))
    gr = ft.wing_geometry(sz["rail_span"], RAIL_AR)
    tris, info = ft.stick_mesh(gr, *geom_span_for_cg)
    st = ft.mesh_stats(tris)
    nose = info["layout"]["nose"]
    holes = sorted(h[1] for h in info["holes"] if h[0] == "ballast" and h[1] < 0)
    station = min(holes, key=lambda x: abs(x + ft.BALLAST_STATION_FRAC_OF_NOSE * nose))
    ticks = [t["x"] for t in info["ticks"]]
    return dict(mass_g=ft.mass_g(st["volume"]), cx=st["centroid"][0], station=station, saddle_end=info["layout"]["x_hi"], nose=nose, length=info["layout"]["x_hi"] - info["layout"]["x_lo"],
                tick_range=(min(ticks), max(ticks)), name=sz["rail_name"], source="regenerated with flat_test.stick_mesh")


def printed_meshes(g, span_mm, t):
    """Three closed shells (print flat on the bed, 0.6 mm plate): main sheet (nose + wing, ending at the hinge line), scored-hinge strip (one layer), fan tail."""
    k = span_mm
    mm = lambda poly: [(x * k, y * k) for x, y in poly]
    main = ft.orient_ccw(mm(g["main_full"]))
    xh = g["x_h"] * k
    gap, ov, wt0 = HINGE_WIDTH_MM, HINGE_OVERLAP_MM, g["wt0"] * k
    side = lambda x: wt0 / 2 + (x - xh) / (g["Lt"] * k) * (g["St"] * k / 2 - wt0 / 2)
    x0 = xh + gap
    A, B, E = (xh + g["Lt"] * k, g["St"] * k / 2), (xh + (g["Lt"] + g["bulge"]) * k, g["St"] * k / 4), (g["tail_end_x"] * k, 0.0)
    tail = ft.orient_ccw([(x0, side(x0)), A, B, E, (B[0], -B[1]), (A[0], -A[1]), (x0, -side(x0))])
    strip = ft.orient_ccw([(xh - ov, -wt0 / 2), (xh + gap + ov, -wt0 / 2), (xh + gap + ov, wt0 / 2), (xh - ov, wt0 / 2)])
    return dict(main=ft.prism(main, 0.0, t), tail=ft.prism(tail, 0.0, t), hinge=ft.prism(strip, 0.0, HINGE_THICK_MM)), dict(main=main, tail=tail, strip=strip)


def build_numbers(fam, size_key, bend, g, res):
    """Mass, wing loading and pull-out radii of one proposed build.  Printed: mesh volume x PLA density (ASSUMED in flat_test).  Foam: area x areal mass (ASSUMED)."""
    sz = SIZES[size_key]
    span, t = sz["span_mm"], sz["thickness_mm"]
    area_sheet = g["sheet_area"] * span * span
    S_w = g["S"] * span * span
    if size_key == "printed":
        shells, _ = printed_meshes(g, span, t)
        vol = sum(ft.mesh_stats(s_)["volume"] for s_ in shells.values())
        m_sheet = ft.mass_g(vol)
    else:
        m_sheet = area_sheet * 1e-6 * ft.FOAM_AREAL_MASS_G_M2
    cx_sheet = ft.polygon_centroid(g["full_outline"])[0] * span
    tr = res["trim"][0.3]
    cg_target = tr["x_cg_span"] * span
    cgs = [res["trim"][c]["x_cg_span"] * span for c in TRIM_CLS]
    rail = rail_info(size_key, (max(min(cgs), 0.0), max(max(cgs), 10.0)))
    M0 = m_sheet + rail["mass_g"]
    cg0 = (m_sheet * cx_sheet + rail["mass_g"] * rail["cx"]) / M0
    mb = ft.ballast_g(M0, cg0, cg_target, rail["station"])
    reachable = mb >= 0
    ballast = max(mb, 0.0)
    M = M0 + ballast
    W = M * 1e-3 * G_ACCEL
    ws = W / (S_w * 1e-6)
    pull = {cl: pullout_radius(ws, cl) for cl in PULLOUT_CLS}
    return dict(fam=fam, size=size_key, span=span, t=t, bend=bend, area_sheet_mm2=area_sheet, S_w_mm2=S_w, m_sheet=m_sheet, rail=rail, cx_sheet=cx_sheet, M0=M0, cg0=cg0, cg_target=cg_target,
                ballast_g=ballast, ballast_reachable=reachable, ballast_raw=mb, M=M, ws=ws, pull=pull, t_pct_mac=100 * t / (g["mac"] * span), t_pct_croot=100 * t / (g["c_root"] * span),
                rail_fits_under_hinge=rail["saddle_end"] <= g["x_h"] * span, hinge_x_mm=g["x_h"] * span, trim_cgs_mm=cgs,
                rail_covers_trim=(rail["tick_range"][0] <= min(cgs) and rail["tick_range"][1] >= max(cgs)))


# =============================================================================================== DXF, drawing, templates
def write_dxf_bird(path: Path, g, span_mm, marks):
    ent = []

    def line(layer, p, q):
        ent.append(f"0\nLINE\n8\n{layer}\n10\n{p[0]:.4f}\n20\n{p[1]:.4f}\n30\n0.0\n11\n{q[0]:.4f}\n21\n{q[1]:.4f}\n31\n0.0\n")
    k = span_mm
    out = g["full_outline"]
    for i in range(len(out)):
        a, b = out[i], out[(i + 1) % len(out)]
        line("OUTLINE", (a[0] * k, a[1] * k), (b[0] * k, b[1] * k))
    xh, wt0 = g["x_h"] * k, g["wt0"] * k
    line("HINGE_BEND_UP", (xh, -wt0 / 2), (xh, wt0 / 2))
    for s in (1, -1):
        line("WRIST_MARKER", (g["x_wrist"] * k, s * g["y_wrist"] * k), ((g["x_wrist"] + g["c_wrist"]) * k, s * g["y_wrist"] * k))
    line("CENTRELINE", (g["nose_x"] * k - 5, 0.0), (g["tail_end_x"] * k + 5, 0.0))
    for lab, x in marks:
        line("CG_MARKS", (x, -4.0), (x, 4.0))
    path.write_text("0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n0\nSECTION\n2\nTABLES\n0\nTABLE\n2\nLAYER\n70\n5\n"
                    + "".join(f"0\nLAYER\n2\n{l}\n70\n0\n62\n{c}\n6\nCONTINUOUS\n" for l, c in (("OUTLINE", 7), ("HINGE_BEND_UP", 1), ("WRIST_MARKER", 3), ("CENTRELINE", 8), ("CG_MARKS", 5)))
                    + "0\nENDTAB\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n" + "".join(ent) + "0\nENDSEC\n0\nEOF\n", encoding="ascii")


def drawing_svg_bird(g, span_mm, label, bends, res, nums, size_key):
    """Dark on-screen drawing, nose up.  Page x = span (y), page y = aft (x)."""
    s = 760.0 / span_mm
    k = span_mm
    x0 = 20 + 380
    ytop = 110.0
    sx = lambda y: x0 + y * k * s
    xmin = g["nose_x"] * k
    sy = lambda x: ytop + (x * k - xmin) * s
    xmax = max(px for px, py in g["full_outline"]) * k
    W, H = 800, int(ytop + (xmax - xmin) * s + 230)
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">', '<rect width="100%" height="100%" fill="#14181d"/>',
         ft._txt(20, 26, f"{label}: {SIZES[size_key]['material']} {SIZES[size_key]['thickness_mm']:g} mm, span {span_mm:g} mm: one flat sheet, tail bent trailing-edge UP along one hinge line", 14, color="#dde3ea", weight="bold"),
         ft._txt(20, 46, "Top view, nose up. Dimensions in mm. G = GEOMETRY, J = JUDGEMENT, AVL = outline-only model (body ignored; says nothing about stall or Reynolds number).", 11, color="#9aa7b4")]
    poly = " ".join(f"{sx(y):.1f},{sy(x):.1f}" for x, y in g["full_outline"])
    o.append(f'<polygon points="{poly}" fill="#27313c" stroke="#dde3ea" stroke-width="1.6"/>')
    xh, wt0 = g["x_h"], g["wt0"]
    tail = [(xh, wt0 / 2), (xh + g["Lt"], g["St"] / 2), (xh + g["Lt"] + g["bulge"], g["St"] / 4), (g["tail_end_x"], 0.0), (g["tail_end_x"] * 1.0, 0.0)]
    tp = [(x, y) for x, y in tail[:4]] + [(x, -y) for x, y in reversed(tail[1:3])] + [(xh, -wt0 / 2)]
    o.append(f'<polygon points="{" ".join(f"{sx(y):.1f},{sy(x):.1f}" for x, y in tp)}" fill="#3a3322" fill-opacity="0.9" stroke="#e89a55" stroke-width="1"/>')
    o.append(f'<line x1="{sx(-wt0 / 2):.1f}" y1="{sy(xh):.1f}" x2="{sx(wt0 / 2):.1f}" y2="{sy(xh):.1f}" stroke="#ff8a5c" stroke-width="3" stroke-dasharray="6 3"/>')
    o.append(ft._txt(sx(wt0 / 2) + 8, sy(xh) - 6, f"hinge line (bend UP, trailing edge raised): {xh * k:.1f} mm aft of root LE (G)", 11, color="#ff8a5c"))
    for sgn in (1, -1):
        o.append(f'<line x1="{sx(sgn * g["y_wrist"]):.1f}" y1="{sy(g["x_wrist"]):.1f}" x2="{sx(sgn * g["y_wrist"]):.1f}" y2="{sy(g["x_wrist"] + g["c_wrist"]):.1f}" stroke="#7fb4ff" stroke-width="1.5" stroke-dasharray="4 3"/>')
    o.append(ft._txt(sx(g["y_wrist"]) + 5, sy(g["x_wrist"]) + 14, "wrist joint line (marker only, not cut)", 11, color="#7fb4ff"))
    o.append(f'<line x1="{x0:.1f}" y1="{sy(xmin / k) - 10:.1f}" x2="{x0:.1f}" y2="{sy(g["tail_end_x"]) + 24:.1f}" stroke="#9aa7b4" stroke-width="1" stroke-dasharray="8 3 2 3"/>')
    # CG scale along the centreline: mm aft of the wing root leading edge
    for xm in range(int(math.floor(xmin / 5) * 5), int(g["tail_end_x"] * k) + 1, 5):
        yy = sy(xm / k)
        ln = 8 if xm % 10 == 0 else 4
        o.append(f'<line x1="{x0 - ln:.1f}" y1="{yy:.1f}" x2="{x0 + ln:.1f}" y2="{yy:.1f}" stroke="#dde3ea" stroke-width="0.8"/>')
        if xm % 10 == 0:
            o.append(ft._txt(x0 + 12, yy + 4, f"{xm}", 10, color="#dde3ea"))
    o.append(ft._txt(x0 + 12, sy(0.0) - 6, "0 = wing root leading edge at the centreline", 11, color="#6cc08b", weight="bold"))
    o.append(f'<line x1="{x0 - 30:.1f}" y1="{sy(0.0):.1f}" x2="{x0 + 30:.1f}" y2="{sy(0.0):.1f}" stroke="#6cc08b" stroke-width="2"/>')
    # AVL neutral point and trim CG markers per proposed bend
    ly = 20
    cols = ["#ff8a80", "#c39bff", "#e0b83a"]
    for i, b in enumerate(bends):
        r = res[b]
        xnp = r["np_span"] * k
        xcg = r["trim"][0.3]["x_cg_span"] * k
        c = cols[i % 3]
        o.append(f'<line x1="{x0 - 150:.1f}" y1="{sy(xnp / k):.1f}" x2="{x0 + 150:.1f}" y2="{sy(xnp / k):.1f}" stroke="{c}" stroke-width="1.2" stroke-dasharray="5 3"/>')
        o.append(ft._txt(x0 + 154, sy(xnp / k) + 4, f"AVL neutral point, bend {b:g}: {xnp:.1f} mm", 10, color=c))
        o.append(f'<line x1="{x0 - 60:.1f}" y1="{sy(xcg / k):.1f}" x2="{x0 + 60:.1f}" y2="{sy(xcg / k):.1f}" stroke="{c}" stroke-width="2.4"/>')
        o.append(ft._txt(x0 - 64, sy(xcg / k) + 4, f"AVL trim CG, CL 0.3, bend {b:g}: {xcg:.1f} mm", 10, "end", c))
    yb = H - 100
    o.append(ft._txt(20, yb, f"Span {span_mm:g} (G). Wing area {g['S'] * k * k:,.0f} mm2 (G), aspect ratio {g['ar']:.2f}, MAC {g['mac'] * k:.1f} mm (G). Tail {g['tail_area'] / g['S'] * 100:.0f}% of wing area (G). Sheet {g['sheet_area'] * k * k:,.0f} mm2 (G).", 11, color="#dde3ea"))
    o.append(ft._txt(20, yb + 18, "Tick marks: every 5 mm, longer every 10 mm, mm aft of the wing root leading edge. Markers are AVL reference points, not a CG window and not a prediction.", 11, color="#9aa7b4"))
    o.append(ft._txt(20, yb + 36, "Bend direction: the tail's trailing edge goes UP (tail set nose-down relative to the wing); see the angle gauge on the template notes page.", 11, color="#9aa7b4"))
    o.append("</svg>")
    return "\n".join(o)


LETTER_W, LETTER_H, MARGIN = ft.LETTER_W, ft.LETTER_H, ft.TEMPLATE_MARGIN


def _mask(o):
    W, H, M = LETTER_W, LETTER_H, MARGIN
    for x, y, w, h in ((0, 0, M, H), (W - M, 0, M, H), (0, 0, W, M), (0, H - M, W, M)):
        o.append(f'<rect x="{x:.2f}" y="{y:.2f}" width="{w:.2f}" height="{h:.2f}" fill="#ffffff"/>')


def template_pages(fam, g, span_mm, bends, res, label):
    """1:1 US Letter portrait print templates (rect / line / polygon / text only): one half sheet with its fan tail, centreline, hinge line with bend arrow text, wrist
    marker line and a CG tick scale; tiled across pages with overlap marks when the half does not fit one page.  Plus a notes page with an angle gauge per bend."""
    k = span_mm
    half = [(x * k, y * k) for x, y in g["half_outline"]]
    xs = [p[0] for p in half]
    xmin, xmax = min(xs) - 4.0, max(xs) + 4.0
    pw = LETTER_W - 2 * MARGIN
    need = xmax - xmin
    tiles = [xmin]
    while tiles[-1] + pw < xmax - 1e-9:
        tiles.append(tiles[-1] + pw - OVERLAP_MM)
    ycl = 240.0
    marks = [(f"AVL trim CG at CL 0.3, bend {b:g}", res[b]["trim"][0.3]["x_cg_span"] * k) for b in bends]
    pages = []
    xh, wt0 = g["x_h"] * k, g["wt0"] * k
    for ti, t0 in enumerate(tiles):
        X = lambda x: MARGIN + (x - t0)
        Y = lambda y: ycl - y
        o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{LETTER_W}mm" height="{LETTER_H}mm" viewBox="0 0 {LETTER_W} {LETTER_H}">', '<rect width="100%" height="100%" fill="white"/>']
        pts = " ".join(f"{X(x):.3f},{Y(y):.3f}" for x, y in half)
        o.append(f'<polygon points="{pts}" fill="#eef3f8" fill-opacity="0.6" stroke="#000" stroke-width="0.5"/>')
        o.append(ft._l(X(xh), Y(0), X(xh), Y(wt0 / 2), "#c2621a", 0.7, "3 1.5"))
        o.append(ft._l(X(g["x_wrist"] * k), Y(g["y_wrist"] * k), X((g["x_wrist"] + g["c_wrist"]) * k), Y(g["y_wrist"] * k), "#2b6cb0", 0.35, "2 1.5"))
        o.append(ft._l(X(xmin), Y(0), X(xmax), Y(0), "#555", 0.35, "3 2"))
        # CG tick scale on the centreline, mm aft of the wing root leading edge
        for xm in range(int(math.ceil(xmin)), int(xmax) + 1):
            if xm % 5:
                continue
            ln = 5.0 if xm % 10 == 0 else 2.5
            o.append(ft._l(X(xm), Y(0) - ln, X(xm), Y(0), "#000", 0.3))
            if xm % 10 == 0:
                o.append(ft._t(X(xm), Y(0) + 5.0, f"{xm}", 2.6, "middle"))
        o.append(ft._l(X(0), Y(0) - 9, X(0), Y(0) + 9, "#c2621a", 0.6))
        o.append(ft._t(X(0) + 1.0, Y(0) + 12.5, "0 = wing root leading edge", 2.8, color="#c2621a", weight="bold"))
        for lab, xm in marks:
            o.append(ft._l(X(xm), Y(0) - 12, X(xm), Y(0) + 2, "#1a7f4b", 0.6))
        o.append(ft._t(X(max(xmin, t0 + 2)) , Y(0) + 17.0, "centreline: fold or flip here for the other half", 2.8, color="#555"))
        o.append(ft._t(X(xh) + 2.0, Y(wt0 / 2) - 2.0, f"hinge: bend tail UP (trailing edge up); score the TOP face", 2.8, color="#c2621a"))
        o.append(ft._t(X(g["x_wrist"] * k) + 1.0, Y(g["y_wrist"] * k) - 1.5, "wrist line (marker only)", 2.6, color="#2b6cb0"))
        _mask(o)
        o.append(ft._t(12, 14, f"{label}: ONE HALF of the sheet with its fan tail, 1:1, US Letter portrait (page {ti + 1} of {len(tiles)})", 4.4, weight="bold"))
        o.append(ft._t(12, 19.5, f"Foamboard {SIZES['foam']['thickness_mm']:g} mm (ASSUMED: measure yours). Span {span_mm:g} mm. Cut along the solid outline; trace, flip on the centreline, trace the other half.", 2.6))
        o.append(ft._t(12, 23.7, "One sheet, one bend: bend the tail up along the hinge line to the angle on the notes page. Print at 100 % (actual size).", 2.6))
        o.append(ft._t(12, 27.9, "Green ticks mark the AVL trim CG at CL 0.3 for the proposed bends (reference points, not a CG window).", 2.6))
        if len(tiles) > 1:
            if ti + 1 < len(tiles):
                xb = tiles[ti + 1]
                o.append(ft._l(X(xb), MARGIN, X(xb), LETTER_H - MARGIN, "#c2621a", 0.4, "4 2"))
                o.append(ft._t(X(xb) - 1.0, 34.0, f"overlap: page {ti + 2} starts here, align this line", 2.6, "end", "#c2621a"))
                for yy in (60.0, 150.0):
                    o.append(ft._l(X(xb) - 3, yy, X(xb) + 3, yy, "#c2621a", 0.4))
            if ti > 0:
                xb = t0 + pw
                prev = tiles[ti - 1] + pw
                o.append(ft._l(X(prev), MARGIN, X(prev), LETTER_H - MARGIN, "#2b6cb0", 0.4, "4 2"))
                o.append(ft._t(X(prev) + 1.0, 34.0, f"overlap: end of page {ti} is this line", 2.6, "start", "#2b6cb0"))
                for yy in (60.0, 150.0):
                    o.append(ft._l(X(prev) - 3, yy, X(prev) + 3, yy, "#2b6cb0", 0.4))
        o.extend(ft._scale_bar(MARGIN + 2.0, 258.0))
        o.append("</svg>")
        pages.append("\n".join(o))
    # ---- notes page: angle gauges and instructions
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{LETTER_W}mm" height="{LETTER_H}mm" viewBox="0 0 {LETTER_W} {LETTER_H}">', '<rect width="100%" height="100%" fill="white"/>',
         ft._t(12, 14, f"{label}: build notes and tail-bend angle gauge, 1:1", 4.4, weight="bold"),
         ft._t(12, 20, "Tail bend is a rotation of the fan tail about the hinge line, trailing edge UP. Score the top face with a V cut (leave the bottom paper),", 2.6),
         ft._t(12, 24.2, "bend the tail up to the gauge line, glue the crease. One sheet, one bend. A flat symmetric sheet has no nose-up moment at zero lift;", 2.6),
         ft._t(12, 28.4, "this bend is what supplies it. These are build instructions, not a prediction that the glider pulls out.", 2.6)]
    y0 = 70.0
    for i, b in enumerate(bends):
        base = 120.0
        rise = base * math.tan(math.radians(b))
        yy = y0 + i * 70.0
        o.append(f'<polygon points="{MARGIN + 10:.2f},{yy:.2f} {MARGIN + 10 + base:.2f},{yy:.2f} {MARGIN + 10 + base:.2f},{yy - rise:.2f}" fill="#fbeedd" fill-opacity="0.6" stroke="#000" stroke-width="0.4"/>')
        o.append(ft._t(MARGIN + 12, yy + 5.5, f"gauge for a {b:g} deg bend: lay the wing along the long edge; the tail's top face follows the slanted edge", 2.6))
        o.append(ft._t(MARGIN + 12 + base * 0.45, yy - 2.0, f"{b:g} deg", 3.0, color="#c2621a", weight="bold"))
    o.append(ft._t(12, 225.0, "Wrist joint line: a drawn marker for later folding; the sheet is not cut or creased there.", 2.6))
    o.append(ft._t(12, 230.0, "CG: measure with a knife edge; the 0 mark of the printed CG rail goes at the wing root leading edge at the centreline.", 2.6))
    o.extend(ft._scale_bar(MARGIN + 2.0, 258.0))
    o.append("</svg>")
    pages.append("\n".join(o))
    return pages


# =============================================================================================== provenance (what is bird data, what is judgement)
def provenance_rows():
    rows = []
    for fam in FAM_ORDER:
        for name, (val, src, note) in FAMILIES[fam]["params"].items():
            v = val if not isinstance(val, tuple) else "; ".join(f"({a:g}, {b:g})" for a, b in val)
            rows.append(dict(family=fam, parameter=name, value=v, source=src, note=note))
    return rows


# =============================================================================================== CSV
def write_csv(path: Path, header, rows):
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def f_(v, n=3):
    return "" if v is None else (f"{v:.{n}f}" if isinstance(v, float) else str(v))


def write_tables(out: Path, geoms, res, scans, effects, yaw, builds):
    write_csv(out / "dimensions.csv", ["family", "parameter", "value", "source_tag", "note"], [[r["family"], r["parameter"], r["value"], r["source"], r["note"]] for r in provenance_rows()])
    rows = []
    for fam in FAM_ORDER:
        g = geoms[fam]
        for sz in ("printed", "foam"):
            k = SIZES[sz]["span_mm"]
            for q, v, u, src in (("span", k, "mm", "GEOMETRY"), ("wing area (outline, body-blended, tail excluded)", g["S"] * k * k, "mm2", "GEOMETRY"), ("sheet area (wing + body + tail)", g["sheet_area"] * k * k, "mm2", "GEOMETRY"),
                                 ("tail area", g["tail_area"] * k * k, "mm2", "GEOMETRY"), ("tail area / wing area", g["tail_area"] / g["S"], "", "GEOMETRY"), ("aspect ratio b^2/S", g["ar"], "", "GEOMETRY"),
                                 ("mean aerodynamic chord", g["mac"] * k, "mm", "GEOMETRY"), ("MAC leading edge, aft of root LE", g["x_le_mac"] * k, "mm", "GEOMETRY"),
                                 ("root chord (centreline)", g["c_root"] * k, "mm", "GEOMETRY"), ("wrist chord", g["c_wrist"] * k, "mm", "GEOMETRY"), ("wrist at semi-span fraction", g["y_wrist"] / g["half_span"], "", "GEOMETRY"),
                                 ("hinge line, aft of root LE", g["x_h"] * k, "mm", "GEOMETRY"), ("hinge width", g["wt0"] * k, "mm", "GEOMETRY"), ("tail length behind wing TE", g["Lt"] * k, "mm", "GEOMETRY"),
                                 ("tail spread", g["St"] * k, "mm", "GEOMETRY"), ("nose ahead of root LE", -g["nose_x"] * k, "mm", "GEOMETRY"), ("reference point for CM (MAC quarter chord), aft of root LE", g["x_ref"] * k, "mm", "GEOMETRY"),
                                 ("thickness / MAC", 100 * SIZES[sz]["thickness_mm"] / (g["mac"] * k), "%", "GEOMETRY (foam: ASSUMED thickness)"), ("thickness / root chord", 100 * SIZES[sz]["thickness_mm"] / (g["c_root"] * k), "%", "GEOMETRY (foam: ASSUMED thickness)")):
                rows.append([fam, FAMILIES[fam]["label"], sz, q, f_(v, 4), u, src])
    write_csv(out / "outline_numbers.csv", ["family", "label", "size", "quantity", "value", "unit", "source_tag"], rows)
    rows = []
    for fam in FAM_ORDER:
        for b in TAIL_BENDS:
            r = res[(fam, b)]
            for c in TRIM_CLS:
                tr = r["trim"][c]
                rows.append([fam, b, c, f_(r["np_pct_mac"], 2), f_(r["np_span"] * 250, 2), f_(r["np_span"] * 400, 2), f_(r["cm0"], 5), f_(tr["x_cg_span"] * 250, 2), f_(tr["x_cg_span"] * 400, 2),
                             f_(100 * tr["static_margin"], 2), f_(r["cn_beta"], 5), f_(r["cl_beta"], 5), f_(r["cla_per_rad"], 3), "AVL (outline only, body ignored, inviscid)"])
    write_csv(out / "avl_table.csv", ["family", "tail_bend_deg_TE_up", "CL", "neutral_point_pct_MAC", "neutral_point_mm_aft_of_root_LE_250", "neutral_point_mm_400", "CM0_about_MAC_quarter_chord (zero lift, nose-up +)",
                                      "trim_CG_mm_250", "trim_CG_mm_400", "static_margin_pct_MAC_at_trim", "Cn_beta_per_rad_no_fin_CL0.3", "Cl_beta_per_rad_CL0.3", "lift_slope_per_rad", "tag"], rows)
    write_csv(out / "scan.csv", ["family", "tail_length_mult", "tail_spread_mult", "hand_sweep_deg", "tail_area_over_wing_area", "CM0 (bend %g deg)" % REF_BEND, "neutral_point_pct_MAC", "static_margin_CL0.3", "Cn_beta_per_rad",
                                 "Cl_beta_per_rad", "CM0_per_tail_area_fraction", "SM_CL0.3_per_tail_area_fraction", "aspect_ratio"],
              [[r["family"], r["tail_len_mult"], r["tail_spread_mult"], r["hand_sweep_deg"], f_(r["tail_area_frac"], 4), f_(r["cm0"], 5), f_(r["np_pct_mac"], 2), f_(r["sm_cl03"], 4), f_(r["cn_beta"], 5),
                f_(r["cl_beta"], 5), f_(r["cm0_per_tail_area"], 4), f_(r["sm_per_tail_area"], 4), f_(r["ar"], 3)] for fam in scans for r in scans[fam]])
    write_csv(out / "scan_main_effects.csv", ["family", "factor", "level", "n", "mean_CM0", "mean_SM_CL0.3", "mean_tail_area_fraction", "mean_CM0_per_tail_area", "mean_SM_per_tail_area", "mean_Cn_beta", "mean_NP_pct_MAC"],
              [[fam, e["factor"], e["level"], e["n"], f_(e["cm0"], 5), f_(e["sm_cl03"], 4), f_(e["tail_area_frac"], 4), f_(e["cm0_per_tail_area"], 4), f_(e["sm_per_tail_area"], 4), f_(e["cn_beta"], 5), f_(e["np_pct_mac"], 2)]
               for fam in effects for e in effects[fam]])
    write_csv(out / "yaw_fixes.csv", ["family", "fix", "detail", "Cn_beta_per_rad_CL0.3", "Cl_beta_per_rad_CL0.3", "neutral_point_pct_MAC", "CM0", "tag"],
              [[r["family"], r["fix"], r["detail"], f_(r["cn_beta"], 5), f_(r["cl_beta"], 5), f_(r["np_pct_mac"], 2), f_(r["cm0"], 5), f"AVL, tail bend {REF_BEND:g} deg, each fix evaluated separately"] for fam in yaw for r in yaw[fam]])
    write_csv(out / "builds.csv", ["family", "size", "tail_bend_deg", "sheet_mass_g", "rail_mass_g", "bare_CG_mm", "target_trim_CG_CL0.3_mm", "ballast_g_at_nose_hole", "ballast_reachable_with_nose_ballast", "all_up_g",
                                   "wing_loading_N_m2", "pullout_radius_m_CL0.3", "pullout_radius_m_CL0.5", "thickness_pct_MAC", "rail", "rail_saddle_end_mm", "hinge_mm", "rail_fits_ahead_of_hinge", "rail_ticks_cover_trim_CGs", "tag"],
              [[b["fam"], b["size"], b["bend"], f_(b["m_sheet"], 2), f_(b["rail"]["mass_g"], 2), f_(b["cg0"], 1), f_(b["cg_target"], 1), f_(b["ballast_raw"], 2), "yes" if b["ballast_reachable"] else "NO", f_(b["M"], 2), f_(b["ws"], 2),
                f_(b["pull"][0.3], 2), f_(b["pull"][0.5], 2), f_(b["t_pct_mac"], 2), b["rail"]["name"], f_(b["rail"]["saddle_end"], 1), f_(b["hinge_x_mm"], 1), "yes" if b["rail_fits_under_hinge"] else "NO",
                "yes" if b["rail_covers_trim"] else "NO", "ARITHMETIC (densities ASSUMED in flat_test.py; AVL trim CG)"] for b in builds])
    write_csv(out / "pullout.csv", ["family", "size", "tail_bend_deg", "wing_loading_N_m2", "CL", "min_turn_radius_m", "assumptions"],
              [[b["fam"], b["size"], b["bend"], f_(b["ws"], 2), c, f_(b["pull"][c], 2), "ARITHMETIC: R = 2 (W/S) / (rho g CL); steady circular pull-up, constant CL, load factor >> 1, rho 1.225, no speed build-up, no stall, no trim, S = wing outline area"] for b in builds for c in PULLOUT_CLS])


# =============================================================================================== HTML
def tag(s):
    return f' <span class="tag">[{s}]</span>'


def page_html(out: Path, geoms, res, scans, effects, yaw, builds, props, files, pdf_name, svg_names, avl_ok=True):
    h = html.escape
    P = [f"<!doctype html><html lang='en'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'><title>Bird-style flat glider outlines</title>"
         f"<style>{ft.CSS}\n.plots{{display:grid;grid-template-columns:repeat(auto-fit,minmax(420px,1fr));gap:12px}}.warn{{background:#2c2418;border:2px solid #e89a55;padding:12px 16px}}table.t td,table.t th{{padding:4px 7px;font-size:13.5px}}"
         f"figure img{{max-width:100%}}</style></head><body><main>"]
    P.append("<h1>First bird-style flat glider outlines</h1>")
    P.append("<p class='note'>Design numbers from an outline-only computer model (AVL) and from arithmetic. Nothing here is a flight result or a prediction that a glider glides or pulls out. Every number carries a tag: GEOMETRY (exact from the outline), AVL (outline-only vortex-lattice model), ARITHMETIC (simple sum, inputs stated), JUDGEMENT (our choice), ASSUMED (a stated guess), CITED (a number from the bird notes, with its source).</p>")
    P.append("<section><h2>1. What this is</h2><p>A one-piece flat sheet, seen from above like a gliding bird: a wing with an inner &quot;arm&quot; panel and an outer swept-back &quot;hand&quot; panel (the line between them, the wrist, is only marked on the sheet), a narrow body strip blended into the wing, and a short fan tail. There is no boom and no separate tailplane.</p>"
             "<p><b>The requirement.</b> The glider is released hanging nose-down from a quad and has to pull out of the dive by itself. A flat symmetric sheet has no nose-up pitching moment at zero lift, so it would not. Here the fan tail is bent <b>trailing-edge up</b> along one straight hinge line (the tail is set nose-down relative to the wing). One sheet, one bend.</p>"
             "<p><b>What the model is.</b> AVL (vortex-lattice) with the wing panels and the tail as a separate surface at incidence minus the bend angle. The body strip is " + BODY_WORD + ". AVL is inviscid and knows only the outline. It says nothing about flat-plate stall, the low Reynolds number, or whether a real build pulls out.</p></section>")
    P.append("<section><h2>2. Two outline families, two sizes</h2><p>BS is swift-like (slender, strongly swept hand, small tail). BC is crow/pigeon-like (broader wing, larger fan tail). Each at 250 mm span (3D printed, 0.6 mm PLA) and 400 mm span (foamboard, 5 mm and 300 g/m&sup2; both ASSUMED).</p><div class='cols'>")
    for fam in FAM_ORDER:
        for sz in ("printed", "foam"):
            nm = svg_names.get((fam, sz))
            if nm:
                P.append(f"<figure><a href='{nm}'><img src='{nm}' alt='{fam} {sz}'></a><figcaption>{fam} {FAMILIES[fam]['label']}, {SIZES[sz]['material']} {SIZES[sz]['span_mm']:g} mm. Click to enlarge.</figcaption></figure>")
    P.append("</div><div class='scroll'><table class='t'><tr><th></th>" + "".join(f"<th>{fam} {sz}</th>" for fam in FAM_ORDER for sz in ("printed", "foam")) + "</tr>")

    def row(label, fn, src):
        P.append(f"<tr><td>{label}{tag(src)}</td>" + "".join(f"<td class='n'>{fn(geoms[fam], SIZES[sz]['span_mm'], sz)}</td>" for fam in FAM_ORDER for sz in ("printed", "foam")) + "</tr>")
    row("Span, mm", lambda g, k, sz: f"{k:g}", "GEOMETRY")
    row("Wing area, mm&sup2;", lambda g, k, sz: f"{g['S'] * k * k:,.0f}", "GEOMETRY")
    row("Aspect ratio (b&sup2;/S)", lambda g, k, sz: f"{g['ar']:.2f}", "GEOMETRY")
    row("Mean aerodynamic chord, mm", lambda g, k, sz: f"{g['mac'] * k:.1f}", "GEOMETRY")
    row("Root chord at the centreline, mm", lambda g, k, sz: f"{g['c_root'] * k:.1f}", "GEOMETRY")
    row("Wrist at fraction of semi-span", lambda g, k, sz: f"{g['y_wrist'] / g['half_span']:.2f}", "GEOMETRY")
    row("Hand-panel leading-edge sweep, deg", lambda g, k, sz: f"{g['p']['hand_sweep_deg']:g}", "JUDGEMENT")
    row("Tail length behind the wing, mm", lambda g, k, sz: f"{g['Lt'] * k:.1f}", "GEOMETRY")
    row("Tail spread, mm", lambda g, k, sz: f"{g['St'] * k:.1f}", "GEOMETRY")
    row("Hinge line, mm aft of the root leading edge", lambda g, k, sz: f"{g['x_h'] * k:.1f}", "GEOMETRY")
    row("Tail area / wing area", lambda g, k, sz: f"{g['tail_area'] / g['S']:.2f}", "GEOMETRY")
    row("Thickness / MAC, %", lambda g, k, sz: f"{100 * SIZES[sz]['thickness_mm'] / (g['mac'] * k):.2f}", "GEOMETRY; foam thickness ASSUMED")
    P.append("</table></div></section>")
    P.append("<section><h2>3. What came from bird data and what is engineering judgement</h2><div class='box'><h3>From the bird notes (CITED)</h3><ul>"
             "<li>BS aspect ratio 9.5: common swift b&sup2;/S = 9.5 (paper value 9.8), from Henningsson &amp; Hedenstr&ouml;m 2011 [R2].</li>"
             "<li>BS tail spread 0.23 of the span: the swift's tail span was 0.09 m at 7 m/s against a 0.39 m wing span [R2] (ARITHMETIC 0.09/0.39); the bird furls it to 0.04 m at 11 m/s.</li>"
             "<li>BC aspect ratio 5.0: magpie 4.97 and jackdaw about 6, from the bird notes table [R9, R2].</li></ul>"
             "<h3>Engineering judgement (JUDGEMENT)</h3><ul><li>Everything else: arm and hand split, sweeps, chord distribution along the hand, tip shape, body width, nose length, tail length, tail root width, tail end rounding, the bend angles tried (0, 5, 10, 15 deg, set by the brief), the fin area, the tip-bend line and angles, the build-selection rule.</li>"
             "<li>The notes do not give arm/hand proportions, and say that sweep costs lift-to-drag ratio on simple plates (McArthur 2008, Re 15,000) and that outer-wing sweep is the part with a stated yaw-stability rationale (Sachs). They also say gliding birds with measured shape were not passively stable and that the bird tail is a lifting surface, not shown to be what stabilises the bird. This design does not rely on either claim: the bent tail is an engineering fix for the zero-lift moment.</li></ul></div>"
             "<p>Full list with tags: <a href='dimensions.csv'>dimensions.csv</a>.</p></section>")
    # ---- AVL tables
    P.append("<section><h2>4. AVL results by tail bend</h2><p>Tail incidence = minus the bend angle (trailing edge up). Reference point for CM: the quarter chord of the wing's mean aerodynamic chord. CM0 is the pitching moment at zero lift, nose-up positive" + tag("AVL") + ". The model is linear, so numbers are the same at both sizes in % MAC; millimetres scale with the span. Static margin at trim = CM0 / CL (in MAC units); the trim CG is that far ahead of the neutral point" + tag("AVL") + ".</p>")
    for fam in FAM_ORDER:
        g = geoms[fam]
        P.append(f"<h3>{fam} {FAMILIES[fam]['label']} (MAC {g['mac'] * 250:.1f} mm at 250 mm, {g['mac'] * 400:.1f} mm at 400 mm)</h3><div class='scroll'><table class='t'><tr><th>tail bend, deg (trailing edge up)</th><th>neutral point, % MAC</th><th>neutral point, mm aft of root LE (250 / 400)</th><th>CM0 (zero lift)</th>"
                 + "".join(f"<th>trim CG at CL {c:g}, mm (250 / 400)</th><th>static margin, % MAC</th>" for c in TRIM_CLS) + "<th>Cn_beta /rad, no fin (CL 0.3)</th><th>Cl_beta /rad</th></tr>")
        for b in TAIL_BENDS:
            r = res[(fam, b)]
            cn = f"{r['cn_beta']:+.4f}" + (" <b>(near zero)</b>" if abs(r["cn_beta"]) < CN_BETA_NEAR_ZERO else "") + (" <b>(negative)</b>" if r["cn_beta"] < 0 else "")
            P.append(f"<tr><td class='n'>{b:g}</td><td class='n'>{r['np_pct_mac']:.1f}</td><td class='n'>{r['np_span'] * 250:.1f} / {r['np_span'] * 400:.1f}</td><td class='n'>{r['cm0']:+.4f}</td>"
                     + "".join(f"<td class='n'>{r['trim'][c]['x_cg_span'] * 250:.1f} / {r['trim'][c]['x_cg_span'] * 400:.1f}</td><td class='n'>{100 * r['trim'][c]['static_margin']:.1f}</td>" for c in TRIM_CLS)
                     + f"<td class='n'>{cn}</td><td class='n'>{r['cl_beta']:+.4f}</td></tr>")
        P.append("</table></div>")
    P.append("<p>Reading it: with no bend there is no nose-up moment at zero lift (CM0 is zero), so the sheet would not pull out of a vertical dive. Each degree of bend adds nose-up moment, and the trim CG moves ahead of the neutral point. A negative trim-CG number means that trim lies ahead of the wing root leading edge; the existing rails cannot carry ballast there.</p></section>")
    # ---- scan
    P.append("<section><h2>5. Which outline choices buy the most nose-up moment and margin per tail area</h2><p>A small scan at a " + f"{REF_BEND:g} deg bend: tail length x {', '.join(f'{v:g}' for v in SCAN_LEVELS)}, tail spread x {', '.join(f'{v:g}' for v in SCAN_LEVELS)}, hand sweep {', '.join(f'{v:+g}' for v in SCAN_SWEEP_DELTAS)} deg ({len(SCAN_LEVELS) ** 2 * len(SCAN_SWEEP_DELTAS)} outlines per family)" + tag("AVL") + ". Each row averages the scan over one level of one factor. Per tail area = divided by tail area / wing area. Full scan in <a href='scan.csv'>scan.csv</a>.</p>")
    for fam in FAM_ORDER:
        P.append(f"<h3>{fam}</h3><div class='scroll'><table class='t'><tr><th>factor</th><th>level</th><th>mean CM0</th><th>mean static margin at CL 0.3, %</th><th>mean tail area / wing area</th><th>CM0 per tail area</th><th>SM per tail area, %</th><th>mean neutral point, % MAC</th><th>mean Cn_beta /rad</th></tr>")
        for e in effects[fam]:
            P.append(f"<tr><td>{e['factor']}</td><td class='n'>{e['level']:g}</td><td class='n'>{e['cm0']:.4f}</td><td class='n'>{100 * e['sm_cl03']:.1f}</td><td class='n'>{e['tail_area_frac']:.3f}</td><td class='n'>{e['cm0_per_tail_area']:.3f}</td><td class='n'>{100 * e['sm_per_tail_area']:.1f}</td><td class='n'>{e['np_pct_mac']:.1f}</td><td class='n'>{e['cn_beta']:+.4f}</td></tr>")
        P.append("</table></div>")
    P.append("</section>")
    # ---- yaw
    P.append("<section><h2>6. Yaw stability without a fin, and two possible fixes</h2><p>Cn_beta is how strongly the sheet turns back into the wind when it yaws; Cl_beta is the roll tendency in sideslip (negative is the stable sign for a wing). Without a fin both are small" + tag("AVL") + ". Each fix below is evaluated separately at a " + f"{REF_BEND:g} deg tail bend. No recommendation is made here; these are the numbers.</p>")
    for fam in FAM_ORDER:
        P.append(f"<h3>{fam}</h3><div class='scroll'><table class='t'><tr><th>case</th><th>detail</th><th>Cn_beta /rad</th><th>Cl_beta /rad</th><th>neutral point, % MAC</th><th>CM0</th></tr>")
        for r in yaw[fam]:
            P.append(f"<tr><td>{r['fix']}</td><td>{h(r['detail'])}</td><td class='n'>{r['cn_beta']:+.4f}</td><td class='n'>{r['cl_beta']:+.4f}</td><td class='n'>{r['np_pct_mac']:.1f}</td><td class='n'>{r['cm0']:+.4f}</td></tr>")
        P.append("</table></div>")
    P.append("</section>")
    # ---- builds
    P.append("<section><h2>7. Proposed builds, masses and pull-out arithmetic</h2><p class='note'>A proposal for Kai, not a decision. Rule (JUDGEMENT): tail bends above 0 whose CL 0.3 trim CG lies at or behind the wing root leading edge (reachable with the existing rails' nose ballast), ranked by how close the CL 0.3 static margin is to the middle of " + f"{100 * SM_BAND[0]:g} to {100 * SM_BAND[1]:g}%" + "; up to two. The same flat sheet serves any bend: the tail is bent by hand to the angle gauge.</p>")
    for fam in FAM_ORDER:
        P.append(f"<p><b>{fam}:</b> proposed tail bends {', '.join(f'{b:g}' for b in props[fam])} deg. " + ("The static margin at CL 0.3 for the bends tried: " + "; ".join(f"{b:g} deg: {100 * res[(fam, b)]['trim'][0.3]['static_margin']:.0f}% MAC, trim CG {res[(fam, b)]['trim'][0.3]['x_cg_span'] * 250:.0f} mm at 250" for b in TAIL_BENDS if b > 0) + ".</p>"))
    P.append("<div class='scroll'><table class='t'><tr><th>build</th><th>bend</th><th>sheet mass, g</th><th>existing rail</th><th>rail, g</th><th>bare CG, mm</th><th>trim CG (CL 0.3), mm</th><th>ballast needed, g</th><th>all-up, g</th><th>wing loading, N/m&sup2;</th><th>pull-out radius CL 0.3 / 0.5, m</th><th>rail saddle fits ahead of the hinge?</th><th>rail ticks cover the trim CGs?</th></tr>")
    for b in builds:
        bal = f"{b['ballast_g']:.1f}" if b["ballast_reachable"] else f"<b>none reaches it</b> ({b['ballast_raw']:.1f})"
        P.append(f"<tr><td>{b['fam']} {b['size']} {b['span']:g} mm</td><td class='n'>{b['bend']:g}</td><td class='n'>{b['m_sheet']:.1f}{tag('ARITHMETIC')}</td><td>{b['rail']['name']}</td><td class='n'>{b['rail']['mass_g']:.1f}</td><td class='n'>{b['cg0']:.1f}</td><td class='n'>{b['cg_target']:.1f}{tag('AVL')}</td><td class='n'>{bal}</td><td class='n'>{b['M']:.1f}</td><td class='n'>{b['ws']:.1f}</td><td class='n'>{b['pull'][0.3]:.1f} / {b['pull'][0.5]:.1f}{tag('ARITHMETIC')}</td><td>{'yes' if b['rail_fits_under_hinge'] else '<b>no</b>: saddle ends at ' + format(b['rail']['saddle_end'], '.0f') + ' mm, hinge at ' + format(b['hinge_x_mm'], '.0f') + ' mm'}</td><td>{'yes' if b['rail_covers_trim'] else '<b>no</b>'}</td></tr>")
    P.append("</table></div><p>Sheet mass: printed = mesh volume x PLA density, foamboard = area x areal mass (both densities ASSUMED, read from flat_test.py). Rail mass from the existing rail mesh. Ballast is added at the rail's default nose hole to bring the CG to the CL 0.3 trim point. "
             "Pull-out radius: R = 2 (W/S) / (rho g CL), the minimum radius of a steady circular pull-up at constant CL with load factor much greater than 1, air density 1.225 (ASSUMED), S the wing outline area; it ignores speed build-up in the dive, stall and trim, and says nothing about whether the glider pulls out" + tag("ARITHMETIC") + ".</p>")
    P.append("<h3>Printed CG rails</h3><p>The existing rails are 104 mm (250 mm span), 78 mm and 166 mm (400 mm span). Their 0 mark sits at the <b>wing root leading edge at the centreline</b>, and so does the 0 mark here. The rail saddle runs aft from 0 under the body strip for one root chord of the rail's own wing (74 mm, 56 mm, 119 mm); it must stop ahead of the tail hinge or it will stiffen the bend. Where the table says no, cut the rail's rear end back at the hinge, or print a shorter rail; the tick range of the existing rails also may not cover the new trim CGs.</p></section>")
    # ---- files
    P.append("<section><h2>8. Files</h2><ul>" + "".join(f"<li>{x}</li>" for x in files) + "</ul>"
             + (f"<p>Print templates for the foamboard size (1:1, US Letter, white): <a href='{pdf_name}'>{pdf_name}</a> (made with <code>flat_templates_pdf.py</code>). Each page has a 100 mm scale bar; check it with a ruler before cutting.</p>" if pdf_name else "") + "</section>")
    P.append("<section><h2>9. What to try at the field</h2><ol>"
             "<li>Weigh the finished glider and measure the CG on a knife edge. Put the CG at one of the AVL trim points (the green ticks) with the rail ballast. These are reference points, not a window.</li>"
             "<li>Make the tail bend with the angle gauge. Check both halves of the sheet are flat and the tail is not twisted.</li>"
             "<li>Hand-glide first, level, at a gentle speed. Note whether the nose rises or falls.</li>"
             "<li>Drone drop from 10 m, released hanging nose-down, with the same method each time. Record the pull-out height lost if you can see it on video.</li>"
             "<li>Try the other proposed bend, then the other outline, changing one thing at a time. Note roll-offs and yaw wander separately: Cn_beta and Cl_beta here are small without a fin.</li>"
             "<li>If it yaws or rolls, the two fixes in section 6 (a small ventral fin under the tail, or bent wing tips) are the first things to try, one at a time.</li></ol>"
             "<h3>What is not known</h3><ul><li>Whether a flat plate sheet at these Reynolds numbers (about 12,000 to 31,000) pulls out of a dive or glides. The published plate data stop at Reynolds number 60,000 and were for unswept rectangles.</li>"
             "<li>Whether the bent tail still works after the dive speed builds up, or stalls.</li><li>Whether a printed hinge strip holds the bend; foam creases are easier.</li><li>Static pitch stability is not the same as a steady glide.</li></ul></section>")
    P.append("</main></body></html>")
    (out / "index.html").write_text("\n".join(P), encoding="utf-8")


# =============================================================================================== main
def run(out: Path = OUT_DEFAULT, avl=True, pdf=True):
    out.mkdir(parents=True, exist_ok=True)
    geoms = {fam: family_geometry(fam) for fam in FAM_ORDER}
    workdir = out / "avl_work"
    res = avl_all(geoms, workdir)
    scans, effects, yaw = {}, {}, {}
    for fam in FAM_ORDER:
        scans[fam] = scan(fam, 250.0, workdir)
        effects[fam] = main_effects(scans[fam])
        yaw[fam] = yaw_fixes(fam, geoms[fam], 250.0, workdir, res[(fam, REF_BEND)])
    props = {fam: propose_bends(fam, res, geoms) for fam in FAM_ORDER}
    builds = []
    svg_names = {}
    files = []
    pdf_pages = []
    for fam in FAM_ORDER:
        g = geoms[fam]
        for sz in ("printed", "foam"):
            sz_d = SIZES[sz]
            k = sz_d["span_mm"]
            folder = out / f"{fam}_{sz}_span{int(k)}"
            folder.mkdir(parents=True, exist_ok=True)
            bends = props[fam]
            rb = {b: res[(fam, b)] for b in TAIL_BENDS}
            for b in bends:
                builds.append(build_numbers(fam, sz, b, g, rb[b]))
            marks = [(f"trim CG CL0.3 bend {b:g}", rb[b]["trim"][0.3]["x_cg_span"] * k) for b in bends]
            write_dxf_bird(folder / "parts.dxf", g, k, marks)
            (folder / "drawing.svg").write_text(drawing_svg_bird(g, k, f"{fam} {FAMILIES[fam]['label']}", bends, rb, None, sz), encoding="utf-8")
            svg_names[(fam, sz)] = f"{folder.name}/drawing.svg"
            files.append(f"{fam} {sz_d['material']} {k:g} mm: <a href='{folder.name}/drawing.svg'>drawing.svg</a>, <a href='{folder.name}/parts.dxf'>parts.dxf</a>")
            if sz == "printed":
                shells, polys = printed_meshes(g, k, sz_d["thickness_mm"])
                tris = [t for s_ in shells.values() for t in s_]
                ft.write_stl(folder / "sheet.stl", tris, f"{fam} bird sheet (3 shells)")
                for nm, s_ in shells.items():
                    rpt = ft.edge_report(s_, nd=4)
                    assert rpt[1:] == (0, 0, 0), (fam, nm, rpt)
                files[-1] += f", <a href='{folder.name}/sheet.stl'>sheet.stl</a> (flat, scored hinge: one {HINGE_THICK_MM:g} mm strip, {HINGE_WIDTH_MM:g} mm wide; three overlapping closed shells)"
            else:
                pages = template_pages(fam, g, k, bends, rb, f"{fam} {FAMILIES[fam]['label']}")
                names = []
                for i, svg in enumerate(pages):
                    nm = f"template_page{i + 1}.svg"
                    (folder / nm).write_text(svg, encoding="utf-8")
                    names.append(folder / nm)
                    pdf_pages.append(folder / nm)
                files[-1] += ", templates: " + ", ".join(f"<a href='{folder.name}/{p.name}'>{p.name}</a>" for p in names)
    pdf_name = None
    if pdf and pdf_pages:
        pdf_name = "bird_foam_templates.pdf"
        subprocess.run([sys.executable, "-B", str(ROOT / "rigid_glider/flat_templates_pdf.py"), str(out / pdf_name)] + [str(p) for p in pdf_pages], check=True)
    write_tables(out, geoms, res, scans, effects, yaw, builds)
    page_html(out, geoms, res, scans, effects, yaw, builds, props, files, pdf_name, svg_names)
    return dict(geoms=geoms, res=res, scans=scans, effects=effects, yaw=yaw, props=props, builds=builds, pdf=pdf_name, pdf_pages=pdf_pages)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=OUT_DEFAULT)
    ap.add_argument("--no-pdf", action="store_true")
    a = ap.parse_args()
    R = run(a.out, True, not a.no_pdf)
    for b in R["builds"]:
        print(b["fam"], b["size"], b["bend"], f"M {b['M']:.1f} g, W/S {b['ws']:.1f}, R {b['pull'][0.3]:.1f}/{b['pull'][0.5]:.1f} m, ballast {b['ballast_raw']:.1f}")
    print("PDF:", R["pdf"], "->", a.out)


if __name__ == "__main__":
    main()
