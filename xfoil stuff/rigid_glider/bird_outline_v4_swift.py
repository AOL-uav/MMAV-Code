"""Swift outline v4: a flat sheet that follows the real swift's GLIDE pose, with every departure from the bird forced by a named constraint of ours and costed.

Data: research/birds/planform_and_gliding.md (swift proportions measured from Lentink & de Kat 2014 figures, about 5-10 % error [digitised]; Henningsson & Hedenstrom 2011 for
the tail width; the arm/hand split is an ESTIMATE in that file).  Model: AVL (inviscid, outline only, body ignored), tail bent up on one hinge, exactly as in bird_outline_v2, whose
helpers are imported READ-ONLY (v2/v3 outputs and scripts are never touched).  Writes only to results/runs/rigid_glider/bird_outline_v4_swift/.

Outlines:  A "as measured" (30 deg sweep setting, the glide pose),  B "minimum change" (smallest departures from A that meet the thresholds),  C "extended" (5 deg setting).
    py -B rigid_glider/bird_outline_v4_swift.py [--out DIR] [--no-pdf] [--quick]
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
import re
import subprocess
from pathlib import Path

import bird_outline as bo
import bird_outline_v2 as v2
import flat_test as ft

ROOT = ft.ROOT
OUT_DEFAULT = ROOT / "results/runs/rigid_glider/bird_outline_v4_swift"
SIZES = v2.SIZES
SPAN = 250.0                         # AVL is run at 250 mm; coefficients and fractions are scale-free, mm values scale with the span
BEND_REF = 5.0
DY_MIN = v2.DY_MIN
MIN_CHORD_FRAC = 0.030               # JUDGEMENT: minimum buildable chord = 3 % of the span (7.5 mm at 250 mm, 12 mm at 400 mm); Kai has not confirmed one
BEND_MAX = 10.0                      # deg: the largest tail bend v2 allowed in its search (JUDGEMENT)
SM_TARGET = 0.10                     # the static margin the ballast / trim CG are evaluated at (middle of the 8-15 % window)

# ---- the bird data used (research/birds/planform_and_gliding.md); all [digitised] unless stated; position = fraction of the wing from the body edge (0) to the tip (1)
MEASURED_EXT = ((0.04, 1.48), (0.10, 1.42), (0.20, 1.29), (0.30, 1.27), (0.40, 1.20), (0.50, 1.13), (0.60, 0.96), (0.70, 0.84), (0.80, 0.69), (0.90, 0.45), (0.96, 0.28))     # 5 deg setting, chord / mean chord
MEASURED_30 = ((0.10, 1.32), (0.30, 1.17), (0.50, 1.02), (0.70, 0.83), (0.80, 0.70), (0.90, 0.56), (0.95, 0.44))                                                            # 30 deg setting, along the flight direction
ELLIPSE = ((0.04, 1.27), (0.10, 1.27), (0.20, 1.25), (0.30, 1.21), (0.40, 1.17), (0.50, 1.10), (0.60, 1.02), (0.70, 0.91), (0.80, 0.76), (0.90, 0.55), (0.96, 0.36))             # exact ellipse [my calc] in the file
SWEEP_SETTINGS = {"5": dict(span=1.00, area=1.00), "30": dict(span=0.89, area=0.94), "50": dict(span=0.66, area=0.84)}

PARAM_SOURCE = {   # parameter -> (tag, where it comes from)
    "ar_target": ("DIGITISED", "8.5 at the 30 deg setting, 10.1 extended [my calc in the data file]; applied here to the two wings alone, body strip excluded (the data file quantity); with the body strip the aspect ratio is about 0.6 higher"),
    "body_half": ("CALCULATED", "body width about 0.10 of span from the frontal area (Henningsson 2014), so half width 0.05 b"),
    "wrist_eta": ("ESTIMATE", "wrist at 0.20-0.25 of the half span from the centreline (arm 0.10-0.15 of the half span after a body of 0.10); NOT a published number"),
    "chord_knots": ("DIGITISED", "chord / mean chord from Lentink & de Kat 2014 fig 4A (extended) and fig 2 (30 deg), plus 5-10 % error"),
    "min_chord": ("JUDGEMENT", "minimum buildable chord 3 % of span; not confirmed by Kai"),
    "le_arm_deg": ("DIGITISED", "inner leading edge 11 deg back at 30 deg setting, 9 deg forward extended"),
    "le_hand_knots": ("DIGITISED", "37 deg over 0.5-0.9 of the wing and 44 deg over the last 25 % at 30 deg setting; 6 deg rising to 17 deg and 26 deg extended"),
    "tail_len": ("CITED (secondary)", "longest tail feather 77-91 mm = 0.19-0.23 of span, seen in a search excerpt only; fork depth NOT FOUND"),
    "tail_spread": ("CITED", "0.23 of span at 7 m/s (Henningsson 2011), 0.10 furled at 11 m/s"),
    "tail_neck": ("JUDGEMENT", "tail base as wide as the body (0.10 b); structural minimum from v2 arithmetic"),
    "hinge_gap": ("JUDGEMENT", "tail hinge distance behind the wing trailing edge at the neck"),
    "nose": ("JUDGEMENT", "no data on the head / neck length ahead of the shoulder"),
}

GLIDE = dict(   # outline A: the swift glide pose (about the 30 deg setting)
    ar_target=8.5, body_half=0.05, nose=0.09, blend=1.0, wrist_eta=0.139,
    chord_knots=((0.0, 1.32), (0.10, 1.32), (0.30, 1.17), (0.50, 1.02), (0.70, 0.83), (0.80, 0.70), (0.90, 0.56), (0.95, 0.44), (1.0, 0.30)), min_chord=MIN_CHORD_FRAC,
    le_arm_deg=11.0, le_hand_knots=((0.50, 37.0), (0.75, 37.0), (1.0, 44.0)),
    tail_len=0.21, tail_spread=0.23, tail_neck=0.10, tail_bulge=0.15, hinge_gap=0.0)
EXTENDED = dict(  # outline C: fully extended (5 deg setting)
    GLIDE, ar_target=10.1, wrist_eta=0.139,
    chord_knots=((0.0, 1.48), (0.04, 1.48), (0.10, 1.42), (0.20, 1.29), (0.30, 1.27), (0.40, 1.20), (0.50, 1.13), (0.60, 0.96), (0.70, 0.84), (0.80, 0.69), (0.90, 0.45), (0.96, 0.28), (1.0, 0.20)),
    le_arm_deg=-9.0, le_hand_knots=((0.50, 6.0), (0.90, 17.0), (1.0, 40.0)))


# =============================================================================================== geometry
def _interp_knots(knots, x):
    ks = sorted(knots)
    if x <= ks[0][0]:
        return ks[0][1]
    for (x0, y0), (x1, y1) in zip(ks, ks[1:]):
        if x <= x1:
            return y0 + (x - x0) / (x1 - x0) * (y1 - y0)
    return ks[-1][1]


def le_angle_deg(eta, p):
    """Leading-edge sweep angle (deg, positive = swept back) at position eta along the wing.  Arm angle up to the wrist, then a linear ramp through the hand knots."""
    ew = p["wrist_eta"]
    if eta <= ew:
        return p["le_arm_deg"]
    kn = [(ew, p["le_arm_deg"])] + [k for k in p["le_hand_knots"] if k[0] > ew]
    return _interp_knots(kn, eta)


def build_geometry4(p: dict, b: float = 1.0, bend_deg: float = 0.0):
    """Flat-sheet outline from the v4 recipe.  Same dictionary keys as bird_outline_v2.build_geometry2 so the v2/v1 helpers (AVL deck, meshes, DXF) can be reused.
    Units: span b = 1; x aft from the root leading edge; y from the centreline."""
    s = b / 2
    yb = p["body_half"] * b
    xn = -p["nose"] * b
    ew = p["wrist_eta"]
    yw = yb + ew * (s - yb)
    cmin = p["min_chord"] * b
    wt0, Lt, St, bulge_f = p["tail_neck"] * b, p["tail_len"] * b, p["tail_spread"] * b, p["tail_bulge"]
    kn = sorted(p["chord_knots"])
    fpch = v2._pchip([k[0] for k in kn], [k[1] for k in kn])
    # leading edge by integration of tan(angle) along y
    nint = 600
    ysamp = [yb + (s - yb) * i / nint for i in range(nint + 1)]
    xs = [0.0]
    for i in range(1, nint + 1):
        e0, e1 = (ysamp[i - 1] - yb) / (s - yb), (ysamp[i] - yb) / (s - yb)
        t0, t1 = math.tan(math.radians(le_angle_deg(e0, p))), math.tan(math.radians(le_angle_deg(e1, p)))
        xs.append(xs[-1] + 0.5 * (t0 + t1) * (ysamp[i] - ysamp[i - 1]))

    def le_x(y):
        if y <= yb:
            return 0.0
        e = (y - yb) / (s - yb) * nint
        i = min(int(e), nint - 1)
        return xs[i] + (e - i) * (xs[i + 1] - xs[i])

    def make_nodes():
        ys = {0.0, wt0 / 2, yb, yw, s}
        for k in range(1, 21):
            ys.add(yb + (s - yb) * k / 20)
        for e, _ in kn:
            ys.add(yb + e * (s - yb))
        ys = sorted(y for y in ys if 0 <= y <= s + 1e-12)
        must = {0.0, wt0 / 2, yb, yw, s}
        kept = [ys[0]]
        for y in ys[1:]:
            if y in must or y - kept[-1] >= DY_MIN:
                if kept[-1] not in must and y - kept[-1] < DY_MIN:
                    kept.pop()
                kept.append(y)
        return kept
    ys = make_nodes()

    def chord_unit(y):          # chord / cbar
        return max(fpch((y - yb) / (s - yb)), 0.0)

    def nodes_for(cbar):
        les, tes = [], []
        for y in ys:
            if y <= yb:
                les.append(0.0)
                tes.append(max(chord_unit(yb), 0.0) * cbar if cbar * chord_unit(yb) > cmin else cmin)
            else:
                c = max(chord_unit(y) * cbar, cmin)
                les.append(le_x(y))
                tes.append(le_x(y) + c)
        return les, tes

    # ar_target is the aspect ratio of the two wings ALONE: (b - 2 y_body)^2 / (S - body strip), the quantity the data file gives.  g["ar"] is b^2 / S with the body strip included.
    def pair_area(cbar_):
        les_, tes_ = nodes_for(cbar_)
        return 2 * (bo._chord_integrals(ys, les_, tes_)[0] - yb * (tes_[0] - les_[0]))
    target = (b - 2 * yb) ** 2 / p["ar_target"]
    lo, hi = 0.005 * b, 2.0 * b
    for _ in range(80):
        mid = (lo + hi) / 2
        if pair_area(mid) < target:
            lo = mid
        else:
            hi = mid
    cbar = (lo + hi) / 2
    les, tes = nodes_for(cbar)
    a_half, c2, xcint, _ = bo._chord_integrals(ys, les, tes)
    S, mac, x_le_mac = 2 * a_half, c2 / a_half, xcint / a_half
    c_root = tes[0] - les[0]
    i_n = ys.index(wt0 / 2)
    te_n = tes[i_n]
    x_h = te_n + p["hinge_gap"] * b
    # ---- sheet outline (right half): nose -> blend -> leading edge -> tip -> trailing edge -> neck -> fan tail
    yB = yb * (1 + 1.0 * p["blend"])
    p3 = (le_x(yB), yB)
    dx = le_x(yB + 1e-4) - le_x(yB)
    nrm = math.hypot(dx, 1e-4)
    L03 = math.hypot(p3[0] - xn, p3[1])
    nose_chain = v2._bezier((xn, 0.0), (xn, 0.55 * yB), (p3[0] - 0.45 * L03 * dx / nrm, p3[1] - 0.45 * L03 * 1e-4 / nrm), p3, 10)
    le_pts = [(x, y) for y, x in zip(ys, les) if y > yB + 1e-12]
    te_pts = [(x, y) for y, x in zip(ys, tes) if y > wt0 / 2 + 1e-12]
    ex = x_h + Lt + 1.5 * bulge_f * Lt
    R = nose_chain + le_pts + list(reversed(te_pts)) + [(te_n, wt0 / 2)]
    if x_h > te_n + 1e-9:
        R += [(x_h, wt0 / 2)]
    i_junction = len(R) - 1
    R += [(x_h + Lt, St / 2), (x_h + Lt + bulge_f * Lt, St / 4), (ex, 0.0)]
    full = v2.simplify(ft.orient_ccw(R + [(x, -y) for (x, y) in reversed(R[1:-1])]))
    main_R = R[:i_junction + 1]
    main_full = v2.simplify(ft.orient_ccw(main_R + [(x, -y) for (x, y) in reversed(main_R[1:])]))
    wing_poly_half = [(0.0, 0.0)] + [(x, y) for y, x in zip(ys, les) if y > 0] + list(reversed([(x, y) for y, x in zip(ys, tes) if y > 0])) + [(tes[0], 0.0)]
    tail_half = [(x_h, wt0 / 2), (x_h + Lt, St / 2), (x_h + Lt + bulge_f * Lt, St / 4), (ex, 0.0), (x_h, 0.0)]
    tail_area = 2 * abs(ft.area_signed(tail_half))
    side = lambda x: wt0 / 2 + (x - x_h) / Lt * (St / 2 - wt0 / 2)
    i_w = ys.index(yw)
    g = dict(p=p, b=b, c_root=c_root, x_h=x_h, c_wrist=tes[i_w] - les[i_w], x_wrist=les[i_w], y_wrist=yw, half_span=s, hand=[], tip_te=[], wt0=wt0, Lt=Lt, St=St, bulge=bulge_f * Lt, tail_end_x=ex,
             y_nodes=ys, le_nodes=les, te_nodes=tes, S=S, S_polygon=2 * abs(ft.area_signed(wing_poly_half)), mac=mac, x_le_mac=x_le_mac, x_ref=x_le_mac + 0.25 * mac, ar=b * b / S,
             half_outline=v2.simplify(R), full_outline=full, main_full=main_full, tail_area=tail_area, sheet_area=abs(ft.area_signed(full)), nose_x=xn, wb=yb, side=side, bend_deg=bend_deg,
             chord_wrist_local=tes[i_w] - les[i_w], n_nodes=len(ys), tail_centroid_x=ft.polygon_centroid(tail_half)[0] if tail_area > 0 else x_h,
             y_body=yb, cbar=cbar, chord_fn=lambda y: max(chord_unit(y) * cbar, cmin) if y > yb else c_root, le_fn=le_x, eta_wrist=ew, te_neck=te_n)
    return g


def chord_over_mean(g, eta):
    """Chord / mean chord of the WINGS (body strip excluded) at position eta along the wing, mean = wing area outside the body strip / its span."""
    yb, s = g["y_body"], g["half_span"]
    y = yb + eta * (s - yb)
    return g["chord_fn"](y) / wing_mean_chord(g)


def wing_mean_chord(g, n=2000):
    yb, s = g["y_body"], g["half_span"]
    h = (s - yb) / n
    return sum(g["chord_fn"](yb + (i + 0.5) * h) for i in range(n)) / n


def area_parts(g):
    """Areas (fractions of the wing area S) of the inner (arm), outer (hand) parts and the body strip, plus tail / body / nose areas in span units."""
    s, yb, yw = g["half_span"], g["y_body"], g["y_wrist"]

    def integ(y0, y1, n=1500):
        h = (y1 - y0) / n
        return 2 * sum(g["chord_fn"](y0 + (i + 0.5) * h) for i in range(n)) * h
    strip, arm, hand = integ(0, yb), integ(yb, yw), integ(yw, s)
    return dict(strip=strip, arm=arm, hand=hand, S=g["S"], tail=g["tail_area"], sheet=g["sheet_area"], body_extra=g["sheet_area"] - g["S"] - g["tail_area"])


TAIL_AREA_FRAC_BIRD = 0.185          # CALCULATED from Henningsson 2011 as quoted in the data file: CL 0.96 on wing area alone vs 0.81 with the spread tail added = 1.185 x the area


def with_tail_area(p, frac=TAIL_AREA_FRAC_BIRD):
    """Return p with tail_len solved so that fan-tail area = frac x wing area (the bird's spread tail adds about 18.5 % to the lifting area at 7 m/s)."""
    lo, hi = 0.02, 0.5
    for _ in range(60):
        mid = (lo + hi) / 2
        g = build_geometry4(dict(p, tail_len=mid))
        if g["tail_area"] / g["S"] < frac:
            lo = mid
        else:
            hi = mid
    return dict(p, tail_len=(lo + hi) / 2)


GLIDE = with_tail_area(GLIDE)
EXTENDED = with_tail_area(EXTENDED)
PARAM_SOURCE["tail_len"] = ("CALCULATED", "solved so the fan tail area is 0.185 x the wing area (Henningsson 2011 lift coefficients as quoted in the data file); the feather length 0.19-0.23 b in a search excerpt includes the fork and the covert overlap")


# =============================================================================================== AVL evaluation and criteria
EXISTING = dict(name="existing (v2)", sm_lo=0.08, sm_hi=0.15, ballast=0.20, cn=0.004, tip="relative to v1", neck="v2 rule", nose_clear=v2.NOSE_CLEAR_SPAN)
PROPOSED = dict(name="proposed alternative (PROPOSED, for Kai)", sm_lo=0.08, sm_hi=0.15, ballast=0.25, cn=0.002, tip="information only", neck="v2 rule", nose_clear=v2.NOSE_CLEAR_SPAN)
N_AVL = {"runs": 0}


def fake_res(cg_span, sm):
    return {"trim": {c: dict(x_cg_span=cg_span, static_margin=sm) for c in v2.TRIM_CLS}}


def evaluate4(name, p, work: Path, v1_tip, quick_panel=False):
    """AVL at bend 0 and 5 (the model is linear in the tail incidence), then arithmetic at static margins 8 % and 10 %."""
    g = build_geometry4(p)
    v2.AVL_NCHORD, v2.AVL_NSPAN = 10, 80
    r0 = v2.run_case2(g, SPAN, 0.0, work / name / "b0", name + "b0")
    r5 = v2.run_case2(g, SPAN, BEND_REF, work / name / "b5", name + "b5")
    N_AVL["runs"] += 2
    mac = g["mac"]
    cm0pd = r5["cm0"] / BEND_REF
    out = dict(name=name, p=p, g=g, np_pct=r5["np_pct_mac"], np_span=r5["np_span"], mac=mac, cm0_per_deg=cm0pd, cn_beta=r5["cn_beta"], cl_beta=r5["cl_beta"], tip_loading=r5["tip_loading"],
               span_eff=r5["span_eff"], cl_alpha=r5["cla_per_rad"], tail_lift_frac=r5["tail_lift_frac"], sm_at_5=r5["cm0"] / 0.3, v1_tip=v1_tip, areas=area_parts(g), r5=r5)
    for s in (0.08, 0.10):
        key = int(round(s * 100))
        cg_span = r5["np_span"] - s * mac
        bend_req = s * 0.3 / cm0pd
        out[f"bend_{key}"] = bend_req
        out[f"cg_mm_{key}"] = cg_span * SPAN
        res = fake_res(cg_span, s)
        for sk in ("printed", "foam"):
            n = v2.size_numbers(g, res, sk, min(bend_req, 99.0))
            out[f"ballast_frac_{sk}_{key}"] = n["ballast_frac"]
            out[f"ballast_g_{sk}_{key}"] = n["ballast_g"]
            out[f"M_{sk}_{key}"] = n["M"]
            out[f"ws_{sk}_{key}"] = n["ws"]
            out[f"reach_{sk}_{key}"] = n["reachable"]
            out[f"bare_cg_{sk}_{key}"] = n["cg0"]
            out[f"sheet_g_{sk}"] = n["m_sheet"]
            out[f"rail_g_{sk}"] = n["rail"]["mass_g"]
        sn = v2.size_numbers(g, res, "printed", min(bend_req, 99.0))
        st = v2.structure_numbers(g, dict(cm0=s * 0.3, m_wrist_over_w_m=r5["m_wrist_over_w_m"]), sn, SPAN, bend_req)
        out[f"neck_ok_{key}"] = st["neck_ok"]
        out[f"neck_min_mm_{key}"] = st["neck_min_mm"]
        out[f"neck_mm"] = st["neck_width_mm"]
        out[f"sigma_neck_{key}"] = st["sigma_neck_MPa"]
        out[f"hinge_strain_{key}"] = st["hinge_strain_pct"]
        out[f"trim_behind_nose_{key}"] = cg_span >= g["nose_x"] + v2.NOSE_CLEAR_SPAN
    out["bare_cg_pct_mac_aft_of_np"] = 100 * (out["bare_cg_printed_10"] / SPAN - r5["np_span"]) / mac
    out["swoop_formula_printed_m"] = 3.0 * out["ws_printed_10"] / (v2.ft.RHO_AIR * 9.80665 * 0.3)
    out["swoop_formula_foam_m"] = 3.0 * out["ws_foam_10"] / (v2.ft.RHO_AIR * 9.80665 * 0.3)
    return out


def criteria4(o, thr):
    """Each criterion as its own number: -> list of (name, value text, limit text, ok)."""
    rows = []
    s_floor = thr["sm_lo"]
    rows.append(("static margin reachable with a bend up to %g deg" % BEND_MAX, f"bend for {100 * s_floor:.0f}% margin = {o['bend_8']:.1f} deg", f"at most {BEND_MAX:g} deg", o["bend_8"] <= BEND_MAX))
    for sk in ("printed", "foam"):
        fr = o[f"ballast_frac_{sk}_8"]
        ok = math.isfinite(fr) and fr < thr["ballast"] and o[f"reach_{sk}_8"]
        rows.append((f"nose weight / all-up mass, {sk}, at an 8% margin", "unreachable" if not math.isfinite(fr) else f"{100 * fr:.1f}%", f"below {100 * thr['ballast']:.0f}%", ok))
    rows.append(("Cn_beta with no fin, per rad", f"{o['cn_beta']:+.4f}", f"at least {thr['cn']}", o["cn_beta"] >= thr["cn"]))
    rows.append(("Cl_beta (stable sign)", f"{o['cl_beta']:+.4f}", "below -0.005", o["cl_beta"] <= v2.CL_BETA_MAX))
    if thr["tip"] == "relative to v1":
        rows.append(("tip loading indicator", f"{o['tip_loading']:.2f}", f"not above v1 {o['v1_tip']:.2f} + 1%", o["tip_loading"] <= o["v1_tip"] * (1 + v2.TIP_TOL)))
    else:
        rows.append(("tip loading indicator", f"{o['tip_loading']:.2f}", "information only (v1 = %.2f)" % o["v1_tip"], True))
    rows.append(("tail neck width vs structural minimum (hinge 0.4 mm, load factor 5)", f"{o['neck_mm']:.1f} mm vs minimum {o['neck_min_mm_8']:.1f} mm", "neck at least the minimum", o["neck_ok_8"]))
    rows.append(("trim CG behind the nose tip", f"{o['cg_mm_8']:.1f} mm vs nose tip {o['g']['nose_x'] * SPAN:.1f} mm", "at least 3 mm behind", o["trim_behind_nose_8"]))
    return rows


def passes(o, thr):
    return all(r[3] for r in criteria4(o, thr))


# =============================================================================================== studies
GAUGE_ANGLES = (2.0, 2.5, 4.0, 5.0, 6.0, 7.5, 8.0)               # the angles on the field-pack gauge
WRIST_ETAS = (0.139, 0.20, 0.30, 0.40)                          # 0.139 = the data file's estimate (0.225 of the half span); 0.2-0.4 = where the leading edge changes slope in the figures
HAND_STEPS = (4.0, 6.0, 7.0, 8.0, 10.0)                         # deg added to the hand leading-edge sweep knots (the search for outline B)
CN_MARGIN = 1.02                                                # outline B must clear the Cn_beta floor by 2 % (the panelling check below moves Cn_beta by under 1 %)


def hand_shift(p, dd):
    return dict(p, le_hand_knots=tuple((e, a + dd) for e, a in p["le_hand_knots"]))


def departures_from_A(A):
    """Single departures from A tried: (key, label, parameter update, what it is for)."""
    return [
        ("hand+4", "hand leading-edge sweep +4 deg", dict(le_hand_knots=hand_shift(A, 4)["le_hand_knots"]), "yaw stiffness"),
        ("hand+6", "hand leading-edge sweep +6 deg", dict(le_hand_knots=hand_shift(A, 6)["le_hand_knots"]), "yaw stiffness"),
        ("hand+7", "hand leading-edge sweep +7 deg", dict(le_hand_knots=hand_shift(A, 7)["le_hand_knots"]), "yaw stiffness"),
        ("hand+8", "hand leading-edge sweep +8 deg", dict(le_hand_knots=hand_shift(A, 8)["le_hand_knots"]), "yaw stiffness"),
        ("hand+10", "hand leading-edge sweep +10 deg", dict(le_hand_knots=hand_shift(A, 10)["le_hand_knots"]), "yaw stiffness"),
        ("arm20", "arm leading-edge sweep 11 -> 20 deg", dict(le_arm_deg=20.0), "yaw stiffness"),
        ("spread.30", "tail spread 0.23 -> 0.30 of span", dict(tail_spread=0.30), "tail authority / yaw"),
        ("tail x1.4", "tail length x 1.4", dict(tail_len=A["tail_len"] * 1.4), "tail authority"),
        ("gap.03", "tail hinge 0.03 b behind the wing trailing edge", dict(hinge_gap=0.03), "tail lever"),
        ("nose.15", "nose 0.09 -> 0.15 of span", dict(nose=0.15), "ballast"),
        ("AR9.5", "aspect ratio of the two wings 8.5 -> 9.5", dict(ar_target=9.5), "ballast / efficiency"),
        ("body.035", "body half width 0.05 -> 0.035 of span", dict(body_half=0.035), "mass"),
    ]


def v2_final_geometry():
    return v2.build_geometry2(v2.feature_params("BS", {"curved_te": 1, "root_blend": 1, "tip_round": 1, "neck": 1, "nose": 1}))


def chord_curve_v2(g2, yb_frac=0.03, n=24):
    """v2 chord / mean chord of the WINGS (y beyond the body strip) at positions eta along the wing, for the overlay with the measured swift."""
    s = g2["half_span"]
    yb = yb_frac
    ys, le, te = g2["y_nodes"], g2["le_nodes"], g2["te_nodes"]

    def c(y):
        for i in range(len(ys) - 1):
            if ys[i] <= y <= ys[i + 1]:
                t = (y - ys[i]) / (ys[i + 1] - ys[i])
                return (te[i] + t * (te[i + 1] - te[i])) - (le[i] + t * (le[i + 1] - le[i]))
        return 0.0
    m = sum(c(yb + (s - yb) * (i + 0.5) / 2000) for i in range(2000)) / 2000
    return [(e, c(yb + e * (s - yb)) / m) for e in [i / n for i in range(n + 1)]], m


def rms_vs(measured, fn):
    d = [(fn(e) - f) for e, f in measured]
    return math.sqrt(sum(x * x for x in d) / len(d))


def panel_check(name, p, work, v1_tip):
    g = build_geometry4(p)
    keep = (v2.AVL_NCHORD, v2.AVL_NSPAN)
    rows = []
    try:
        for nc, ns in ((8, 64), (10, 80), (16, 128)):
            v2.AVL_NCHORD, v2.AVL_NSPAN = nc, ns
            r = v2.run_case2(g, SPAN, BEND_REF, work / f"panel_{name}_{nc}", f"p{nc}")
            N_AVL["runs"] += 1
            rows.append(dict(outline=name, nc=nc, ns=ns, np_pct=r["np_pct_mac"], cm0=r["cm0"], cn=r["cn_beta"], cl=r["cl_beta"], tip=r["tip_loading"]))
    finally:
        v2.AVL_NCHORD, v2.AVL_NSPAN = keep
    return rows


def run_studies(out: Path, quick=False, log=print):
    work = out / "avl"
    v2.AVL_NCHORD, v2.AVL_NSPAN = 10, 80
    g1 = v2.build_geometry2(v2.feature_params("BS", {}))
    v1_tip = v2.run_case2(g1, SPAN, BEND_REF, work / "v1ref", "v1ref")["tip_loading"]
    N_AVL["runs"] += 2
    S = {"v1_tip": v1_tip}
    log("A and C")
    A = evaluate4("A_as_measured", GLIDE, work, v1_tip)
    C = evaluate4("C_extended", EXTENDED, work, v1_tip)
    deps = []
    if not quick:
        for key, label, upd, why in departures_from_A(GLIDE):
            log("  departure " + key)
            deps.append((key, label, why, upd, evaluate4("A_" + key.replace(".", "_").replace(" ", "_"), dict(GLIDE, **upd), work, v1_tip)))
    # outline B: the smallest hand-sweep departure that clears the Cn_beta floor with margin and meets everything else (existing thresholds)
    B_key, B = None, None
    cand = {d[0]: d[4] for d in deps}
    for k in HAND_STEPS:
        key = f"hand+{k:g}"
        o = cand.get(key) or evaluate4("B_" + key.replace("+", "p"), hand_shift(GLIDE, k), work, v1_tip)
        cand[key] = o
        if passes(o, EXISTING) and o["cn_beta"] >= EXISTING["cn"] * CN_MARGIN:
            B_key, B = key, o
            break
    if B is None:
        B_key = "hand+10"
        B = cand.get(B_key) or evaluate4("B_hand10", hand_shift(GLIDE, 10), work, v1_tip)
    B["name"] = "B_hybrid"
    S.update(A=A, C=C, B=B, B_key=B_key, deps=deps)
    wrist = []
    log("wrist sensitivity")
    for nm, o0 in (("A", GLIDE), ("B", B["p"])):
        for we in WRIST_ETAS:
            p = dict(o0, wrist_eta=we)
            o = evaluate4(f"{nm}_wrist{we:g}", p, work, v1_tip)
            wrist.append((nm, we, o))
    S["wrist"] = wrist
    if not quick:
        log("panelling")
        S["panel"] = panel_check("A", GLIDE, work, v1_tip) + panel_check("B", B["p"], work, v1_tip)
    else:
        S["panel"] = []
    return S


# =============================================================================================== drawings
def outline_svg(items, span=SPAN, scale=0.9, title=""):
    """Top views side by side, nose up, same scale.  items = [(label, g, colour)].  Wrist (blue dashed), body edge (purple dashed) and hinge (orange) lines mark the natural fold lines."""
    w_each = span * scale + 50
    ymax = max((g["tail_end_x"] - g["nose_x"]) * span for _, g, _ in items)
    H = ymax * scale + 100
    o = [f"<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 {w_each * len(items):.0f} {H:.0f}' width='{w_each * len(items):.0f}' height='{H:.0f}' style='max-width:100%;background:#14181d;border:1px solid #333c46;border-radius:6px'>"]
    o.append(f"<text x='10' y='20' font-family='Segoe UI,Arial' font-size='13' fill='#dde3ea'>{html.escape(title)}</text>")
    for i, (lab, g, col) in enumerate(items):
        cx = i * w_each + w_each / 2
        y0 = 55 - g["nose_x"] * span * scale
        X = lambda y: cx + y * span * scale
        Y = lambda x: y0 + x * span * scale
        pts = " ".join(f"{X(y):.1f},{Y(x):.1f}" for x, y in g["full_outline"])
        o.append(f"<polygon points='{pts}' fill='#27313c' stroke='{col}' stroke-width='1.6'/>")
        yw, yb = g["y_wrist"], g["y_body"]
        for yy, c_, dash in ((yw, "#7fb4ff", "5 3"), (yb, "#c39bff", "2 3")):
            for sg in (1, -1):
                xl = g["le_fn"](yy)
                xt = xl + g["chord_fn"](yy)
                o.append(f"<line x1='{X(sg * yy):.1f}' y1='{Y(xl):.1f}' x2='{X(sg * yy):.1f}' y2='{Y(xt):.1f}' stroke='{c_}' stroke-width='1.2' stroke-dasharray='{dash}'/>")
        xh = g["x_h"]
        o.append(f"<line x1='{X(-g['wt0'] / 2):.1f}' y1='{Y(xh):.1f}' x2='{X(g['wt0'] / 2):.1f}' y2='{Y(xh):.1f}' stroke='#ff8a5c' stroke-width='2.4'/>")
        o.append(f"<text x='{cx:.0f}' y='{H - 36:.0f}' font-family='Segoe UI,Arial' font-size='12' fill='#dde3ea' text-anchor='middle'>{html.escape(lab)}</text>")
        o.append(f"<text x='{cx:.0f}' y='{H - 20:.0f}' font-family='Segoe UI,Arial' font-size='10.5' fill='#9aa7b4' text-anchor='middle'>wing area {g['S'] * span * span:,.0f} mm2, AR {g['ar']:.1f}, tail {100 * g['tail_area'] / g['S']:.0f}% of it</text>")
    o.append(f"<text x='10' y='{H - 4:.0f}' font-family='Segoe UI,Arial' font-size='10.5' fill='#9aa7b4'>blue dashed = wrist (ESTIMATED position), purple dotted = body edge, orange = tail hinge: the natural fold lines. Scale: span {span:g} mm = {span * scale:.0f} px.</text></svg>")
    return "".join(o)


def chord_plot_svg(curves, w=640, h=380):
    """curves: [(label, colour, points, style)] with points (position along wing, chord / mean chord); style 'line' or 'dots'."""
    L, R, T, B = 52, 12, 24, 44
    X = lambda e: L + e * (w - L - R)
    Y = lambda f: T + (1.7 - f) / 1.7 * (h - T - B)
    o = [f"<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 {w} {h}' width='{w}' height='{h}' style='max-width:100%;background:#14181d;border:1px solid #333c46;border-radius:6px'>"]
    for k in range(0, 18, 2):
        f = k / 10
        o.append(f"<line x1='{L}' y1='{Y(f):.1f}' x2='{w - R}' y2='{Y(f):.1f}' stroke='#262e37'/><text x='{L - 5}' y='{Y(f) + 3:.1f}' font-size='10' fill='#9aa7b4' text-anchor='end' font-family='Segoe UI'>{f:.1f}</text>")
    for k in range(0, 11):
        e = k / 10
        o.append(f"<line x1='{X(e):.1f}' y1='{T}' x2='{X(e):.1f}' y2='{h - B}' stroke='#262e37'/><text x='{X(e):.1f}' y='{h - B + 14}' font-size='10' fill='#9aa7b4' text-anchor='middle' font-family='Segoe UI'>{e:.1f}</text>")
    for i, (lab, col, pts, style) in enumerate(curves):
        if style == "dots":
            o.append(f"<polyline fill='none' stroke='{col}' stroke-width='1' stroke-opacity='0.5' points='{' '.join(f'{X(e):.1f},{Y(f):.1f}' for e, f in pts)}'/>")
            for e, f in pts:
                o.append(f"<circle cx='{X(e):.1f}' cy='{Y(f):.1f}' r='3.2' fill='{col}'/>")
        else:
            o.append(f"<polyline fill='none' stroke='{col}' stroke-width='{2.2 if style == 'line' else 1.6}' stroke-dasharray='{'' if style == 'line' else '5 3'}' points='{' '.join(f'{X(e):.1f},{Y(f):.1f}' for e, f in pts)}'/>")
        o.append(f"<text x='{w - R - 6}' y='{T + 14 + 14 * i}' font-size='11' fill='{col}' text-anchor='end' font-family='Segoe UI'>{html.escape(lab)}</text>")
    o.append(f"<text x='{L + (w - L - R) / 2}' y='{h - 6}' font-size='11' fill='#9aa7b4' text-anchor='middle' font-family='Segoe UI'>position along the wing, body edge (0) to tip (1)</text>")
    o.append(f"<text x='12' y='{T + (h - T - B) / 2}' font-size='11' fill='#9aa7b4' text-anchor='middle' font-family='Segoe UI' transform='rotate(-90 12 {T + (h - T - B) / 2})'>chord / mean chord of the wings</text></svg>")
    return "".join(o)


# =============================================================================================== build files for the recommended outline
def trim_res(o, bend):
    cm0 = o["cm0_per_deg"] * bend
    return {"trim": {c: dict(x_cg_span=o["np_span"] - o["mac"] * cm0 / c, static_margin=cm0 / c) for c in v2.TRIM_CLS}}


def nearest_gauge(bend):
    return min(GAUGE_ANGLES, key=lambda a: abs(a - bend))


def write_dxf4(path: Path, g, span_mm, marks):
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
    for yy, layer in ((g["y_wrist"], "WRIST_MARKER"), (g["y_body"], "BODY_EDGE_MARKER")):
        for sg in (1, -1):
            xl = g["le_fn"](yy)
            line(layer, (xl * k, sg * yy * k), ((xl + g["chord_fn"](yy)) * k, sg * yy * k))
    line("CENTRELINE", (g["nose_x"] * k - 5, 0.0), (g["tail_end_x"] * k + 5, 0.0))
    for lab, x in marks:
        line("CG_MARKS", (x, -4.0), (x, 4.0))
    layers = (("OUTLINE", 7), ("HINGE_BEND_UP", 1), ("WRIST_MARKER", 3), ("BODY_EDGE_MARKER", 6), ("CENTRELINE", 8), ("CG_MARKS", 5))
    path.write_text("0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n0\nSECTION\n2\nTABLES\n0\nTABLE\n2\nLAYER\n70\n6\n"
                    + "".join(f"0\nLAYER\n2\n{l}\n70\n0\n62\n{c}\n6\nCONTINUOUS\n" for l, c in layers) + "0\nENDTAB\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n" + "".join(ent) + "0\nENDSEC\n0\nEOF\n", encoding="ascii")


def add_body_edge_to_svg(svg_text, g, span_mm):
    """Insert the body-edge fold line into a bird_outline.drawing_svg_bird drawing (same coordinate mapping as that function)."""
    s = 760.0 / span_mm
    k = span_mm
    x0, ytop = 400.0, 110.0
    xmin = g["nose_x"] * k
    sx = lambda y: x0 + y * k * s
    sy = lambda x: ytop + (x * k - xmin) * s
    yb = g["y_body"]
    xl = g["le_fn"](yb)
    xt = xl + g["chord_fn"](yb)
    extra = "".join(f'<line x1="{sx(sg * yb):.1f}" y1="{sy(xl):.1f}" x2="{sx(sg * yb):.1f}" y2="{sy(xt):.1f}" stroke="#c39bff" stroke-width="1.5" stroke-dasharray="2 3"/>' for sg in (1, -1))
    extra += f'<text x="{sx(yb) + 5:.1f}" y="{sy(xl) + 14:.1f}" font-family="Segoe UI,Arial" font-size="11" fill="#c39bff">body edge (fold line, marker only)</text>'
    return svg_text.replace("</svg>", extra + "</svg>")


def template_pages4(g, span_mm, label, material, thickness, bends, res_by_bend):
    """The v2 template pages plus the body-edge fold line (the v2 pages already carry the hinge, the wrist marker, the centreline and the CG ticks)."""
    pages = v2.template_pages2(g, span_mm, label, material, thickness, bends, res_by_bend)
    k = span_mm
    xh_mm = g["x_h"] * k
    yb = g["y_body"]
    xl = g["le_fn"](yb) * k
    xt = xl + g["chord_fn"](yb) * k
    out = []
    for i, pg in enumerate(pages):
        m = re.search(r'<line [^>]*x1="([-0-9.]+)" y1="([-0-9.]+)" x2="[-0-9.]+" y2="([-0-9.]+)" stroke="#c2621a" stroke-width="0.7"', pg)
        if not m or "gauge for a" in pg:
            out.append(pg)
            continue
        X1h, Y0 = float(m.group(1)), float(m.group(2))
        Xf = lambda x: X1h + (x - xh_mm)
        Yf = lambda y: Y0 - y
        lo, hi = ft.TEMPLATE_MARGIN + 4.0, ft.LETTER_W - ft.TEMPLATE_MARGIN - 4.0
        xa, xb = Xf(xl), Xf(xt)
        if xb < ft.TEMPLATE_MARGIN or xa > ft.LETTER_W - ft.TEMPLATE_MARGIN:
            out.append(pg)
            continue
        add = f'<line x1="{xa:.3f}" y1="{Yf(yb * k):.3f}" x2="{xb:.3f}" y2="{Yf(yb * k):.3f}" stroke="#7a3fb0" stroke-width="0.35" stroke-dasharray="1 1.5"/>'
        if lo <= xa + 1 and xa + 40 <= hi:
            add += f'<text x="{xa + 1.0:.2f}" y="{Yf(yb * k) + 3.2:.2f}" font-family="Helvetica" font-size="2.4" fill="#7a3fb0">body edge (fold line, marker)</text>'
        mask = '<rect x="0.00" y="0.00" width="10.00"'
        out.append(pg.replace(mask, add + mask, 1) if mask in pg else pg)
    return out


def build_files(o, bend, out: Path, name, make_pdf=True):
    g = o["g"]
    res = trim_res(o, bend)
    info = dict(files=[], builds=[], pages=[], bend=bend)
    cgs_all = {}
    for sk in ("printed", "foam"):
        sz = SIZES[sk]
        k = sz["span_mm"]
        folder = out / f"{name}_{sk}_span{int(k)}"
        folder.mkdir(parents=True, exist_ok=True)
        n = v2.size_numbers(g, res, sk, bend, exact=True)
        rail = n["rail"]
        ft.write_stl(folder / "rail.stl", rail["tris"], f"{name} {sk} CG rail")
        assert ft.edge_report(rail["tris"], nd=4)[1:] == (0, 0, 0)
        marks = [(f"trim CG CL0.3", res["trim"][0.3]["x_cg_span"] * k)]
        write_dxf4(folder / "parts.dxf", g, k, marks)
        svg = bo.drawing_svg_bird(g, k, f"{name} swift v4", [bend], {bend: dict(np_span=o["np_span"], trim=res["trim"])}, None, sk)
        (folder / "drawing.svg").write_text(add_body_edge_to_svg(svg, g, k), encoding="utf-8")
        fl = f"{name} {sz['material']} {k:g} mm: <a href='{folder.name}/drawing.svg'>drawing.svg</a>, <a href='{folder.name}/parts.dxf'>parts.dxf</a>, <a href='{folder.name}/rail.stl'>rail.stl</a>"
        if sk == "printed":
            shells, _ = v2.printed_meshes2(g, k, sz["thickness_mm"])
            tris = [t for s_ in shells.values() for t in s_]
            ft.write_stl(folder / "sheet.stl", tris, f"{name} v4 sheet (3 shells)")
            for nm, s_ in shells.items():
                assert ft.edge_report(s_, nd=4)[1:] == (0, 0, 0), (nm,)
            fl += f", <a href='{folder.name}/sheet.stl'>sheet.stl</a>"
        pages = template_pages4(g, k, f"{name} swift v4 {k:g} mm", sz["material"], sz["thickness_mm"], [bend], {bend: dict(np_span=o["np_span"], trim=res["trim"])})
        names = []
        for i, pgsvg in enumerate(pages):
            nm = f"template_page{i + 1}.svg"
            (folder / nm).write_text(pgsvg, encoding="utf-8")
            names.append(folder / nm)
        info["pages"] += names
        fl += ", templates: " + ", ".join(f"<a href='{folder.name}/{p.name}'>{p.name}</a>" for p in names)
        info["files"].append(fl)
        info["builds"].append(dict(size=sk, span=k, n=n, folder=folder.name))
    if make_pdf and info["pages"]:
        pdf = out / f"{name}_templates.pdf"
        subprocess.run([sys.executable, "-B", str(ROOT / "rigid_glider/flat_templates_pdf.py"), str(pdf)] + [str(p) for p in info["pages"]], check=True, stdout=subprocess.DEVNULL)
        info["pdf"] = pdf.name
    return info


# =============================================================================================== csv and page
def write_csv(path: Path, header, rows):
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def fnum(v, n=3):
    return "" if v is None else (f"{v:.{n}g}" if isinstance(v, float) else str(v))


def summary_row(o):
    g = o["g"]
    pl = area_parts(g)
    return [o["name"], f"{pair_ar(g):.2f}", f"{g['ar']:.2f}", f"{g['S'] * SPAN ** 2:,.0f}", f"{g['c_root'] * SPAN:.1f}", f"{g['chord_fn'](g['y_wrist']) * SPAN:.1f}", f"{g['mac'] * SPAN:.1f}",
            f"{100 * g['y_wrist'] / g['half_span']:.0f}", f"{g['tail_area'] / g['S']:.3f}", f"{pl['body_extra'] * SPAN ** 2:,.0f}", f"{o['np_pct']:.1f}", f"{o['cm0_per_deg']:.5f}", f"{o['bend_8']:.2f}", f"{o['bend_10']:.2f}",
            f"{o['cg_mm_10']:.1f}", f"{100 * o['ballast_frac_printed_8']:.1f}", f"{100 * o['ballast_frac_foam_8']:.1f}", f"{100 * o['ballast_frac_printed_10']:.1f}", f"{100 * o['ballast_frac_foam_10']:.1f}",
            f"{o['cn_beta']:+.5f}", f"{o['cl_beta']:+.5f}", f"{o['tip_loading']:.3f}", f"{o['M_printed_10']:.2f}", f"{o['M_foam_10']:.2f}"]


SUMMARY_HDR = ["outline", "aspect ratio of the two wings alone (body strip excluded)", "aspect ratio with the body strip included (b^2/S)", "wing area mm2 at 250 mm", "root chord mm", "wrist chord mm", "MAC mm", "wrist % of half span", "tail area / wing area", "body (nose+blend) area mm2",
               "NP % MAC", "CM0 per deg of bend", "bend for 8% margin deg", "bend for 10% margin deg", "trim CG mm at 10% margin (250)", "ballast % printed at 8%", "ballast % foam at 8%", "ballast % printed at 10%",
               "ballast % foam at 10%", "Cn_beta no fin", "Cl_beta", "tip loading", "all-up g printed (10% margin)", "all-up g foam (10% margin)"]


def pullout_factors():
    """Sim-to-formula ratios and tail-effectiveness factors from the pull-out study (results/runs/rigid_glider/pullout/cases.csv), or None if the file is absent."""
    path = ROOT / "results/runs/rigid_glider/pullout/cases.csv"
    if not path.is_file():
        return None
    with path.open(encoding="utf-8") as f:
        R = list(csv.DictReader(f))
    med = lambda v: sorted(v)[len(v) // 2] if v else None
    base = [r for r in R if r["tag"] == "base"]
    out = {}
    for key, spec in (("printed", "BS_printed_250"), ("foam", "BS_foam_400")):
        out[f"ratio_{key}"] = med([float(r["depth_over_formula"]) for r in base if r["spec"] == spec and r["depth_over_formula"]])
    flat = [r for r in base if not r["spec"].startswith("BS")]
    bkey = {(r["spec"], r["cg_mm"], r["up_deg"]): r for r in flat}
    for e in ("75%", "50%"):
        v = []
        for r in R:
            if r["tag"] == f"effectiveness {e}" and not r["spec"].startswith("BS") and r["swoop_depth_m"]:
                b0 = bkey.get((r["spec"], r["cg_mm"], r["up_deg"]))
                if b0 and b0["swoop_depth_m"] and b0["verdict"] == r["verdict"] == "SETTLES TO A GLIDE":
                    v.append(float(r["swoop_depth_m"]) / float(b0["swoop_depth_m"]))
        out[f"eff{e[:-1]}"] = med(v)
    out["ratio_flat"] = med([float(r["depth_over_formula"]) for r in flat if r["depth_over_formula"] and r["cannot_say"] == "False"])
    return out


def depth_range(n, fac, kind):
    """First-swoop depth: formula (constant CL, full tail effect) up to formula x sim ratio x 50 % tail effectiveness factor.  -> (formula, low, high) in m, MODEL."""
    f0 = 3.0 * n["ws"] / (ft.RHO_AIR * 9.80665 * 0.3)
    if not fac:
        return f0, f0, None
    hi = f0 * (fac[f"ratio_{kind}"] or 1.0) * (fac["eff50"] or 1.0)
    return f0, f0, hi


def build_page(out: Path, S, infos, quick):
    h = html.escape
    A, B, C = S["A"], S["B"], S["C"]
    g2 = v2_final_geometry()
    c_v2, m_v2 = chord_curve_v2(g2)
    gA, gB, gC = A["g"], B["g"], C["g"]
    MEASURED_50 = ((0.1, 1.19), (0.3, 1.07), (0.5, 0.95), (0.7, 0.89), (0.8, 0.89), (0.9, 0.74), (0.95, 0.57))        # research file 1.3, 50 deg column
    curves = [("measured swift, extended (5 deg) [digitised]", "#e0b83a", MEASURED_EXT, "dots"), ("measured swift, 30 deg setting [digitised]", "#6cc08b", MEASURED_30, "dots"),
              ("measured swift, 50 deg setting [digitised]", "#ff9f43", MEASURED_50, "dots"), ("exact ellipse [calc]", "#9aa7b4", ELLIPSE, "dash"), ("v2 BS final", "#ff8a80", c_v2, "line"),
              ("v4 A (glide pose)", "#7fb4ff", [(e / 24, chord_over_mean(gA, e / 24)) for e in range(25)], "line"), ("v4 C (extended)", "#c39bff", [(e / 24, chord_over_mean(gC, e / 24)) for e in range(25)], "dash")]
    (out / "chord_overlay.svg").write_text(chord_plot_svg(curves), encoding="utf-8")
    items = [("v2 BS final", g2 | {"y_body": 0.03, "le_fn": lambda y, g2=g2: _le2(g2, y), "chord_fn": lambda y, g2=g2: _ch2(g2, y)}, "#ff8a80"), ("A as measured (glide pose)", gA, "#7fb4ff"),
             ("B hybrid (hand sweep +%s deg)" % S["B_key"].replace("hand+", ""), gB, "#e89a55"), ("C extended pose", gC, "#c39bff")]
    (out / "outlines.svg").write_text(outline_svg(items, title="Swift outlines side by side (top view, nose up, same scale)"), encoding="utf-8")
    rms_ext = rms_vs([(e, f) for e, f in MEASURED_EXT if 0.04 <= e <= 0.96], lambda e: _interp_pts(c_v2, e))
    rms_30 = rms_vs(MEASURED_30, lambda e: _interp_pts(c_v2, e))
    rms_A30 = rms_vs(MEASURED_30, lambda e: chord_over_mean(gA, e))
    rms_Cext = rms_vs([(e, f) for e, f in MEASURED_EXT if 0.04 <= e <= 0.96], lambda e: chord_over_mean(gC, e))
    fac = pullout_factors()
    iA, iB = infos["A"], infos["B"]
    bendA, bendB = iA["bend"], iB["bend"]
    kB = S["B_key"].replace("hand+", "")
    deps = {d[0]: d[4] for d in S["deps"]}
    cn_floor = EXISTING["cn"]
    wr = {(nm, we): o for nm, we, o in S["wrist"]}
    P = [f"<!doctype html><html lang='en'><head><meta charset='utf-8'><title>Swift outline v4</title><style>{ft.CSS}\n.scroll{{overflow-x:auto}}table.t td,table.t th{{padding:4px 7px;font-size:13px;vertical-align:top}}.warn{{background:#2c2418;border:2px solid #e89a55;padding:10px 16px}}figure{{margin:12px 0}}</style></head><body><main>"]
    P.append("<h1>Swift outline v4: two candidates that follow the real glide pose</h1>")
    P.append("<p class='note'>Tags: <b>DIGITISED</b> (measured by us from published bird figures, 5-10% error), <b>CITED</b>, <b>CALCULATED</b>, <b>MODEL</b> (AVL: inviscid, outline only, body ignored), <b>JUDGEMENT</b>, <b>ESTIMATE</b> (the data file's own estimate), <b>PROPOSED</b>. "
             f"Source of the bird numbers: <code>research/birds/planform_and_gliding.md</code>. {N_AVL['runs']} AVL runs were made (serial, 10 x 80 panels unless stated). Nothing here was flown or built." + (" <b>QUICK MODE.</b>" if quick else "") + "</p>")
    # ---- 1
    P.append("<section><h2>1. Summary: A and B are two candidates</h2>")
    rows = []
    for lab, o in (("A as measured (glide pose)", A), ("B hybrid", B), ("C extended pose (comparison)", C)):
        f_ex = [r[0] for r in criteria4(o, EXISTING) if not r[3]]
        f_pr = [r[0] for r in criteria4(o, PROPOSED) if not r[3]]
        rows.append(f"<tr><td><b>{lab}</b></td><td class='n'>{pair_ar(o['g']):.1f} / {o['g']['ar']:.1f}</td><td class='n'>{o['np_pct']:.1f}</td><td class='n'>{o['bend_8']:.1f} / {o['bend_10']:.1f}</td><td class='n'>{o['cg_mm_10']:.1f}</td>"
                    f"<td class='n'>{100 * o['ballast_frac_printed_8']:.0f}% / {100 * o['ballast_frac_foam_8']:.0f}%</td><td class='n'>{100 * o['ballast_frac_printed_10']:.0f}% / {100 * o['ballast_frac_foam_10']:.0f}%</td><td class='n'>{o['cn_beta']:+.4f}</td><td class='n'>{o['cl_beta']:+.4f}</td>"
                    f"<td class='n'>{o['tip_loading']:.2f}</td><td class='n'>{o['M_printed_10']:.1f} / {o['M_foam_10']:.1f}</td><td>{h('; '.join(f_ex) or 'none')}</td><td>{h('; '.join(f_pr) or 'none')}</td></tr>")
    P.append("<div class='scroll'><table class='t'><tr><th>outline</th><th>aspect ratio: two wings alone / with body strip</th><th>NP % MAC [MODEL]</th><th>tail bend for 8% / 10% margin, deg [MODEL]</th><th>trim CG at 10% margin, mm (250) [MODEL]</th><th>nose weight / all-up mass at 8% margin: printed / foam</th>"
             "<th>at 10% margin</th><th>Cn_beta, no fin</th><th>Cl_beta</th><th>tip loading [indicator]</th><th>all-up mass g, printed / foam [CALCULATED]</th><th>existing-threshold items not met</th><th>not met under the proposed alternative</th></tr>" + "".join(rows) + "</table></div>")
    P.append("<div class='warn'><p><b>Two candidates, not a recommendation.</b> "
             "<b>A</b> is the measured glide pose: the 30 degree setting's chord law, leading-edge sweep and aspect ratio, as faithfully as a flat sheet allows. It answers the original request most directly. "
             f"<b>B</b> is a <b>hybrid, not a measured shape</b>: it keeps A's chord law and aspect ratio but raises the hand leading-edge sweep by {kB} degrees, to a value between the bird's 30 and 50 degree settings; the real bird at that sweep has less span and a more even chord, so B does not exist in the bird. "
             f"The reason to consider B is <b>less nose weight</b>: {100 * B['ballast_frac_printed_8']:.1f}% against {100 * A['ballast_frac_printed_8']:.1f}% of the all-up mass printed, {100 * B['ballast_frac_foam_8']:.1f}% against {100 * A['ballast_frac_foam_8']:.1f}% foam (at an 8% margin), for a larger tail bend ({B['bend_10']:.1f} against {A['bend_10']:.1f} degrees for a 10% margin).</p>"
             f"<p><b>The yaw floor is not a reason.</b> The Cn_beta floor of {cn_floor} has no evidence behind it (section 4). A has {A['cn_beta']:+.4f} and B {B['cn_beta']:+.4f}, but Cn_beta rises smoothly with the hand sweep and nothing happens at 0.004. Neither outline is known to be yaw-stable without a fin. "
             "Which to build first is Kai's choice; cutting both in foam is the informative test. Everything here is MODEL.</p></div>")
    P.append("<p>Caveats that apply to every number: AVL's neutral point can carry up to about 5% MAC of panelling uncertainty with the tail in the wake plane; real flat plates lift 8-16% above AVL and their aerodynamic centre sits forward of AVL's (plate_balance); no force data exist for a flat plate cut to a bird outline at Re 15,000-40,000; AVL says nothing about stall. "
             f"The build files use a 10% static margin, not the 8% floor: measured plates put the lift point 0 to 5% of the chord ahead of linear theory, so a nominal 8% could be only 3 to 5% in reality, which is where the pull-out study says the model cannot say.</p></section>")
    # ---- 2
    P.append("<section><h2>2. Was the v2 shape what the real swift looks like? Partly, and the outer wing was too thin for its pose</h2><p>Kai's complaint was a broad inner wing and a very thin outer wing. The comparison uses the measured chord distributions of the real swift (Lentink & de Kat 2014, research file sections 1.2 and 1.3) as chord divided by the mean chord of the wings alone, against position along the wing from the body edge to the tip.</p>")
    P.append(f"<figure>{chord_plot_svg(curves)}<figcaption>Chord / mean chord. Dots: measured swift [DIGITISED, 5-10% error] at the extended, 30 and 50 degree settings. Red: the v2 BS final. Blue and violet: v4 A (glide) and C (extended).</figcaption></figure>")
    P.append("<div class='scroll'><table class='t'><tr><th>position along the wing</th><th>swift extended</th><th>swift 30 deg</th><th>swift 50 deg</th><th>ellipse</th><th>v2 BS final</th><th>v4 A</th><th>v4 C</th></tr>")
    for e in (0.1, 0.3, 0.5, 0.7, 0.8, 0.9, 0.95):
        P.append(f"<tr><td class='n'>{e:g}</td><td class='n'>{_interp_pts(MEASURED_EXT, e):.2f}</td><td class='n'>{_interp_pts(MEASURED_30, e):.2f}</td><td class='n'>{_interp_pts(MEASURED_50, e):.2f}</td><td class='n'>{_interp_pts(ELLIPSE, e):.2f}</td><td class='n'>{_interp_pts(c_v2, e):.2f}</td><td class='n'>{chord_over_mean(gA, e):.2f}</td><td class='n'>{chord_over_mean(gC, e):.2f}</td></tr>")
    v08, v09 = _interp_pts(c_v2, 0.8), _interp_pts(c_v2, 0.9)
    P.append(f"</table></div><p><b>What the numbers say.</b> Against the measured EXTENDED swift the v2 chord law differs by {rms_ext:.2f} (rms, in units of the mean chord), inside the 5-10% digitising error, so the broad root is real. But v2 was drawn with a hand sweep of about 50 degrees, and at that sweep the real bird's chord is far fuller outboard: "
             f"{_interp_pts(MEASURED_50, 0.8):.2f} at 0.8 and {_interp_pts(MEASURED_50, 0.9):.2f} at 0.9 of the wing (research file section 1.3), against v2's {v08:.2f} and {v09:.2f}. <b>So the outer wing of v2 was too thin for the pose it was drawn in; Kai's eye was right.</b> "
             "The bird at that sweep also has only 0.66 of its span; a gliding swift holds 0.85 to 0.90, close to the 30 degree setting. The other bird-unlike things in v2: the wrist at 40% of the half span against an ESTIMATED 20-25%, and the pairing of an extended-wing chord law with a swept hand. "
             "A pointed tip is itself bird-like (the bird's chord at 0.94 of the wing is "
             f"{_interp_pts(MEASURED_EXT, 0.94):.2f} of the mean, v2 has {_interp_pts(c_v2, 0.94):.2f}); the problem was the pairing, not the taper alone.</p></section>")
    P.append("<figure><img src='outlines.svg' alt='outlines' style='max-width:100%'><figcaption>Top views, nose up, same scale: v2 BS final, v4 A, v4 B, v4 C.</figcaption></figure>")
    # ---- 3
    P.append("<section><h2>3. Outline A, the glide pose as measured</h2><p>Recipe parameters (all free numbers) and where each comes from:</p><div class='scroll'><table class='t'><tr><th>parameter</th><th>value used</th><th>tag</th><th>basis</th></tr>")
    pa = GLIDE
    vals = {"ar_target": f"{pa['ar_target']:.2f} for the two wings alone (body strip excluded); {gA['ar']:.2f} with the body strip", "body_half": f"{pa['body_half']:g} b (width {2 * pa['body_half'] * SPAN:.0f} mm at 250)", "wrist_eta": f"{pa['wrist_eta']:g} of the wing = {gA['y_wrist'] / gA['half_span']:.3f} of the half span ({gA['y_wrist'] * SPAN:.0f} mm from the centreline at 250)",
            "chord_knots": "; ".join(f"{e:g}: {f:g}" for e, f in pa["chord_knots"]), "min_chord": f"{pa['min_chord']:g} b = {pa['min_chord'] * 250:.1f} mm at 250, {pa['min_chord'] * 400:.0f} mm at 400", "le_arm_deg": f"{pa['le_arm_deg']:g} deg back",
            "le_hand_knots": "; ".join(f"{e:g}: {a:g} deg" for e, a in pa["le_hand_knots"]), "tail_len": f"{pa['tail_len']:.3f} b ({pa['tail_len'] * 250:.0f} mm at 250)", "tail_spread": f"{pa['tail_spread']:g} b ({pa['tail_spread'] * 250:.0f} mm): the SLOW-speed (7 m/s) value; the bird furls it to 0.10 b at 11 m/s",
            "tail_neck": f"{pa['tail_neck']:g} b ({pa['tail_neck'] * 250:.0f} mm)", "hinge_gap": f"{pa['hinge_gap']:g} b", "nose": f"{pa['nose']:g} b ({pa['nose'] * 250:.0f} mm)"}
    for k_, v_ in vals.items():
        tg, bs = PARAM_SOURCE[k_]
        P.append(f"<tr><td>{k_}</td><td>{h(v_)}</td><td>{tg}</td><td>{h(bs)}</td></tr>")
    pl = area_parts(gA)
    P.append("</table></div>")
    P.append(f"<p><b>Aspect ratio.</b> The data file's 8.5 (glide pose) and 10.1 (extended) are for the two wings alone, without the body. The outlines were built so that the two wings alone have exactly those values; with the body strip included (the way the sheet area, and the AVL area, is counted) they come out at {gA['ar']:.2f} for A and {gC['ar']:.2f} for C. "
             "In outlines.csv both values are given.</p>")
    P.append(f"<p>Resulting planform at 250 mm span [CALCULATED from the recipe]: root chord {gA['c_root'] * SPAN:.1f} mm, chord at the wrist {gA['chord_fn'](gA['y_wrist']) * SPAN:.1f} mm, mean chord of the wings {wing_mean_chord(gA) * SPAN:.1f} mm, tip chord {gA['chord_fn'](gA['half_span']) * SPAN:.1f} mm (the minimum buildable chord truncates the point). "
             f"Area split: body strip {100 * pl['strip'] / gA['S']:.0f}%, arm wing {100 * pl['arm'] / gA['S']:.0f}%, hand wing {100 * pl['hand'] / gA['S']:.0f}% of the wing area ({gA['S'] * SPAN ** 2:,.0f} mm&sup2;). Fan tail {gA['tail_area'] * SPAN ** 2:,.0f} mm&sup2; ({100 * gA['tail_area'] / gA['S']:.1f}% of the wing area), "
             f"nose and root blend (body) {pl['body_extra'] * SPAN ** 2:,.0f} mm&sup2; (sheet total {gA['sheet_area'] * SPAN ** 2:,.0f} mm&sup2;). The chord law reproduces the measured 30 degree values with rms {rms_A30:.3f} of the mean chord, and C the measured extended values with rms {rms_Cext:.3f}.</p>")
    P.append("<h3>What a flat sheet cannot copy (from the data file)</h3><ul><li>Camber, and the thickness change from the arm to the hand: a flat plate makes less lift and stalls earlier. No flat-plate force data for a bird outline at our Reynolds number exist.</li>"
             "<li>Twist or washout, feather gaps and slots, feather roughness (1-2% of the chord), the porous tip, and the change of span, sweep and tail spread with speed (only a folding mechanism could).</li>"
             "<li>The tail fork (depth NOT FOUND) and the tail carrying and trimming lift in flight: the sheet has a rounded fan tail bent once on one hinge.</li>"
             "<li>The pointed tip: a sheet needs a minimum chord (JUDGEMENT, not confirmed by Kai).</li></ul>")
    P.append("<h3>Departures that remain in A, and what each costs</h3><ul>"
             f"<li><b>Minimum chord at the tip</b>: the tip chord is {gA['chord_fn'](gA['half_span']) * SPAN:.1f} mm instead of a point; the bird's chord at 0.95 of the wing is {_interp_pts(MEASURED_30, 0.95):.2f} of the mean ({_interp_pts(MEASURED_30, 0.95) * wing_mean_chord(gA) * SPAN:.1f} mm here), so only the last 5% differs.</li>"
             "<li><b>Fan tail instead of a fork</b>, its length solved so its area is the bird's 18.5% of the wing area (CALCULATED from lift coefficients 0.96 and 0.81); the search-excerpt feather length (0.19-0.23 b) would give a tail with about twice that area. Cost: unknown. The tail spread is the slow-speed value.</li>"
             "<li><b>Nose 0.09 b</b>: no head or neck data; it only changes ballast (a longer nose helps, see section 5).</li>"
             "<li><b>The wrist is an ESTIMATE</b>; its effect is in section 6.</li></ul></section>")
    # ---- 4
    P.append("<section><h2>4. The thresholds, one number each</h2><p>Each criterion is shown as its own number for A, B and C, under the existing (v2) thresholds, which are unchanged, and under an alternative I label PROPOSED where I think a threshold is arbitrary. Kai decides.</p>")
    for lab, o in (("A as measured", A), ("B hybrid", B), ("C extended", C)):
        P.append(f"<h3>{lab}</h3><div class='scroll'><table class='t'><tr><th>criterion</th><th>value</th><th>existing limit</th><th>existing</th><th>PROPOSED alternative</th><th>proposed</th></tr>")
        ce, cp = criteria4(o, EXISTING), criteria4(o, PROPOSED)
        for (nm, val, lim, ok), (_, _, lim2, ok2) in zip(ce, cp):
            P.append(f"<tr><td>{h(nm)}</td><td>{h(val)}</td><td>{h(lim)}</td><td style='background:{'#1d3b2a' if ok else '#3b1d1d'}'>{'met' if ok else 'NOT MET'}</td><td>{h(lim2)}</td><td style='background:{'#1d3b2a' if ok2 else '#3b1d1d'}'>{'met' if ok2 else 'NOT MET'}</td></tr>")
        P.append("</table></div>")
    P.append("<h3>The Cn_beta floor of 0.004 has no evidence behind it</h3>")
    P.append("<div class='scroll'><table class='t'><tr><th>hand leading-edge sweep added to A</th>" + "".join(f"<th>+{k} deg</th>" for k in ("0", "4", "6", "7", "8", "10")) + "</tr><tr><td>Cn_beta, no fin [MODEL]</td>" +
             "".join(f"<td class='n'>{(A if k == '0' else deps.get('hand+' + k, {'cn_beta': float('nan')}))['cn_beta']:+.5f}</td>" for k in ("0", "4", "6", "7", "8", "10")) + "</tr></table></div>")
    wB1, wB2, wB3 = wr.get(("B", 0.139)), wr.get(("B", 0.2)), wr.get(("B", 0.3))
    P.append("<p>Cn_beta rises smoothly with the hand sweep; nothing happens at 0.004, which was our own choice. "
             + (f"B clears the floor by only {100 * (wB1['cn_beta'] / cn_floor - 1):.1f}% with the estimated wrist (0.139 of the wing); with the wrist at 0.2 it is {abs(100 * (wB2['cn_beta'] / cn_floor - 1)):.1f}% {'under' if wB2['cn_beta'] < cn_floor else 'over'} the line, i.e. on it" if wB1 and wB2 else "") +
             ". Neither A nor B is known to be yaw-stable without a fin. In the earlier AVL comparisons for the P07 flat wing (a different outline, 25 degree sweep, 3 degrees of dihedral) Cn_beta was 0.0016 bare, about 0.008 with 4% tip fins and 0.017 with a 6% centre fin "
             "(<code>rigid_glider/ARCH_COMPARISON.md</code>, lateral table; <code>rigid_glider/ARTICLE1.md</code>), so fins change it by factors of 5 to 10, far more than the sweep changes here. The data file (<code>research/birds/planform_and_gliding.md</code>, section 3) says whether a flat swept plate is yaw-stable without a fin is NOT FOUND and advises planning for tip fins or a small fin until a test shows otherwise: "
             "<b>a fin or tip fins are the fallback if either outline wanders in yaw.</b></p>")
    P.append("<h3>Which thresholds are real for this project?</h3><ul>"
             "<li><b>Static margin window 8-15%</b>: the floor is real in kind (a positive margin is needed; the neutral point is uncertain by about 5% MAC and a flat plate's aerodynamic centre sits forward of AVL's) but the number 8 is JUDGEMENT; the ceiling just limits the bend. Kept.</li>"
             "<li><b>Nose weight under 20% of the all-up mass</b>: real as a cost, but 20% is inherited from our own v2 choice. PROPOSED alternative 25%, shown as a second column.</li>"
             "<li><b>Cn_beta floor 0.004 with no fin</b>: arbitrary (above). PROPOSED alternative: any value clearly above the AVL noise, 0.002, until a flight says otherwise.</li>"
             "<li><b>Tip loading not above the v1 outline</b>: inherited; v1 was itself arbitrary and the indicator has no demonstrated link to stalling for a flat plate. PROPOSED: report it, do not gate on it.</li>"
             "<li><b>Neck minimum</b>: real (a 0.4 mm hinge strip must carry the tail load) but rests on ASSUMED strength.</li></ul></section>")
    # ---- 5
    P.append("<section><h2>5. Outline B: a hybrid that trades a larger tail bend for less nose weight</h2><p>Single departures from A, each tried alone (AVL, bend 0 and 5 deg). Items not met are the existing thresholds.</p>")
    P.append("<div class='scroll'><table class='t'><tr><th>departure</th><th>aimed at</th><th>Cn_beta</th><th>bend for 8% margin, deg</th><th>ballast printed / foam at 8%</th><th>tip loading</th><th>NP % MAC</th><th>existing-threshold items not met</th></tr>")
    P.append(f"<tr style='background:#1c2229'><td><b>A (reference)</b></td><td></td><td class='n'>{A['cn_beta']:+.4f}</td><td class='n'>{A['bend_8']:.1f}</td><td class='n'>{100 * A['ballast_frac_printed_8']:.1f}% / {100 * A['ballast_frac_foam_8']:.1f}%</td><td class='n'>{A['tip_loading']:.2f}</td><td class='n'>{A['np_pct']:.1f}</td><td>{h('; '.join(r[0] for r in criteria4(A, EXISTING) if not r[3]) or 'none')}</td></tr>")
    for key, label, why, upd, o in S["deps"]:
        P.append(f"<tr><td>{h(label)}</td><td>{why}</td><td class='n'>{o['cn_beta']:+.4f}</td><td class='n'>{o['bend_8']:.1f}</td><td class='n'>{100 * o['ballast_frac_printed_8']:.1f}% / {100 * o['ballast_frac_foam_8']:.1f}%</td><td class='n'>{o['tip_loading']:.2f}</td><td class='n'>{o['np_pct']:.1f}</td><td>{h('; '.join(r[0] for r in criteria4(o, EXISTING) if not r[3]) or 'none')}</td></tr>")
    P.append("</table></div>")
    h10 = deps.get("hand+10")
    P.append(f"<p><b>B = A with the hand-wing leading-edge sweep +{kB} deg</b> (knots 37/37/44 deg become {37 + float(kB):g}/{37 + float(kB):g}/{44 + float(kB):g} deg). It is the smallest step in the search that is at least {100 * (CN_MARGIN - 1):.0f}% above the 0.004 floor in Cn_beta while meeting the other existing thresholds in this model. "
             f"<b>Disclosure:</b> the margin rule was first 5%, which gives +10 deg (Cn_beta {h10['cn_beta']:+.4f}); I changed it to 2% when the panelling check showed Cn_beta moves by under 1%. Either margin is arbitrary and the floor itself has no evidence, so +8 and +10 deg are equally defensible. "
             f"What B buys: nose weight at an 8% margin {100 * A['ballast_frac_printed_8']:.1f}% to {100 * B['ballast_frac_printed_8']:.1f}% printed ({100 * A['ballast_frac_foam_8']:.1f}% to {100 * B['ballast_frac_foam_8']:.1f}% foam); at a 10% margin printed {100 * A['ballast_frac_printed_10']:.1f}% to {100 * B['ballast_frac_printed_10']:.1f}%; Cn_beta {A['cn_beta']:+.4f} to {B['cn_beta']:+.4f}. "
             f"What it costs: less tail authority ({A['cm0_per_deg']:.4f} to {B['cm0_per_deg']:.4f} CM0 per degree, so {A['bend_10']:.1f} to {B['bend_10']:.1f} deg of bend for a 10% margin) and a neutral point moved from {A['np_pct']:.1f} to {B['np_pct']:.1f}% MAC. "
             "B is a hybrid: the chord law and aspect ratio of the 30 degree pose with a hand sweep between the 30 and 50 degree poses, where the real bird has less span and a more even chord. The nose and hinge gap reduce ballast but do nothing for Cn_beta.</p></section>")
    # ---- 6
    P.append(f"<section><h2>6. Outline C and the wrist</h2><p>C is the 5 degree setting (aspect ratio {pair_ar(gC):.1f} for the two wings alone, {gC['ar']:.1f} with the body strip; arm leading edge 9 degrees forward, hand sweep 6 to 17 degrees rising to the tip). It is for comparison and for the later folding work, and with the existing thresholds it is far off: Cn_beta {C['cn_beta']:+.4f}, Cl_beta {C['cl_beta']:+.4f} (barely stable), and nose weight {100 * C['ballast_frac_printed_8']:.0f}% printed / {100 * C['ballast_frac_foam_8']:.0f}% foam at an 8% margin, because the bare CG sits {C['bare_cg_pct_mac_aft_of_np']:.0f}% MAC behind the neutral point; "
             f"the tail is very effective here ({C['cm0_per_deg']:.4f} CM0 per degree, {C['bend_8']:.1f} deg for an 8% margin).</p>")
    P.append("<h3>Sensitivity to the wrist position (an ESTIMATE in the data)</h3><p>Wrist positions are in the file's units, fractions of the wing from the body edge to the tip; the half-span equivalents are in the second column.</p><div class='scroll'><table class='t'><tr><th>outline</th><th>wrist, fraction of the wing (half span)</th><th>NP % MAC</th><th>bend for 8% margin</th><th>ballast printed / foam at 8%</th><th>Cn_beta</th><th>Cn_beta vs the 0.004 floor</th><th>tip loading</th><th>existing-threshold items not met</th></tr>")
    for nm, we, o in S["wrist"]:
        P.append(f"<tr><td>{nm}</td><td class='n'>{we:g} ({o['g']['y_wrist'] / o['g']['half_span']:.3f})</td><td class='n'>{o['np_pct']:.1f}</td><td class='n'>{o['bend_8']:.1f}</td><td class='n'>{100 * o['ballast_frac_printed_8']:.1f}% / {100 * o['ballast_frac_foam_8']:.1f}%</td><td class='n'>{o['cn_beta']:+.4f}</td><td class='n'>{100 * (o['cn_beta'] / cn_floor - 1):+.1f}%</td><td class='n'>{o['tip_loading']:.2f}</td><td>{h('; '.join(r[0] for r in criteria4(o, EXISTING) if not r[3]) or 'none')}</td></tr>")
    P.append("</table></div>")
    okB = [we for nm, we, o in S["wrist"] if nm == "B" and passes(o, EXISTING)]
    okA = [we for nm, we, o in S["wrist"] if nm == "A" and passes(o, EXISTING)]
    cnB = {we: o["cn_beta"] for nm, we, o in S["wrist"] if nm == "B"}
    P.append(f"<p>Reading the table (wrist positions in the file's wing-fraction units; half-span equivalents 0.139 = 0.225, 0.2 = 0.28, 0.3 = 0.37, 0.4 = 0.46): A meets all existing thresholds at {okA or 'none of the wrist positions tried'} (it is under the Cn_beta line at every one, and its nose weight also passes the 20% limit only up to 0.2). "
             f"B meets all of them only at {okB or 'none'}; at 0.2 its Cn_beta is {cnB.get(0.2, float('nan')):+.5f}, which is on the line (within {abs(100 * (cnB.get(0.2, 0) / cn_floor - 1)):.1f}% of it, smaller than the panelling spread); at 0.3 and 0.4 it is {abs(100 * (cnB.get(0.3, 0) / cn_floor - 1)):.0f}% and {abs(100 * (cnB.get(0.4, 0) / cn_floor - 1)):.0f}% under it. "
             "So B's Cn_beta result depends on an estimated wrist position. The wrist here is where the leading-edge sweep starts to increase (and the natural fold line); moving it outboard shortens the hand and changes the ramp.</p></section>")
    # ---- 7 panelling
    if S["panel"]:
        P.append("<section><h2>7. Panelling check (neutral point and Cn_beta)</h2><div class='scroll'><table class='t'><tr><th>outline</th><th>panels (chord x span)</th><th>NP % MAC</th><th>CM0 at 5 deg</th><th>Cn_beta</th><th>Cl_beta</th><th>tip loading</th></tr>" + "".join(
            f"<tr><td>{r['outline']}</td><td>{r['nc']} x {r['ns']}</td><td class='n'>{r['np_pct']:.1f}</td><td class='n'>{r['cm0']:+.4f}</td><td class='n'>{r['cn']:+.4f}</td><td class='n'>{r['cl']:+.4f}</td><td class='n'>{r['tip']:.2f}</td></tr>" for r in S["panel"]) + "</table></div>"
                 f"<p>For these two outlines the neutral point moves by at most {max(abs(r['np_pct'] - S['panel'][1 if r['outline'] == 'A' else 4]['np_pct']) for r in S['panel']):.1f} points of MAC between 8 x 64 and 16 x 128 panels and Cn_beta by under 1%. That is smaller than the up to about 5% MAC seen with the tail in the wake plane on some v2 outlines, which remains the caveat to carry. Differences smaller than these spreads are not results.</p></section>")
    # ---- 8 builds, depth
    P.append("<section><h2>8. Build files, pull-out depth and masses (both candidates)</h2>")
    P.append("<p>Build files for A and B use a <b>10% static margin</b> and each outline's own tail bend: the nearest angle on the field gauge (2, 2.5, 4, 5, 6, 7.5, 8 deg) to what it needs. Why 10% and not the 8% floor: measured plates put the lift point 0 to 5% of the chord ahead of linear theory, so a nominal 8% could be 3 to 5% in reality, which is where the pull-out study says the model cannot say.</p>")
    P.append("<div class='scroll'><table class='t'><tr><th>build</th><th>tail bend, deg</th><th>margin at that bend</th><th>trim CG, mm aft of root LE</th><th>sheet g</th><th>rail g</th><th>nose weight g (% of all-up)</th><th>all-up g</th><th>wing loading N/m&sup2;</th><th>first-swoop depth, m [MODEL]: formula to upper bound</th></tr>")
    for nm, o, info in (("A", A, iA), ("B", B, iB)):
        for b in info["builds"]:
            n = b["n"]
            f0, lo, hi = depth_range(n, fac, b["size"])
            cm = o["cm0_per_deg"] * info["bend"] / 0.3
            P.append(f"<tr><td>{nm} {b['size']} {b['span']:g} mm</td><td class='n'>{info['bend']:g}</td><td class='n'>{100 * cm:.1f}%</td><td class='n'>{n['cg_target']:.1f}</td><td class='n'>{n['m_sheet']:.2f}</td><td class='n'>{n['rail']['mass_g']:.2f}</td><td class='n'>{n['ballast_g']:.2f} ({100 * n['ballast_frac']:.0f}%)</td><td class='n'>{n['M']:.2f}</td><td class='n'>{n['ws']:.1f}</td><td class='n'>{f0:.0f} to {('%.0f' % hi) if hi else 'n/a'}</td></tr>")
    P.append("</table></div>")
    if fac:
        P.append(f"<p><b>Pull-out depth.</b> The lower number is 3 (W/S) / (rho g CL) at CL 0.3, the constant-lift-coefficient relation. The upper number multiplies it by the pull-out study's simulation-to-formula ratio for the bird outline ({fac['ratio_printed']:.2f} printed, {fac['ratio_foam']:.2f} for the light foam bird) and by the factor for a tail that is only 50% as effective as AVL says ({fac['eff50']:.2f}; {fac['eff75']:.2f} at 75%) "
                 "(<code>results/runs/rigid_glider/pullout/cases.csv</code>, MODEL). <b>A 10 m drop may not be enough for the printed outline</b>; the pull-out study's plates had aspect ratios 4 and 5, so the bird outlines (8.5) are an extrapolation.</p>")
    P.append("<p><b>What the bend rests on.</b> The tail bend's effect comes from inviscid AVL for a small flat tail in the wing's wake at a chord Reynolds number near 10,000; it is unvalidated. If the real tail is less effective, the trim lift falls and the depth rises. The tail spread used (0.23 of the span) is the bird's slow-speed (7 m/s) value. "
             "Drop height and whether to add a nose-up input are Kai's decisions.</p>")
    P.append(f"<p>The bird's own CG (10% of the body length behind the shoulder, an all-species value) would be about {0.10 * 0.42 * 250:.0f} mm aft of the shoulder; the trim CGs above are mm aft of the root leading edge. The body (nose and root blend) adds {area_parts(gA)['body_extra'] * SPAN ** 2:,.0f} mm&sup2; to the sheet ({(area_parts(gA)['body_extra'] * SPAN ** 2 * 0.6 * 1.24e-3):.2f} g printed).</p></section>")
    P.append("<section><h2>9. Where the fold lines fall (no folding design yet)</h2><p>On every drawing and template the <b>wrist</b> (blue dashed; ESTIMATED position) and the <b>body edge</b> (purple dotted) are marked as fold-line candidates, plus the tail hinge (orange). "
             f"For A they sit at {gA['y_wrist'] * SPAN:.0f} mm and {gA['y_body'] * SPAN:.0f} mm from the centreline at 250 mm span ({gA['y_wrist'] * 400:.0f} mm and {gA['y_body'] * 400:.0f} mm at 400 mm); for B at {gB['y_wrist'] * SPAN:.0f} mm and {gB['y_body'] * SPAN:.0f} mm. They are markers only: nothing is cut or creased there.</p></section>")
    P.append("<section><h2>10. Files</h2><ul>" + "".join(f"<li>{x}</li>" for x in iA["files"] + iB["files"]) + "</ul><p>" + "".join(f"Templates for {nm}, 1:1 Letter, white: <a href='{i.get('pdf', '')}'>{i.get('pdf', '')}</a>. " for nm, i in (("A", iA), ("B", iB)) if i.get("pdf")) +
             "Tables: <a href='outlines.csv'>outlines.csv</a>, <a href='criteria.csv'>criteria.csv</a>, <a href='departures.csv'>departures.csv</a>, <a href='wrist_sensitivity.csv'>wrist_sensitivity.csv</a>, <a href='panelling.csv'>panelling.csv</a>, <a href='chord_comparison.csv'>chord_comparison.csv</a>. Drawings: <a href='outlines.svg'>outlines.svg</a>, <a href='chord_overlay.svg'>chord_overlay.svg</a>.</p></section></main></body></html>")
    (out / "index.html").write_text("\n".join(P), encoding="utf-8")
    return dict(rms_ext=rms_ext, rms_30=rms_30, rms_A30=rms_A30, rms_Cext=rms_Cext)


def pair_ar(g):
    """Aspect ratio of the two wings alone: span without the body, area without the body strip (the quantity the data file gives)."""
    return (g["b"] - 2 * g["y_body"]) ** 2 / (g["S"] - area_parts(g)["strip"])


def _interp_pts(pts, x):
    pts = sorted(pts)
    if x <= pts[0][0]:
        return pts[0][1]
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        if x <= x1:
            return y0 + (x - x0) / (x1 - x0) * (y1 - y0)
    return pts[-1][1]


def _le2(g2, y):
    ys, le = g2["y_nodes"], g2["le_nodes"]
    for i in range(len(ys) - 1):
        if ys[i] <= y <= ys[i + 1]:
            return le[i] + (y - ys[i]) / (ys[i + 1] - ys[i]) * (le[i + 1] - le[i])
    return le[-1]


def _ch2(g2, y):
    ys, le, te = g2["y_nodes"], g2["le_nodes"], g2["te_nodes"]
    for i in range(len(ys) - 1):
        if ys[i] <= y <= ys[i + 1]:
            t = (y - ys[i]) / (ys[i + 1] - ys[i])
            return (te[i] + t * (te[i + 1] - te[i])) - (le[i] + t * (le[i + 1] - le[i]))
    return 0.0


def write_tables(out: Path, S, rms):
    A, B, C = S["A"], S["B"], S["C"]
    write_csv(out / "outlines.csv", SUMMARY_HDR, [summary_row(o) for o in (A, B, C)])
    rows = []
    for lab, o in (("A", A), ("B", B), ("C", C)):
        for thr in (EXISTING, PROPOSED):
            for nm, val, lim, ok in criteria4(o, thr):
                rows.append([lab, thr["name"], nm, val, lim, "pass" if ok else "FAIL"])
    write_csv(out / "criteria.csv", ["outline", "threshold set", "criterion", "value", "limit", "result"], rows)
    write_csv(out / "departures.csv", ["departure", "aimed at", "Cn_beta", "Cl_beta", "NP % MAC", "CM0 per deg", "bend for 8% deg", "ballast % printed at 8%", "ballast % foam at 8%", "tip loading", "existing criteria not met"],
              [["A (reference)", "", f"{A['cn_beta']:+.5f}", f"{A['cl_beta']:+.5f}", f"{A['np_pct']:.2f}", f"{A['cm0_per_deg']:.5f}", f"{A['bend_8']:.2f}", f"{100 * A['ballast_frac_printed_8']:.2f}", f"{100 * A['ballast_frac_foam_8']:.2f}", f"{A['tip_loading']:.3f}",
                "; ".join(r[0] for r in criteria4(A, EXISTING) if not r[3])]] +
              [[label, why, f"{o['cn_beta']:+.5f}", f"{o['cl_beta']:+.5f}", f"{o['np_pct']:.2f}", f"{o['cm0_per_deg']:.5f}", f"{o['bend_8']:.2f}", f"{100 * o['ballast_frac_printed_8']:.2f}", f"{100 * o['ballast_frac_foam_8']:.2f}", f"{o['tip_loading']:.3f}",
                "; ".join(r[0] for r in criteria4(o, EXISTING) if not r[3])] for key, label, why, upd, o in S["deps"]])
    write_csv(out / "wrist_sensitivity.csv", ["outline", "wrist fraction of wing", "wrist fraction of half span", "NP % MAC", "bend for 8% deg", "ballast % printed at 8%", "ballast % foam at 8%", "Cn_beta", "Cl_beta", "tip loading", "existing criteria not met"],
              [[nm, we, f"{o['g']['y_wrist'] / o['g']['half_span']:.3f}", f"{o['np_pct']:.2f}", f"{o['bend_8']:.2f}", f"{100 * o['ballast_frac_printed_8']:.2f}", f"{100 * o['ballast_frac_foam_8']:.2f}", f"{o['cn_beta']:+.5f}", f"{o['cl_beta']:+.5f}", f"{o['tip_loading']:.3f}",
                "; ".join(r[0] for r in criteria4(o, EXISTING) if not r[3])] for nm, we, o in S["wrist"]])
    write_csv(out / "panelling.csv", ["outline", "chordwise", "spanwise", "NP % MAC", "CM0 at 5 deg", "Cn_beta", "Cl_beta", "tip loading"], [[r["outline"], r["nc"], r["ns"], f"{r['np_pct']:.2f}", f"{r['cm0']:.5f}", f"{r['cn']:.5f}", f"{r['cl']:.5f}", f"{r['tip']:.3f}"] for r in S["panel"]])
    g2 = v2_final_geometry()
    c_v2, _ = chord_curve_v2(g2)
    pts = sorted({e for e, _ in MEASURED_EXT} | {e for e, _ in MEASURED_30} | {0.94})
    write_csv(out / "chord_comparison.csv", ["position along the wing (0 body edge, 1 tip)", "swift extended (measured, interpolated)", "ellipse", "swift 30 deg (measured, interpolated)", "v2 BS final", "v4 A", "v4 B", "v4 C"],
              [[e, f"{_interp_pts(MEASURED_EXT, e):.3f}", f"{_interp_pts(ELLIPSE, e):.3f}", f"{_interp_pts(MEASURED_30, e):.3f}", f"{_interp_pts(c_v2, e):.3f}", f"{chord_over_mean(A['g'], e):.3f}", f"{chord_over_mean(B['g'], e):.3f}", f"{chord_over_mean(C['g'], e):.3f}"] for e in pts])


def run(out: Path = OUT_DEFAULT, pdf=True, quick=False, log=print):
    out.mkdir(parents=True, exist_ok=True)
    S = run_studies(out, quick, log)
    infos = {}
    for nm in ("A", "B"):
        o = S[nm]
        infos[nm] = build_files(o, nearest_gauge(o["bend_10"]), out, nm, make_pdf=pdf)      # both candidates: same 10 % margin, each outline's own bend
    rms = build_page(out, S, infos, quick)
    write_tables(out, S, rms)
    return dict(S=S, info=infos["B"], infos=infos, rms=rms, avl_runs=N_AVL["runs"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=OUT_DEFAULT)
    ap.add_argument("--no-pdf", action="store_true")
    ap.add_argument("--quick", action="store_true")
    a = ap.parse_args()
    R = run(a.out, not a.no_pdf, a.quick)
    S = R["S"]
    print("AVL runs:", R["avl_runs"], "| B =", S["B_key"])
    for nm in ("A", "B", "C"):
        o = S[nm]
        print(nm, f"NP {o['np_pct']:.1f} bend8 {o['bend_8']:.2f} Cn {o['cn_beta']:+.4f} ballast8 {100 * o['ballast_frac_printed_8']:.1f}/{100 * o['ballast_frac_foam_8']:.1f} tip {o['tip_loading']:.2f}")


if __name__ == "__main__":
    main()
