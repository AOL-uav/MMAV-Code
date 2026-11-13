"""Shape sweep v1: a bounded, structured sweep over planform families and stability features, scored on lateral robustness (twist tolerance, spiral, oscillation) and pitch margin.
MODEL study (inviscid AVL 3.52), nothing here is a flight result.  Standard library only (GIF writer, 5x7 bitmap font, SVG, HTML are in this file).

    py -B rigid_glider/shape_sweep_v1.py [--out DIR] [--work DIR] [--stage 1|2|all] [--refs-only] [--render]

Reuses (read-only import) straight_smooth_v1 (deck pieces, mass items, AVL session, steady-turn closed form with p' = r' tan(-gamma), asymmetry definitions),
flat_test (trapezoid planform), bird_outline(_v2) / bird_outline_v4_swift (Swift A outline family), fin_kit_v4 (fin geometry, eigenvalues), known_good_zanonia (data files),
sheet_builder (mass, bed fit).  Writes only under results/runs/rigid_glider/shape_sweep_v1/ (AVL scratch files go to --work, outside the OneDrive folder).
Axes: AVL x aft, y starboard, z up.  Lengths mm unless a name says _m.  Pre-stated rule and method: README.md of the output folder (frozen before the run).
"""
from __future__ import annotations
import sys
sys.dont_write_bytecode = True
import argparse
import csv
import hashlib
import json
import math
import os
import pickle
import re
import tempfile
import time
import zlib
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import flat_test as ft                    # read-only imports
import b45_centre_flaps as b45
import bird_outline as bo
import bird_outline_v2 as v2
import bird_outline_v4_swift as s4
import fin_kit_v4 as fk
import straight_smooth_v1 as ss
import sheet_builder as sb

ROOT = ft.ROOT
OUT = ROOT / "results/runs/rigid_glider/shape_sweep_v1"
ZAN_DIR = ROOT / "research/known_good/zanonia"

# ------------------------------------------------------------------------------------------------ constants (pre-stated in README; do not change after the run)
SPAN = 400.0
SM_TARGET = 0.12                 # static margin held in the sweep, fraction of MAC
CL_TARGET = 0.30                 # Stage 1 trim CL
CL_STAGE2 = (0.25, 0.35)
CD0 = 0.0307                     # ASSUMED profile drag; calibrated so Swift A as flown (CL 0.296, AVL induced CD 0.0036) gives gamma = -6.61 deg (the swiftA_oscillation value)
GAMMA_REF = ss.GLIDE_REF         # -6.61 deg, used as is for the three reference configurations
FLAP_Y = (3.0 / 20.0, 2.0 / 5.0)  # reflex flap spans 0.15..0.40 of the semi-span (B45: 30..80 mm of 200)
FLAP_FRAC = 0.15                 # T family reflex flap chord, fraction of the local chord (ASSUMED; B45 flown flap is 10 mm = about 9 %)
DELTA_MAX = 25.0                 # deg: reflex / tail-bend limit above which the trim is called infeasible
KINK = 0.6                       # tip dihedral starts at 0.6 of the semi-span (outer 40 %)
FIN_H = 24.0                     # T tip fin height, mm (B45 as flown), chord = tip chord
TIP_FIN_T_MM, PLA_RHO = 0.6, 1.24
TW_R_M = 30.0                    # turn radius used for the twist tolerance
SPIRAL_T2_MIN = 30.0             # s, rule 1
DR_ZETA_MIN = 0.10
SM_BAND = (8.0, 20.0)            # % MAC, rule 4
ALPHA_MAX = 5.0                  # deg, rule 4
ALPHA_BREAK = 8.0                # deg, thin-plate lift-slope break (report)
TW_FACTOR = 3.0                  # rule 3: times the B45 baseline twist tolerance
BED = 250.0
AMP = ss.AMP
ASYM = ss.ASYM_DEG
Z_B = 0.4 / (2 * TW_R_M)         # r_hat for R = 30 m at b = 0.4 m


# ================================================================================================ outline / planform specs
def poly_area(poly):
    return 0.5 * sum(poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1] for i in range(len(poly)))


def mirror_poly(half):
    """Right-half outline (nose/root-front point first and root-rear point last, both on y = 0) -> full closed outline."""
    return list(half) + [(x, -y) for x, y in reversed(half[1:-1])]


def spec_T(ar, sweep, taper, flap_mm=None, name=None):
    """Trapezoid flying wing at 400 mm: flat_test.wing_geometry, reflex flap (control) over 0.15..0.40 of the semi-span, B45-style tip fins (24 mm tall, chord = tip chord)."""
    G = ft.wing_geometry(SPAN, ar, taper, sweep)
    half = SPAN / 2
    cr, ct, tipx = G["cr"], G["ct"], G["tipx"]
    hp = [(0.0, 0.0), (tipx, half), (tipx + ct, half), (cr, 0.0)]
    return dict(family="T", name=name or f"T_AR{ar:g}_S{sweep:g}_L{taper:g}", params=dict(ar=ar, sweep=sweep, taper=taper), G=G, half=half, flap_mm=flap_mm,
                chord=lambda y: cr + (ct - cr) * y / half, le=lambda y: tipx * y / half,
                header=(G["area"] * 1e-6, G["mac"] * 1e-3, SPAN * 1e-3), cg_ref=G["mac_le_x"] + 0.25 * G["mac"], x_tail=tipx + 0.25 * ct,
                half_outline=hp, full_outline=mirror_poly(hp), S_mm2=G["area"], mac=G["mac"], mass_model="printed", wing_poly=mirror_poly(hp), tail_poly=None)


B_LEVELS = dict(ar=(7.0, 8.5, 10.0), hand=(-8.0, 0.0, 8.0), wrist=(0.05, 0.139, 0.30), tail=(0.10, 0.185, 0.30))
_GEO_CACHE = {}


def b_params(levels):
    """Swift family parameter dict from four levels (-1, 0, 1); (0, 0, 0, 0) returns s4.GLIDE itself (Swift A exactly)."""
    if tuple(levels) == (0, 0, 0, 0):
        return s4.GLIDE
    a, h, w, t = levels
    p = dict(s4.GLIDE, ar_target=B_LEVELS["ar"][a + 1], wrist_eta=B_LEVELS["wrist"][w + 1])
    p = s4.hand_shift(p, B_LEVELS["hand"][h + 1])
    return s4.with_tail_area(p, B_LEVELS["tail"][t + 1])


def b_name(levels):
    a, h, w, t = levels
    return "B_AR%g_H%+g_W%g_T%g" % (B_LEVELS["ar"][a + 1], B_LEVELS["hand"][h + 1], 100 * B_LEVELS["wrist"][w + 1], 100 * B_LEVELS["tail"][t + 1])


def spec_B(levels=(0, 0, 0, 0), p=None, name=None):
    p = p or b_params(levels)
    g = s4.build_geometry4(p)
    k = SPAN
    hp = [(x * k, y * k) for x, y in g["half_outline"]]
    wing_nodes = [(y * k, x * k, z * k, c * k) for y, x, z, c in bo.wing_sections(g)]
    tail_nodes = [(y * k, x * k, z * k, c * k) for y, x, z, c in v2.tail_sections2(g)]
    full = [(x * k, y * k) for x, y in g["full_outline"]]
    tail_half = [(g["x_h"] * k, g["wt0"] * k / 2), ((g["x_h"] + g["Lt"]) * k, g["St"] * k / 2), ((g["x_h"] + g["Lt"] + g["bulge"]) * k, g["St"] * k / 4), (g["tail_end_x"] * k, 0.0), (g["x_h"] * k, 0.0)]
    return dict(family="B", name=name or b_name(levels), params=dict(levels=tuple(levels), ar_target=p["ar_target"], hand_shift=p["le_hand_knots"][0][1] - s4.GLIDE["le_hand_knots"][0][1],
                                                                      wrist_eta=p["wrist_eta"], tail_area_frac=g["tail_area"] / g["S"], tail_len=p["tail_len"]),
                g=g, half=SPAN / 2, wing_nodes=wing_nodes, tail_nodes=tail_nodes, header=(g["S"] * k * k * 1e-6, g["mac"] * k * 1e-3, k * 1e-3), cg_ref=g["x_ref"] * k,
                x_tail=g["tail_centroid_x"] * k, half_outline=hp, full_outline=full, S_mm2=g["S"] * k * k, mac=g["mac"] * k, x_hinge=g["x_h"] * k, mass_model="printed",
                wing_poly=[(x * k, y * k) for x, y in g["main_full"]], tail_poly=mirror_poly(tail_half[:4]))


def zan_data():
    geo = json.loads((ZAN_DIR / "zanonia_geometry.json").read_text(encoding="utf-8"))
    with (ZAN_DIR / "zanonia_outline.csv").open(encoding="utf-8") as f:
        rows = list(csv.reader(f))[1:]
    return geo, [(float(r[0]), float(r[1]), float(r[2])) for r in rows]


ZAN_CURL = (0.30, 2.2)            # curl region (outer fraction of the semi-span) and tip incidence deg: RESULTS.md 'CG and curl together' row, CG 0 mm forward, SM 1.0 %, AVL trim CL 0.39


def spec_Z():
    """Zanonia scaled to 400 mm span (published size 155 mm): outline from the scan, curl 30 % / 2.2 deg, CG at the marked cross (SM about 1 % MAC)."""
    geo, tab = zan_data()
    f = SPAN / geo["span_in"]
    s_tip = tab[-1][0]
    f_reg, theta = ZAN_CURL
    s0 = (1.0 - f_reg) * s_tip
    nodes = []
    n_sec = 30
    for i in range(n_sec):
        s = s_tip * i / (n_sec - 1)
        xl, xt = ss_interp(tab, s)
        nodes.append((s * f, xl * f, 0.0, (xt - xl) * f, -theta * max(0.0, (s - s0) / (s_tip - s0))))
    hp = [(xl * f, s * f) for s, xl, xt in tab] + [(xt * f, s * f) for s, xl, xt in reversed(tab)]
    hp = [(x, y) for x, y in hp]
    # right half polygon, root edge on y = 0: leading edge out, trailing edge back
    le = [(xl * f, s * f) for s, xl, xt in tab]
    te = [(xt * f, s * f) for s, xl, xt in reversed(tab)]
    hp = le + te
    S = geo["area_in2"] * f * f
    return dict(family="Z", name="Zanonia_400", params=dict(curl=ZAN_CURL), geo=geo, half=SPAN / 2, wing_nodes=nodes, tail_nodes=[], header=(S * 1e-6, geo["mac_in"] * f * 1e-3, SPAN * 1e-3),
                cg_ref=geo["cg_x_in"] * f, cg_fixed=geo["cg_x_in"] * f, x_tail=hp and max(p[0] for p in hp), half_outline=hp, full_outline=mirror_poly(hp), S_mm2=S, mac=geo["mac_in"] * f,
                mass_model="zan", wing_poly=mirror_poly(hp), tail_poly=None, areal_g_m2=100.0, ctl=False)


def ss_interp(tab, s):
    for a, b in zip(tab, tab[1:]):
        if a[0] <= s <= b[0]:
            t = (s - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
            return a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2])
    return tab[-1][1], tab[-1][2]


def spec_B45_ref():
    sp = spec_T(4.5, 25.0, 0.5, flap_mm=b45.FLAP_CHORD_MM, name="B45_as_flown")
    sp.update(mass_model="ss_b45", cg_fixed=ss.B45_CG, delta_fixed=ss.B45_FLAP_UP)
    return sp


def spec_SW_ref():
    sp = spec_B((0, 0, 0, 0), name="SwiftA_as_flown")
    sp.update(mass_model="ss_sw", cg_fixed=ss.SW_CG, delta_fixed=0.0, tail_ainc=ss.SW_BEND)
    return sp


# ================================================================================================ AVL decks (explicit halves, no YDUPLICATE)
def fin_geometry_B(spec, pct):
    """Centre fin at the tail end sized pct % of S (fin_kit_v4 'tail' option), mm."""
    f = fk.fin_geometry(spec["g"], "tail", pct / 100.0)
    return dict(c=f["c"] * SPAN, h=f["h"] * SPAN, x_le=f["x_le"] * SPAN, x_te=f["x_te"] * SPAN, capped=f["capped"], area=f["c"] * f["h"] * SPAN * SPAN)


def tip_fin_dims(spec):
    return dict(h=FIN_H, c=spec["G"]["ct"] if "G" in spec else 0.0)


def kink_y(spec):
    """Kink station of the outer-panel dihedral, mm: 0.6 of the semi-span; for outlines with many nodes (B, Z) snapped to the nearest existing node (AVL cannot panel a sliver interval)."""
    if "yk" not in spec:
        half = spec["half"]
        yk = KINK * half
        if spec["family"] in ("B", "Z"):
            yk = min((n[0] for n in spec["wing_nodes"]), key=lambda y: abs(y - yk))
        spec["yk"] = yk
    return spec["yk"]


def rot_outer(y, dih, yk):
    """Outer-panel dihedral: points beyond the kink rotate about the kink line; returns (y, z)."""
    if dih == 0 or y <= yk:
        return y, 0.0
    d = y - yk
    return yk + d * math.cos(math.radians(dih)), d * math.sin(math.radians(dih))


def wing_sections_right(spec, st, ainc):
    """Right-half wing sections (ss.sec dicts, mm) with the outer-panel dihedral applied."""
    half, dih = spec["half"], st.get("dih", 0.0)
    yk = kink_y(spec)
    out = []
    if spec["family"] == "T":
        ys = [0.0, FLAP_Y[0] * half, FLAP_Y[1] * half] + ([yk] if dih != 0 else []) + [half]
        y1, y2 = ys[1], ys[2]
        ns = {0: max(2, round(y1 / 5)), 1: 8, 2: round(((yk if dih != 0 else half) - y2) / 8)}
        if dih != 0:
            ns[3] = round((half - yk) / 8)
            ns[4] = 0
        else:
            ns[3] = 0
        for i, y0 in enumerate(ys):
            flap = i in (1, 2)
            c = spec["chord"](y0)
            fm = spec["flap_mm"] if spec.get("flap_mm") else FLAP_FRAC * c
            cs = [("flapR", b45.GAIN, 1 - fm / c)] if flap else []
            y, z = rot_outer(y0, dih, yk)
            out.append(ss.sec(spec["le"](y0), y, z, c, ainc, ns[i], cs))
        return out
    nodes = [(y, x, z, c, a) for (y, x, z, c, a) in spec["wing_nodes"]] if spec["family"] == "Z" else [(y, x, z, c, 0.0) for (y, x, z, c) in spec["wing_nodes"]]
    if dih != 0 and not any(abs(n[0] - yk) < 1e-9 for n in nodes):
        for i in range(1, len(nodes)):
            if nodes[i][0] > yk:
                a, c_ = nodes[i - 1], nodes[i]
                t = (yk - a[0]) / (c_[0] - a[0])
                nodes.insert(i, tuple(a[j] + t * (c_[j] - a[j]) for j in range(5)))
                break
    for y0, x, z0, c, a0 in nodes:
        y, z = rot_outer(y0, dih, yk)
        out.append(ss.sec(x, y, z + z0, c, ainc + a0))
    return out


def tail_sections_right(spec, ainc):
    return [ss.sec(x, y, z, c, ainc, None, [("bendR", -1.0, 0.0)]) for y, x, z, c in spec["tail_nodes"]]


def tip_point(spec, st):
    return rot_outer(spec["half"], st.get("dih", 0.0), kink_y(spec))


def fin_sections_T(spec, st, toe):
    fin = st.get("fin")
    if not fin or fin[0] != "tip":
        return []
    cant = math.radians(fin[1])
    yt, zt = tip_point(spec, st)
    h, c = FIN_H, spec["G"]["ct"]
    return [ss.sec(spec["G"]["tipx"], yt + dz * math.sin(cant), zt + dz * math.cos(cant), c, toe) for dz in (h / 2, -h / 2)]


def build_deck(spec, st, cg, zcg, wing_inc=(0.0, 0.0), fin_toe=(0.0, 0.0), name="shp"):
    S, mac, b = spec["header"]
    s = f"{name}\n0.0\n0 0 0.0\n{S:.9f} {mac:.9f} {b:.9f}\n{cg / 1000:.9f} 0.0 {zcg / 1000:.9f}\n"
    wr = wing_sections_right(spec, st, wing_inc[0])
    wl = ss.mirror(wing_sections_right(spec, st, wing_inc[1]))
    fam = spec["family"]
    if fam == "T":
        s += ss.surface_text("WingR", f"{ft.AVL_NCHORD} 1.0", wr, 1) + ss.surface_text("WingL", f"{ft.AVL_NCHORD} 1.0", wl, 1)
        fr, fl = fin_sections_T(spec, st, fin_toe[0]), fin_sections_T(spec, st, fin_toe[1])
        if fr:
            s += ss.surface_text("FinR", "5 1.0 8 -2.0", fr, 3) + ss.surface_text("FinL", "5 1.0 8 -2.0", ss.mirror(fl), 3)
        return s
    hdr = f"{v2.AVL_NCHORD} 1.0 {v2.AVL_NSPAN} {v2.AVL_SPACE}"
    s += ss.surface_text("WingR", hdr, wr, 1) + ss.surface_text("WingL", hdr, wl, 1)
    if fam == "B":
        a0 = spec.get("tail_ainc", 0.0)
        tr, tl = tail_sections_right(spec, -a0), ss.mirror(tail_sections_right(spec, -a0))
        s += ss.surface_text("TailR", "6 1.0 12 -2.0", tr, 2) + ss.surface_text("TailL", "6 1.0 12 -2.0", tl, 2)
        fin = st.get("fin")
        if fin and fin[0] == "centre":
            fg = fin_geometry_B(spec, fin[1])
            secs = [ss.sec(fg["x_le"], 0.0, z, fg["c"], fin_toe[0]) for z in (0.0, fg["h"])]
            s += ss.surface_text("CentreFin", "5 1.0 8 -2.0", secs, 5)
    return s


# ================================================================================================ masses (printed sheet from sheet_builder; foam references from straight_smooth_v1)
_KIT = {}


def sheet_points(poly, n=110):
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    pts = []
    for i in range(n):
        x = min(xs) + (i + 0.5) * (max(xs) - min(xs)) / n
        for j in range(n):
            y = min(ys) + (j + 0.5) * (max(ys) - min(ys)) / n
            if fk._inside(poly, x, y):
                pts.append((x, y))
    return pts


def svg_half(spec):
    pts = " ".join(f"{x:.5f},{y:.5f}" for x, y in spec["half_outline"])
    return f"<svg xmlns='http://www.w3.org/2000/svg'><polygon points='{pts}'/></svg>\n"


def kit_config(spec, outline_path, layers=2):
    cfg = dict(name=spec["name"], span_mm=SPAN, source=dict(type="svg", path=str(outline_path), kind="half", flip_y=False), sheet=dict(n_layers=layers), mass=dict(target_cg_mm=None),
               tail=dict(bend_deg=0.0), fin=dict(type="none"))
    if spec["family"] == "B":
        cfg["source"]["hinge_x_mm"] = spec["x_hinge"]
    return sb.load_config(cfg)


def kit_info(spec, work: Path):
    """sheet_builder bare-kit masses for the outline (2 layers x 0.1 mm, DEFAULT bead / root strip / rail, no fins, dihedral 0, tail flat): CALCULATED from the STL volumes.  Cached per outline."""
    key = spec["name"]
    if key in _KIT:
        return _KIT[key]
    od = work / "outlines"
    od.mkdir(parents=True, exist_ok=True)
    pth = od / f"{spec['name']}.svg"
    pth.write_text(svg_half(spec), encoding="utf-8")
    try:
        r = sb.build_kit(kit_config(spec, pth), log=lambda *a: None)
        rows = r["parts_rows"] + r["glue_rows"]
        m_w = sum(m for n, m, x in rows if n.startswith("wing piece"))
        rail = [(m, x) for n, m, x in rows if n == "rail"][0]
        glue = sum(m for n, m, x in rows if "glue" in n)
        fu = r["wing"]["fit_unsplit"]
        info = dict(ok=True, m_wing=m_w, m_rail=rail[0], cx_rail=rail[1], m_glue=glue, station=r["station"], fits=bool(fu["fits"]), fit_w=fu["width"], fit_h=fu["height"],
                    pieces=len(r["wing"]["pieces"]), x_nose=r["x_nose"], notes=[n for n in r["notes"] if n.startswith("FLAG")])
    except Exception as e:                                                   # fall back to an area estimate, flagged
        a = abs(poly_area(spec["full_outline"]))
        fit = sb.fit_search([tuple(p) for p in spec["half_outline"]], BED, 0.5)
        info = dict(ok=False, error=repr(e), m_wing=a * 0.2 * PLA_RHO * 1e-3 * 1.35, m_rail=2.5, cx_rail=spec["cg_ref"], m_glue=0.3, station=-30.0, fits=bool(fit["fits"]), fit_w=fit["width"],
                    fit_h=fit["height"], pieces=1, x_nose=min(p[0] for p in spec["half_outline"]), notes=["FLAG: sheet_builder failed, area estimate used"])
    _KIT[key] = info
    return info


def fin_mass_T(spec):
    return FIN_H * spec["G"]["ct"] * TIP_FIN_T_MM * PLA_RHO * 1e-3          # g, one fin


def mass_printed(spec, st, cg, work: Path):
    """All-up mass model of a printed glider with the CG at `cg`: sheet points (kit wing mass spread over the planform, outer panel rotated by the dihedral), rail + glue lumped at the rail, fins,
    lead ballast at the kit's station (nose) sized for `cg`.  ASSUMED: kit built flat; point-mass inertia."""
    kit = kit_info(spec, work)
    if "pts" not in spec:
        spec["pts"] = sheet_points(spec["full_outline"])
    pts = spec["pts"]
    me = kit["m_wing"] / len(pts)
    half, dih = spec["half"], st.get("dih", 0.0)
    yk = kink_y(spec)
    items = []
    for x, y in pts:
        yy, zz = rot_outer(abs(y), dih, yk)
        items.append(ss.item(me, x, math.copysign(yy, y), zz))
    items.append(ss.item(kit["m_rail"] + kit["m_glue"], kit["cx_rail"], 0.0, 0.0))
    fin = st.get("fin")
    if fin and fin[0] == "tip":
        mf = fin_mass_T(spec)
        yt, zt = tip_point(spec, st)
        c, h = spec["G"]["ct"], FIN_H
        for sgn in (1, -1):
            items.append(ss.item(mf + 0.05, spec["G"]["tipx"] + c / 2, sgn * yt, zt, mf * h * h / 12, mf * (h * h + c * c) / 12, mf * c * c / 12))
    elif fin and fin[0] == "centre":
        fg = fin_geometry_B(spec, fin[1])
        mf = fg["area"] * TIP_FIN_T_MM * PLA_RHO * 1e-3 * 1.15 + 0.05
        items.append(ss.item(mf, fg["x_le"] + fg["c"] / 2, 0.0, fg["h"] / 2, mf * fg["h"] ** 2 / 12, 0.0, mf * fg["c"] ** 2 / 12))
    M0 = sum(i["m"] for i in items)
    cg0 = sum(i["m"] * i["x"] for i in items) / M0
    station = kit["station"]
    mb = M0 * (cg - cg0) / (station - cg)
    aft = False
    if mb < 0:                                                               # bare CG already ahead of the target: aft ballast at the tail end
        aft = True
        station = max(p[0] for p in spec["half_outline"]) - 5.0
        mb = M0 * (cg - cg0) / (station - cg)
    items.append(ss.item(mb, station, 0.0, 0.0))
    a = ss.assemble(items)
    a.update(M0_g=M0, cg0=cg0, ballast_g=mb, station=station, aft_ballast=aft, kit_ok=kit["ok"])
    return a


def mass_zan(spec, cg):
    if "pts" not in spec:
        spec["pts"] = sheet_points(spec["full_outline"], 100)
    area = abs(poly_area(spec["full_outline"]))
    m = area * 1e-6 * spec["areal_g_m2"]
    pts = spec["pts"]
    items = [ss.item(m / len(pts), x, y, 0.0) for x, y in pts]
    M0 = m
    cg0 = sum(i["m"] * i["x"] for i in items) / M0
    nose = min(p[0] for p in spec["half_outline"])
    station = nose + 3.0 if cg < cg0 else max(p[0] for p in spec["half_outline"]) - 3.0
    mb = M0 * (cg - cg0) / (station - cg)
    items.append(ss.item(mb, station, 0.0, 0.0))
    a = ss.assemble(items)
    a.update(M0_g=M0, cg0=cg0, ballast_g=mb, station=station, aft_ballast=bool(cg > cg0), kit_ok=True)
    return a


def mass_for(spec, st, cg, work):
    mm = spec["mass_model"]
    if mm == "ss_b45":
        return ss.b45_mass(cg, 0.0, 1.0)
    if mm == "ss_sw":
        return ss.sw_mass(cg, 0.0, False)
    if mm == "zan":
        return mass_zan(spec, cg)
    return mass_printed(spec, st, cg, work)


# ================================================================================================ evaluation
def lin2(CL, Cm):
    """Linear model about the reference CG from the four states (alpha, delta) = (0,0), (4,0), (0,10), (4,10)."""
    return dict(c0=CL[0], ca=(CL[1] - CL[0]) / 4.0, cd=(CL[2] - CL[0]) / 10.0, m0=Cm[0], ma=(Cm[1] - Cm[0]) / 4.0, md=(Cm[2] - Cm[0]) / 10.0)


def newton_step(L, s, ec, em, cl_t, delta_fixed=None):
    """(alpha, delta) from the linear model with offsets (ec, em) at the CG shift s = (cg - cg_ref)/MAC.  delta_fixed: solve alpha only for Cm = 0."""
    if delta_fixed is not None:
        d = delta_fixed
        cl = L["c0"] + L["cd"] * d + ec
        rhs = -(L["m0"] + L["md"] * d + cl * s + em)
        return rhs / (L["ma"] + L["ca"] * s), d
    a11, a12 = L["ca"], L["cd"]
    a21, a22 = L["ma"] + L["ca"] * s, L["md"] + L["cd"] * s
    r1 = cl_t - L["c0"] - ec
    r2 = -(L["m0"] + L["c0"] * s + em + ec * s)
    det = a11 * a22 - a12 * a21
    return (r1 * a22 - a12 * r2) / det, (a11 * r2 - a21 * r1) / det


def oper_cmds(alpha, d1, d2, ctl=True):
    return ([f"D1 D1 {d1:.6f}", f"D2 D2 {d2:.6f}"] if ctl else []) + [f"A A {alpha:.6f}"]


def lat_modes(rows):
    """Roll, spiral and oscillatory-mode data from the 4x4 lateral block; overdamped (no complex pair) gets zeta = 1."""
    idx = (4, 5, 6, 7)
    a = [[rows[i][j] for j in idx] for i in idx]
    z = fk.eig_roots(a)
    real = sorted(r.real for r in z if abs(r.imag) < 1e-6 * max(1.0, abs(r)))
    cplx = [r for r in z if r.imag > 1e-6 * max(1.0, abs(r))]
    out = dict(eigs=[(r.real, r.imag) for r in z])
    if cplx:
        dr = cplx[0]
        out.update(osc=True, dr_wn=abs(dr), dr_zeta=-dr.real / abs(dr), dr_re=dr.real, dr_im=dr.imag)
        out["roll"], out["spiral"] = (real[0], real[-1]) if real else (None, None)
    else:
        out.update(osc=False, dr_wn=0.0, dr_zeta=1.0, dr_re=0.0, dr_im=0.0, roll=real[0], spiral=real[-1])
    return out


def rawdon(spec, st, cg, cl):
    """Rawdon spiral number SS = EDA(deg) * (lv / b) / CL (computed, not part of the rule).  EDA = outer-panel dihedral weighted by the share of int(c y dy) outboard of the kink;
    lv = distance from the CG to the quarter chord of the fin if present, else of the tail (B) / of the tip chord (T)."""
    half, dih = spec["half"], st.get("dih", 0.0)
    n = 400
    ys = [(i + 0.5) * half / n for i in range(n)]
    if spec["family"] == "T":
        cf = spec["chord"]
    elif spec["family"] == "B":
        cf = lambda y: spec["g"]["chord_fn"](y / SPAN) * SPAN
    else:
        cf = lambda y: 1.0
    w_all = sum(cf(y) * y for y in ys)
    w_out = sum(cf(y) * y for y in ys if y > kink_y(spec))
    eda = dih * (w_out / w_all if w_all else 0.0)
    fin = st.get("fin")
    if spec["family"] == "T":
        xv = spec["G"]["tipx"] + 0.25 * spec["G"]["ct"]
    elif fin and fin[0] == "centre":
        fg = fin_geometry_B(spec, fin[1])
        xv = fg["x_le"] + 0.25 * fg["c"]
    else:
        xv = spec["x_tail"]
    lv = xv - cg
    return dict(eda=eda, lv=lv, ss=eda * (lv / SPAN) / cl if cl else 0.0)


def evaluate(spec, st, work: Path, log=None):
    """One configuration: trim (target mode: SM 12 % MAC, trim CL = st['cl'] by reflex flap / tail bend; fixed mode: given CG and surface setting), gamma, modes (theta0 = alpha + gamma, Ixz),
    asymmetries (a) differential surface through AVL's control columns, (b) wing-half incidence, (c) fin toe; closed-form r_hat; twist tolerances."""
    t0 = time.time()
    has_ctl = spec.get("ctl", True)
    oc = lambda a, d1, d2: oper_cmds(a, d1, d2, has_ctl)
    fixed = "cg_fixed" in spec
    wd = work / st["name"]
    mac = spec["mac"]
    S_m2 = spec["header"][0]
    cl_t = st.get("cl", CL_TARGET)
    cg_ref = spec["cg_ref"]
    # ---- 1. linear model about the reference CG (zcg 0)
    deck0 = build_deck(spec, st, cg_ref, 0.0)
    if spec["family"] == "T":
        d_hi = 10.0
    else:
        d_hi = 10.0
    ctl2 = lambda d: (d, d)
    states = [(0.0, 0.0), (4.0, 0.0), (0.0, d_hi), (4.0, d_hi)]
    base_d = spec.get("delta_fixed", 0.0)
    stages = [(oc(a, *ctl2(d)), f"s{i}") for i, (a, d) in enumerate(states)]
    o0 = ss.avl_session(wd / "lin", "shp", deck0, stages)
    L = lin2([o0[f"s{i}"]["CL"] for i in range(4)], [o0[f"s{i}"]["Cm"] for i in range(4)])
    if fixed:                                              # linearise around the fixed setting instead: alpha-only trim with the given delta
        pass
    xnp = sum(o0[f"s{i}"]["Xnp"] for i in range(4)) / 4 * 1000.0
    cg = spec["cg_fixed"] if fixed else xnp - SM_TARGET * mac
    asm = mass_for(spec, st, cg, work)
    zcg = asm["cg"][2]
    ec = em = 0.0
    alpha = delta = None
    it_used = 0
    conv = False
    for it in range(6):
        it_used = it + 1
        s = (cg - cg_ref) / mac
        alpha, delta = newton_step(L, s, ec, em, cl_t, base_d if fixed else None)
        deckz = build_deck(spec, st, cg, zcg)
        out = ss.avl_session(wd / "trim", "shp", deckz, [(oc(alpha, delta, delta), "t")])["t"]
        xnp_now = out["Xnp"] * 1000.0
        sm_now = (xnp_now - cg) / mac
        ok_cm = abs(out["Cm"]) < (3e-4 if fixed else 1.5e-4)
        ok_cl = fixed or abs(out["CL"] - cl_t) < 0.002
        ok_sm = fixed or abs(sm_now - SM_TARGET) < 0.0005
        if ok_cm and ok_cl and ok_sm:
            conv = True
            break
        ec = out["CL"] - (L["c0"] + L["ca"] * alpha + L["cd"] * delta)
        em = out["Cm"] - (L["m0"] + L["ma"] * alpha + L["md"] * delta + out["CL"] * s)
        if not fixed and not ok_sm:
            cg = xnp_now - SM_TARGET * mac
            asm = mass_for(spec, st, cg, work)
            zcg = asm["cg"][2]
    cl_trim = out["CL"]
    V = math.sqrt(2 * asm["M_g"] * 1e-3 * ss.GRAV / (ss.RHO * S_m2 * cl_trim))
    gamma = GAMMA_REF if (fixed or spec["family"] == "Z") else -math.degrees(math.atan2(CD0 + out["CD"], cl_trim))
    # ---- 2. main run: base state + differential surface, mode matrix at theta0 = alpha + gamma with Ixz
    extra = [f"E {alpha + gamma:.5f}", f"IZX {-asm['jxz_kgm2']:.6e}"]
    stg = ([(oc(alpha, delta + AMP * ASYM / 2, delta - AMP * ASYM / 2), "a_ctl")] if has_ctl else []) + [(oc(alpha, delta, delta), "base")]
    main = ss.avl_session(wd / "main", "shp", deckz, stg, mass=ss.mass_file(asm), V=V, mode_extra=extra)
    rows = ss.parse_sysmat_full(wd / "main" / "sys.out")
    modes = lat_modes([r[:12] for r in rows])
    A4, B4 = ss.lat_blocks(rows)
    b0 = main["base"]
    asym = {"a": ss.asym_entry(main["a_ctl"], b0)} if has_ctl else {}
    wb = ss.avl_session(wd / "b", "shp", build_deck(spec, st, cg, zcg, wing_inc=(AMP * ASYM / 2, -AMP * ASYM / 2)), [(oc(alpha, delta, delta), "x")])["x"]
    asym["b"] = ss.asym_entry(wb, b0)
    fin = st.get("fin")
    if fin and ((spec["family"] == "T" and fin[0] == "tip") or (spec["family"] == "B" and fin[0] == "centre")):
        A = AMP * ASYM / 2
        kw = (lambda x: dict(fin_toe=(x, 0.0)))
        cp = ss.avl_session(wd / "c_plus", "shp", build_deck(spec, st, cg, zcg, **kw(+A)), [(oc(alpha, delta, delta), "x")])["x"]
        cm_ = ss.avl_session(wd / "c_minus", "shp", build_deck(spec, st, cg, zcg, **kw(-A)), [(oc(alpha, delta, delta), "x")])["x"]
        odd = {k_: (cp[k_] - cm_[k_]) / 2.0 for k_ in ("Cl", "Cn", "CY", "CL", "Cm")}
        sg = 1.0 if odd["CY"] < 0 else -1.0
        asym["c"] = ss.asym_entry(odd, {k_: 0.0 for k_ in odd}, sg * AMP / A)
    d = {k: v for k, v in b0.items()}
    b_m = spec["header"][2]
    tw = {}
    for k, v in asym.items():
        rs, rh, den, beta = ss.rhat_gamma(d["Clb"], d["Cnb"], d["Clr"], d["Cnr"], d["Clp"], d["Cnp"], v["dCl"], v["dCn"], gamma)
        v.update(rhat=rh, rhat_s=rs, D=den, beta=beta, tw30=(Z_B / abs(rh)) * ASYM if rh else float("inf"), bank=math.degrees(math.atan(2 * rh * V * V / (b_m * ss.GRAV))))
        tw[k] = v["tw30"]
    # oscillation mode shape and label (information)
    shape = None
    cplx = [e for e in modes["eigs"] if e[1] > 1e-9]
    if cplx:
        shape = ss.mode_shape(A4, V, complex(*cplx[0]))
    sp = modes["spiral"]
    sp_text = "stable" if sp < 0 else f"doubles in {math.log(2) / sp:.1f} s"
    rw = rawdon(spec, st, cg, cl_trim)
    res = dict(name=st["name"], family=spec["family"], spec_name=spec["name"], params=spec["params"], feat={k: v for k, v in st.items() if k != "name"}, fixed=fixed,
               cg=cg, zcg=zcg, xnp=xnp_now, mac=mac, sm_pct=100 * (xnp_now - cg) / mac, alpha=alpha, delta=delta, cl_trim=cl_trim, cm_trim=out["Cm"], converged=bool(conv), iterations=it_used,
               gamma=gamma, CDi=out["CD"], V=V, Re=V * mac * 1e-3 / 1.5e-5, mass_g=asm["M_g"], ballast_g=asm["ballast_g"], aft_ballast=bool(asm.get("aft_ballast")),
               wing_loading_N_m2=asm["M_g"] * 1e-3 * ss.GRAV / S_m2, inertia=asm["I_kgm2"], jxz=asm["jxz_kgm2"], deriv={k: v for k, v in b0.items()},
               modes={k: v for k, v in modes.items() if k != "eigs"}, shape=shape, asym=asym, tw30=tw, rawdon=rw, seconds=time.time() - t0, spiral_text=sp_text,
               trim_feasible=bool(abs(delta) <= DELTA_MAX and conv))
    kit = kit_info(spec, work)
    res["bed_fits"] = bool(kit["fits"])
    res["bed_w_h"] = (kit["fit_w"], kit["fit_h"])
    res["kit_ok"] = bool(kit["ok"])
    res["mass_tag"] = ("CALCULATED (sheet_builder STL volumes, 2 x 0.1 mm, default bead/rail) + ASSUMED (point-mass distribution, lead ballast)" if spec["mass_model"] == "printed"
                       else "ASSUMED (foam 300 g/m2 / plate 100 g/m2)")
    return res


# ================================================================================================ grid, rule, ranking
T_AR, T_SWEEP, T_TAPER = (3.5, 4.5, 6.0), (15.0, 25.0, 35.0), (0.3, 0.5, 0.8)
S2_DIH = (0.0, 10.0, 20.0)
S2_FIN = {"T": (None, ("tip", 0.0), ("tip", 20.0)), "B": (None, ("centre", 2.0), ("centre", 4.0))}
FIN_TAG = {None: "nofin", ("tip", 0.0): "tip0", ("tip", 20.0): "tip20", ("centre", 2.0): "c2", ("centre", 4.0): "c4"}
FIN_TEXT = {None: "no fin", ("tip", 0.0): "tip fins cant 0", ("tip", 20.0): "tip fins cant 20", ("centre", 2.0): "centre fin 2% S", ("centre", 4.0): "centre fin 4% S"}
CAP_CONFIGS = 300


def b_levels_all():
    import itertools
    out = []
    for a, h, w in itertools.product((-1, 0, 1), repeat=3):
        out.append((a, h, w, ((a + h + w + 1) % 3) - 1))
    return out


def outline_specs():
    """54 Stage-1 outlines (27 T, 27 B) as spec dicts."""
    import itertools
    L = [spec_T(ar, sw_, tp) for ar, sw_, tp in itertools.product(T_AR, T_SWEEP, T_TAPER)]
    L += [spec_B(lv) for lv in b_levels_all()]
    return L


def ref_specs():
    return [(spec_B45_ref(), dict(name="B45_as_flown", fin=("tip", 0.0), dih=0.0)), (spec_SW_ref(), dict(name="SwiftA_as_flown", fin=None, dih=0.0)), (spec_Z(), dict(name="Zanonia_400", fin=None, dih=0.0))]


def stage1_st(spec):
    return dict(name=spec["name"], fin=("tip", 0.0) if spec["family"] == "T" else None, dih=0.0, cl=CL_TARGET)


def stage2_st(spec, dih, fin, cl):
    return dict(name=f"{spec['name']}__d{dih:g}__{FIN_TAG[fin]}__cl{cl:.2f}", fin=fin, dih=dih, cl=cl, outline=spec["name"])


def pitch_margin(res):
    return ALPHA_BREAK - res["alpha"]


def cancel_ratio(res):
    """|Cnb dCl - Clb dCn| / (|Cnb dCl| + |Clb dCn|) for the wing-half incidence asymmetry (b): the numerator of r_hat.  Near 0 the two terms cancel and the twist tolerance is an artefact of the cancellation
    (B45 and Swift A as flown: about 0.6).  Post-hoc diagnostic, not part of the pre-stated rule."""
    d, a = res["deriv"], res["asym"]["b"]
    t1, t2 = d["Cnb"] * a["dCl"], d["Clb"] * a["dCn"]
    return abs(t1 - t2) / (abs(t1) + abs(t2)) if (abs(t1) + abs(t2)) else 0.0


def judge(res, tw_base):
    m, d = res["modes"], res["deriv"]
    sp = m["spiral"]
    r1 = bool(sp < 0 or sp < math.log(2) / SPIRAL_T2_MIN)
    r2 = bool(m["dr_zeta"] >= DR_ZETA_MIN and d["Cnb"] > 0 and m["roll"] is not None and m["roll"] < 0)
    r3 = bool(res["tw30"]["b"] >= TW_FACTOR * tw_base)
    r4 = bool(SM_BAND[0] <= res["sm_pct"] <= SM_BAND[1] and res["alpha"] <= ALPHA_MAX and res["trim_feasible"])
    r5 = bool(res["bed_fits"])
    parts = (r1, r2, r3, r4, r5)
    return dict(r1=r1, r2=r2, r3=r3, r4=r4, r5=r5, n_pass=sum(parts), candidate=bool(all(parts)) and not res["fixed"], pitch_margin=pitch_margin(res))


def avg_ranks(vals):
    idx = sorted(range(len(vals)), key=lambda i: vals[i])
    r = [0.0] * len(vals)
    i = 0
    while i < len(idx):
        j = i
        while j + 1 < len(idx) and vals[idx[j + 1]] == vals[idx[i]]:
            j += 1
        for k in range(i, j + 1):
            r[idx[k]] = (i + j) / 2.0 + 1.0
        i = j + 1
    return r


def spearman(x, y):
    if len(x) < 3:
        return None
    rx, ry = avg_ranks(x), avg_ranks(y)
    mx, my = sum(rx) / len(rx), sum(ry) / len(ry)
    sxx = sum((a - mx) ** 2 for a in rx)
    syy = sum((b - my) ** 2 for b in ry)
    if sxx == 0 or syy == 0:
        return None
    return sum((a - mx) * (b - my) for a, b in zip(rx, ry)) / math.sqrt(sxx * syy)


def select_stage2(res1, per_family=5):
    """Pre-stated rule: filter F = trim feasible, Cnb > 0, fits bed; per family rank by twist tolerance and pitch margin, score = rank sum (ties: larger twist tolerance), 5 best each."""
    chosen, notes = [], []
    for fam in ("T", "B"):
        mem = [r for r in res1 if r["family"] == fam]
        F = [r for r in mem if r["trim_feasible"] and r["deriv"]["Cnb"] > 0 and r["bed_fits"]]
        def score_of(group):
            tw = [-r["tw30"]["b"] for r in group]
            pm = [-pitch_margin(r) for r in group]
            rt, rp = avg_ranks(tw), avg_ranks(pm)
            return sorted(zip(group, [a + b for a, b in zip(rt, rp)]), key=lambda t: (t[1], -t[0]["tw30"]["b"]))
        pick = [g for g, sc in score_of(F)][:per_family]
        if len(pick) < per_family:
            rest = [r for r in mem if r not in pick]
            fill = [g for g, sc in score_of(rest)][:per_family - len(pick)]
            pick += fill
            notes.append(f"family {fam}: only {len(F)} outlines in F, filled with {[r['name'] for r in fill]}")
        chosen += pick
    return chosen, notes


def pareto_front(points):
    """points: list of (x, y, key), larger better in both; returns the keys of the non-dominated points."""
    keys = []
    for i, (x, y, k) in enumerate(points):
        if not any((x2 >= x and y2 >= y) and (x2 > x or y2 > y) for j, (x2, y2, k2) in enumerate(points) if j != i):
            keys.append(k)
    return keys


def pick_top3(rows):
    """rows: result dicts with 'rule'. Candidates by TW30; if fewer than 3, near-misses by (n_pass, TW30); one per outline."""
    el = [r for r in rows if not r["fixed"] and r["trim_feasible"]]
    cand = sorted([r for r in el if r["rule"]["candidate"]], key=lambda r: -r["tw30"]["b"])
    near = sorted([r for r in el if not r["rule"]["candidate"]], key=lambda r: (-r["rule"]["n_pass"], -r["tw30"]["b"]))
    out, seen = [], set()
    for lab, grp in (("CANDIDATE", cand), ("NEAR-MISS", near)):
        for r in grp:
            o = r.get("outline", r["spec_name"])
            if o in seen:
                continue
            seen.add(o)
            out.append((lab, r))
            if len(out) == 3:
                return out
    return out


def run_one(spec, st, work: Path, cache: Path, fresh=False, log=print):
    cp = cache / (st["name"] + ".pkl")
    if cp.is_file() and not fresh:
        return pickle.load(open(cp, "rb"))
    res = evaluate(spec, st, work)
    res["outline"] = st.get("outline", spec["name"])
    cache.mkdir(parents=True, exist_ok=True)
    pickle.dump(res, open(cp, "wb"))
    return res


# ================================================================================================ the run
def compute(work: Path, fresh=False, log=print, stage="all", refs_only=False):
    t0 = time.time()
    cache = work / "cache"
    fails = []

    def go(spec, st):
        t = time.time()
        r = None
        for attempt in range(4):
            try:
                r = run_one(spec, st, work, cache, fresh)
                break
            except OSError as e:                                                      # transient Windows resource errors when spawning AVL: retry
                log(f"retry {st['name']} after {e!r}")
                time.sleep(5.0 * (attempt + 1))
            except Exception as e:                                                    # an AVL / geometry failure is recorded, never silently dropped
                fails.append((st["name"], repr(e)))
                log(f"FAILED {st['name']}: {e!r}")
                return None
        if r is None:
            fails.append((st["name"], "OSError after 4 attempts"))
            return None
        log(f"{st['name']}: conv {r['converged']} alpha {r['alpha']:.2f} delta {r['delta']:+.2f} CL {r['cl_trim']:.3f} SM {r['sm_pct']:.2f} spiral {r['modes']['spiral']:+.4f} zeta {r['modes']['dr_zeta']:+.3f} TW30b {r['tw30']['b']:.3f}  ({time.time() - t:.1f} s)")
        return r
    refs = [go(sp, st) for sp, st in ref_specs()]
    if refs_only:
        return dict(refs=[r for r in refs if r], res1=[], res2=[], selected=[], notes=[], fails=fails, wall=time.time() - t0)
    specs = outline_specs()
    specmap = {sp["name"]: sp for sp in specs}
    res1 = [r for r in (go(sp, stage1_st(sp)) for sp in specs) if r]
    sel, notes = select_stage2(res1)
    n_total = len(refs) + len(res1) + len(sel) * len(S2_DIH) * 3 * len(CL_STAGE2)
    if n_total > CAP_CONFIGS:
        raise SystemExit(f"grid would be {n_total} configurations (cap {CAP_CONFIGS}); stopping")
    log(f"Stage 2 outlines: {[r['name'] for r in sel]}; notes {notes}; planned total {n_total}")
    res2 = []
    if stage in ("all", "2"):
        for r in sel:
            sp = specmap[r["spec_name"]]
            for dih in S2_DIH:
                for fin in S2_FIN[sp["family"]]:
                    for cl in CL_STAGE2:
                        x = go(sp, stage2_st(sp, dih, fin, cl))
                        if x:
                            res2.append(x)
    wall = sum(r["seconds"] for r in refs + res1 + res2 if r)
    return dict(refs=[r for r in refs if r], res1=res1, res2=res2, selected=[r["name"] for r in sel], notes=notes, fails=fails, wall=wall, wall_clock=time.time() - t0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(OUT))
    ap.add_argument("--work", default=str(Path(tempfile.gettempdir()) / "shape_sweep_v1_avl"))
    ap.add_argument("--stage", default="all")
    ap.add_argument("--refs-only", action="store_true")
    ap.add_argument("--fresh", action="store_true")
    ap.add_argument("--render", action="store_true", help="render outputs from the cached results (no AVL)")
    ap.add_argument("--compute-only", action="store_true")
    a = ap.parse_args()
    work, out = Path(a.work), Path(a.out)
    work.mkdir(parents=True, exist_ok=True)
    out.mkdir(parents=True, exist_ok=True)
    pk = work / "state.pkl"
    if a.render:
        state = pickle.load(open(pk, "rb"))
    else:
        state = compute(work, a.fresh, log=lambda m: print(m, flush=True), stage=a.stage, refs_only=a.refs_only)
        pickle.dump(state, open(pk, "wb"))
        print(f"computed: refs {len(state['refs'])}, stage 1 {len(state['res1'])}, stage 2 {len(state['res2'])}, failures {len(state['fails'])}, model wall {state['wall']:.0f} s", flush=True)
        if a.compute_only or a.refs_only:
            return
    write_outputs(state, out, work)



# ================================================================================================ bitmap font (5 x 7, upper case), canvas, GIF writer (stdlib LZW)
_G = {
    " ": [".....", ".....", ".....", ".....", ".....", ".....", "....."],
    "A": [".###.", "#...#", "#...#", "#####", "#...#", "#...#", "#...#"], "B": ["####.", "#...#", "#...#", "####.", "#...#", "#...#", "####."],
    "C": [".####", "#....", "#....", "#....", "#....", "#....", ".####"], "D": ["####.", "#...#", "#...#", "#...#", "#...#", "#...#", "####."],
    "E": ["#####", "#....", "#....", "####.", "#....", "#....", "#####"], "F": ["#####", "#....", "#....", "####.", "#....", "#....", "#...."],
    "G": [".####", "#....", "#....", "#.###", "#...#", "#...#", ".###."], "H": ["#...#", "#...#", "#...#", "#####", "#...#", "#...#", "#...#"],
    "I": [".###.", "..#..", "..#..", "..#..", "..#..", "..#..", ".###."], "J": ["..###", "...#.", "...#.", "...#.", "...#.", "#..#.", ".##.."],
    "K": ["#...#", "#..#.", "#.#..", "##...", "#.#..", "#..#.", "#...#"], "L": ["#....", "#....", "#....", "#....", "#....", "#....", "#####"],
    "M": ["#...#", "##.##", "#.#.#", "#.#.#", "#...#", "#...#", "#...#"], "N": ["#...#", "##..#", "##..#", "#.#.#", "#..##", "#..##", "#...#"],
    "O": [".###.", "#...#", "#...#", "#...#", "#...#", "#...#", ".###."], "P": ["####.", "#...#", "#...#", "####.", "#....", "#....", "#...."],
    "Q": [".###.", "#...#", "#...#", "#...#", "#.#.#", "#..#.", ".##.#"], "R": ["####.", "#...#", "#...#", "####.", "#.#..", "#..#.", "#...#"],
    "S": [".####", "#....", "#....", ".###.", "....#", "....#", "####."], "T": ["#####", "..#..", "..#..", "..#..", "..#..", "..#..", "..#.."],
    "U": ["#...#", "#...#", "#...#", "#...#", "#...#", "#...#", ".###."], "V": ["#...#", "#...#", "#...#", "#...#", "#...#", ".#.#.", "..#.."],
    "W": ["#...#", "#...#", "#...#", "#.#.#", "#.#.#", "##.##", "#...#"], "X": ["#...#", "#...#", ".#.#.", "..#..", ".#.#.", "#...#", "#...#"],
    "Y": ["#...#", "#...#", ".#.#.", "..#..", "..#..", "..#..", "..#.."], "Z": ["#####", "....#", "...#.", "..#..", ".#...", "#....", "#####"],
    "0": [".###.", "#...#", "#..##", "#.#.#", "##..#", "#...#", ".###."], "1": ["..#..", ".##..", "..#..", "..#..", "..#..", "..#..", ".###."],
    "2": [".###.", "#...#", "....#", "...#.", "..#..", ".#...", "#####"], "3": ["####.", "....#", "....#", ".###.", "....#", "....#", "####."],
    "4": ["...#.", "..##.", ".#.#.", "#..#.", "#####", "...#.", "...#."], "5": ["#####", "#....", "####.", "....#", "....#", "#...#", ".###."],
    "6": [".###.", "#....", "#....", "####.", "#...#", "#...#", ".###."], "7": ["#####", "....#", "...#.", "..#..", ".#...", ".#...", ".#..."],
    "8": [".###.", "#...#", "#...#", ".###.", "#...#", "#...#", ".###."], "9": [".###.", "#...#", "#...#", ".####", "....#", "....#", ".###."],
    ".": [".....", ".....", ".....", ".....", ".....", ".##..", ".##.."], ",": [".....", ".....", ".....", ".....", ".##..", "..#..", ".#..."],
    ":": [".....", ".##..", ".##..", ".....", ".##..", ".##..", "....."], "-": [".....", ".....", ".....", "#####", ".....", ".....", "....."],
    "+": [".....", "..#..", "..#..", "#####", "..#..", "..#..", "....."], "=": [".....", ".....", "#####", ".....", "#####", ".....", "....."],
    "/": ["....#", "....#", "...#.", "..#..", ".#...", "#....", "#...."], "%": ["##..#", "##..#", "...#.", "..#..", ".#...", "#..##", "#..##"],
    "(": ["...#.", "..#..", ".#...", ".#...", ".#...", "..#..", "...#."], ")": [".#...", "..#..", "...#.", "...#.", "...#.", "..#..", ".#..."],
    "_": [".....", ".....", ".....", ".....", ".....", ".....", "#####"], "'": ["..#..", "..#..", ".....", ".....", ".....", ".....", "....."],
    "<": ["...#.", "..#..", ".#...", "#....", ".#...", "..#..", "...#."], ">": [".#...", "..#..", "...#.", "....#", "...#.", "..#..", ".#..."],
    "~": [".....", ".....", ".#...", "#.#.#", "...#.", ".....", "....."], "^": ["..#..", ".#.#.", "#...#", ".....", ".....", ".....", "....."],
    "*": [".....", "#.#.#", ".###.", "#####", ".###.", "#.#.#", "....."], "`": [".##..", "#..#.", "#..#.", ".##..", ".....", ".....", "....."],
    "!": ["..#..", "..#..", "..#..", "..#..", "..#..", ".....", "..#.."], "?": [".###.", "#...#", "....#", "...#.", "..#..", ".....", "..#.."],
    "|": ["..#..", "..#..", "..#..", "..#..", "..#..", "..#..", "..#.."], "#": [".#.#.", "#####", ".#.#.", ".#.#.", "#####", ".#.#.", "....."],
}
PALETTE = [(0x14, 0x18, 0x1d), (0x1c, 0x22, 0x29), (0x33, 0x3c, 0x46), (0xdd, 0xe3, 0xea), (0x9a, 0xa7, 0xb4), (0xe8, 0x9a, 0x55), (0x6c, 0xc0, 0x8b), (0xff, 0x8a, 0x80),
           (0x7f, 0xb4, 0xff), (0x27, 0x31, 0x3c), (0x3a, 0x55, 0x6e), (0x2f, 0x6b, 0x4a), (0x7a, 0x3b, 0x3b), (0xc3, 0x9b, 0xff), (0x55, 0x60, 0x6b), (0xff, 0xff, 0xff)]
BG, PANEL, LINE, INK, MUTED, ACCENT, GOOD, BAD, BLUE, FILL, FILL_B, FILL_G, FILL_R, PURPLE, GRID, WHITE = range(16)


class Canvas:
    def __init__(self, w, h, bg=BG):
        self.w, self.h = w, h
        self.buf = bytearray([bg]) * (w * h)

    def px(self, x, y, c):
        if 0 <= x < self.w and 0 <= y < self.h:
            self.buf[y * self.w + x] = c

    def rect(self, x, y, w, h, c):
        for yy in range(max(0, y), min(self.h, y + h)):
            x0, x1 = max(0, x), min(self.w, x + w)
            if x1 > x0:
                self.buf[yy * self.w + x0:yy * self.w + x1] = bytes([c]) * (x1 - x0)

    def line(self, x0, y0, x1, y1, c, t=1):
        x0, y0, x1, y1 = int(round(x0)), int(round(y0)), int(round(x1)), int(round(y1))
        dx, dy = abs(x1 - x0), -abs(y1 - y0)
        sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
        err = dx + dy
        while True:
            for a in range(t):
                for b in range(t):
                    self.px(x0 + a, y0 + b, c)
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy

    def poly_outline(self, pts, c, t=1, closed=True):
        n = len(pts)
        for i in range(n if closed else n - 1):
            a, b = pts[i], pts[(i + 1) % n]
            self.line(a[0], a[1], b[0], b[1], c, t)

    def fill_poly(self, pts, c):
        ys = [p[1] for p in pts]
        y0, y1 = int(math.floor(min(ys))), int(math.ceil(max(ys)))
        n = len(pts)
        for y in range(max(0, y0), min(self.h, y1 + 1)):
            yc = y + 0.5
            xs = []
            for i in range(n):
                (xa, ya), (xb, yb) = pts[i], pts[(i + 1) % n]
                if (ya <= yc < yb) or (yb <= yc < ya):
                    xs.append(xa + (yc - ya) * (xb - xa) / (yb - ya))
            xs.sort()
            for k in range(0, len(xs) - 1, 2):
                xa, xb = int(round(xs[k])), int(round(xs[k + 1]))
                xa, xb = max(0, xa), min(self.w, xb)
                if xb > xa:
                    self.buf[y * self.w + xa:y * self.w + xb] = bytes([c]) * (xb - xa)

    def text(self, x, y, s, c=INK, scale=2):
        for ch in str(s).upper():
            g = _G.get(ch, _G["?"])
            for r, row in enumerate(g):
                for k, v in enumerate(row):
                    if v == "#":
                        self.rect(x + k * scale, y + r * scale, scale, scale, c)
            x += 6 * scale


def lzw_encode(data, min_bits=4):
    """GIF variable-width LZW; returns the code stream bytes (not yet split into sub-blocks)."""
    clear, eoi = 1 << min_bits, (1 << min_bits) + 1
    out = bytearray()
    acc = nb = 0
    width = min_bits + 1
    table = {}
    nxt = eoi + 1

    def emit(code, wd):
        nonlocal acc, nb
        acc |= code << nb
        nb += wd
        while nb >= 8:
            out.append(acc & 255)
            acc >>= 8
            nb -= 8
    emit(clear, width)
    prefix = data[0]
    for px in data[1:]:
        key = (prefix << 8) | px
        c = table.get(key)
        if c is not None:
            prefix = c
            continue
        emit(prefix, width)
        table[key] = nxt
        if nxt == (1 << width) and width < 12:
            width += 1
        nxt += 1
        if nxt >= 4095:
            emit(clear, width)
            table.clear()
            nxt = eoi + 1
            width = min_bits + 1
        prefix = px
    emit(prefix, width)
    emit(eoi, width)
    if nb:
        out.append(acc & 255)
    return bytes(out)


def sub_blocks(b):
    out = bytearray()
    for i in range(0, len(b), 255):
        ch = b[i:i + 255]
        out.append(len(ch))
        out += ch
    out.append(0)
    return bytes(out)


def gif_bytes(frames, w, h, delay_cs=60, palette=PALETTE):
    out = bytearray(b"GIF89a")
    out += w.to_bytes(2, "little") + h.to_bytes(2, "little") + bytes([0xF0 | (len(palette).bit_length() - 2), 0, 0])
    for r, g, b in palette:
        out += bytes([r, g, b])
    out += b"\x21\xFF\x0BNETSCAPE2.0\x03\x01\x00\x00\x00"
    for fr in frames:
        out += b"\x21\xF9\x04" + bytes([0x04, delay_cs & 255, delay_cs >> 8, 0, 0])
        out += b"\x2C" + (0).to_bytes(2, "little") * 2 + w.to_bytes(2, "little") + h.to_bytes(2, "little") + b"\x00"
        out += bytes([4]) + sub_blocks(lzw_encode(bytes(fr), 4))
    out += b"\x3B"
    return bytes(out)


def gif_decode(data):
    """Minimal GIF decoder (global palette, LZW, sub-blocks) for the self-check: returns (w, h, n_frames, [pixel index bytes per frame], delays_cs)."""
    assert data[:6] in (b"GIF89a", b"GIF87a")
    w, h = int.from_bytes(data[6:8], "little"), int.from_bytes(data[8:10], "little")
    flags = data[10]
    pos = 13
    if flags & 0x80:
        pos += 3 * (2 << (flags & 7))
    frames, delays = [], []
    while pos < len(data):
        b = data[pos]
        if b == 0x3B:
            break
        if b == 0x21:
            label = data[pos + 1]
            pos += 2
            if label == 0xF9:
                delays.append(int.from_bytes(data[pos + 2:pos + 4], "little"))
            while data[pos] != 0:
                pos += data[pos] + 1
            pos += 1
        elif b == 0x2C:
            fw, fh = int.from_bytes(data[pos + 5:pos + 7], "little"), int.from_bytes(data[pos + 7:pos + 9], "little")
            pos += 10
            mb = data[pos]
            pos += 1
            raw = bytearray()
            while data[pos] != 0:
                raw += data[pos + 1:pos + 1 + data[pos]]
                pos += data[pos] + 1
            pos += 1
            frames.append(lzw_decode(bytes(raw), mb, fw * fh))
        else:
            raise ValueError("bad GIF block")
    return w, h, len(frames), frames, delays


def lzw_decode(raw, min_bits, npix):
    clear, eoi = 1 << min_bits, (1 << min_bits) + 1
    width = min_bits + 1
    table = {i: bytes([i]) for i in range(clear)}
    nxt = eoi + 1
    out = bytearray()
    prev = None
    acc = nb = 0
    pos = 0
    while len(out) < npix:
        while nb < width and pos < len(raw):
            acc |= raw[pos] << nb
            nb += 8
            pos += 1
        if nb < width:
            break
        code = acc & ((1 << width) - 1)
        acc >>= width
        nb -= width
        if code == clear:
            table = {i: bytes([i]) for i in range(clear)}
            nxt = eoi + 1
            width = min_bits + 1
            prev = None
            continue
        if code == eoi:
            break
        if code in table:
            ent = table[code]
        elif code == nxt and prev is not None:
            ent = prev + prev[:1]
        else:
            raise ValueError("bad LZW code")
        out += ent
        if prev is not None and nxt < 4096:
            table[nxt] = prev + ent[:1]
            nxt += 1
            if nxt == (1 << width) and width < 12:
                width += 1
        prev = ent
    return bytes(out[:npix])


# ================================================================================================ outputs: tables, frames, svg, html, README results
def tup(f):
    return tuple(f) if f else None


def fin_text(feat):
    return FIN_TEXT[tup(feat.get("fin"))]


def surface_name(r):
    return "flap" if r["family"] == "T" else ("tail bend" if r["family"] == "B" else "setting")


def spiral_short(r):
    sp = r["modes"]["spiral"]
    return f"STABLE T1/2 {math.log(2) / -sp:.0f} S" if sp < 0 else f"DOUBLES IN {math.log(2) / sp:.0f} S"


def prepare(state):
    refs, res1, res2 = state["refs"], state["res1"], state["res2"]
    base = next(r for r in refs if r["name"] == "B45_as_flown")
    tw_base = base["tw30"]["b"]
    for tag, grp in (("ref", refs), ("1", res1), ("2", res2)):
        for r in grp:
            r["stage"] = tag
            r["delta_shown"] = r["delta"] + (ss.SW_BEND if r["name"] == "SwiftA_as_flown" else 0.0)           # Swift A as flown carries its 6 deg bend as section incidence, not as control deflection
            r["rule"] = judge(r, tw_base)
    return tw_base, refs + res1 + res2


def config_rows(allr, tw_base):
    rows = []
    for r in allr:
        d, m, fe = r["deriv"], r["modes"], r["feat"]
        a, rw = r["asym"], r["rawdon"]
        sh = r["shape"] or {}
        rows.append(dict(
            config=r["name"], stage=r["stage"], outline=r["outline"], family=r["family"], dihedral_outer40_deg=fe.get("dih", 0.0), fin=fin_text(fe), trim_CL_target=fe.get("cl", ""),
            trim_CL=round(r["cl_trim"], 4), static_margin_pct_mac=round(r["sm_pct"], 3), trim_alpha_deg=round(r["alpha"], 3), pitch_margin_to_8deg=round(pitch_margin(r), 3),
            trim_surface=surface_name(r), trim_surface_deg=round(r["delta_shown"], 3), trim_feasible=int(r["trim_feasible"]), trim_converged=int(r["converged"]), gamma_deg=round(r["gamma"], 3),
            gamma_source="fixed -6.61 (reference)" if r["fixed"] or r["family"] == "Z" else f"-atan((CD0 {CD0} + CD_AVL {r['CDi']:.4f}) / CL)", V_m_s=round(r["V"], 3), Re_MAC=round(r["Re"]),
            cg_mm_aft_root_LE=round(r["cg"], 2), xnp_mm=round(r["xnp"], 2), mac_mm=round(r["mac"], 2), mass_g=round(r["mass_g"], 3), ballast_g=round(r["ballast_g"], 3),
            aft_ballast_needed=int(r["aft_ballast"]), mass_tag=r["mass_tag"], wing_loading_N_m2=round(r["wing_loading_N_m2"], 3), bed_fits_250=int(r["bed_fits"]), bed_w_mm=round(r["bed_w_h"][0], 1), bed_h_mm=round(r["bed_w_h"][1], 1),
            Cnb=round(d["Cnb"], 5), Clb=round(d["Clb"], 5), Cnr=round(d["Cnr"], 5), Clr=round(d["Clr"], 5), Cnp=round(d["Cnp"], 5), Clp=round(d["Clp"], 5), CLa_per_rad=round(d["CLa"] * 1.0, 3),
            Cma_per_rad=round(d["Cma"], 4), Cmq=round(d["Cmq"], 4), yaw_floor_1_over_abs_Cnr=round(1.0 / abs(d["Cnr"]), 1) if d["Cnr"] else "",
            spiral_eig_per_s=round(m["spiral"], 5), spiral=r["spiral_text"], roll_eig_per_s=round(m["roll"], 3), osc_wn=round(m["dr_wn"], 3), osc_zeta=round(m["dr_zeta"], 4),
            osc_beta_over_phi=round(sh.get("beta_over_phi", float("nan")), 3) if sh else "", Jxz_kgm2=r["jxz"],
            twist_tol_R30_deg_b=round(r["tw30"]["b"], 4), twist_tol_ratio_vs_B45=round(r["tw30"]["b"] / tw_base, 3), twist_tol_cancellation_ratio=round(cancel_ratio(r), 3), twist_flag=FLAG_TEXT if flagged(r) else "", twist_tol_R30_deg_a=round(r["tw30"].get("a", float("nan")), 4),
            twist_tol_R30_deg_c=round(r["tw30"].get("c", float("nan")), 4) if "c" in r["tw30"] else "", rhat_per_deg_b=round(a["b"]["rhat"], 6), dCl_b=round(a["b"]["dCl"], 6), dCn_b=round(a["b"]["dCn"], 6),
            bank_deg_per_deg_b=round(a["b"]["bank"], 2), EDA_deg=round(rw["eda"], 3), lv_mm=round(rw["lv"], 2), Rawdon_SS=round(rw["ss"], 3),
            rule1_spiral=int(r["rule"]["r1"]), rule2_oscillation=int(r["rule"]["r2"]), rule3_twist=int(r["rule"]["r3"]), rule4_pitch=int(r["rule"]["r4"]), rule5_bed=int(r["rule"]["r5"]),
            parts_passed=r["rule"]["n_pass"], CANDIDATE=int(r["rule"]["candidate"]), seconds=round(r["seconds"], 1)))
        p = r["params"]
        if r["family"] == "T":
            rows[-1].update(AR=p["ar"], sweep_qc_deg=p["sweep"], taper=p["taper"])
        elif r["family"] == "B":
            rows[-1].update(ar_target=p["ar_target"], hand_sweep_shift_deg=round(p["hand_shift"], 3), wrist_eta=p["wrist_eta"], tail_area_over_S=round(p["tail_area_frac"], 4))
    return rows


def frame_extent(specs):
    xs = [p[0] for sp in specs for p in sp["full_outline"]]
    return min(xs), max(xs)


PLAN_CX, PLAN_TOP, FRONT_BASE = 215, 22, 392
FW, FH = 800, 420


def draw_plan(cv, spec, sc, xmin, r=None, col=FILL):
    P = lambda x, y: (PLAN_CX + y * sc, PLAN_TOP + (x - xmin) * sc)
    cv.fill_poly([P(x, y) for x, y in spec["full_outline"]], FILL_B if spec["family"] == "B" else col)
    if spec["family"] == "B":
        cv.fill_poly([P(x, y) for x, y in spec["wing_poly"]], col)
    cv.poly_outline([P(x, y) for x, y in spec["full_outline"]], INK, 1)
    if r is not None:
        gx, gy = P(r["cg"], 0)
        cv.line(gx - 6, gy, gx + 6, gy, ACCENT, 1)
        cv.line(gx, gy - 6, gx, gy + 6, ACCENT, 1)
        nx, ny = P(r["xnp"], 0)
        cv.line(nx - 4, ny - 4, nx + 4, ny + 4, BLUE, 1)
        cv.line(nx - 4, ny + 4, nx + 4, ny - 4, BLUE, 1)
    cv.text(PLAN_CX - 190, 2, "PLAN", MUTED, 1)


def front_geometry(spec, feat):
    """Front-view polylines in mm (y, z): wing line and fins."""
    dih = feat.get("dih", 0.0)
    half = spec["half"]
    yk = kink_y(spec)
    yt, zt = rot_outer(half, dih, yk)
    wing = [(-yt, zt), (-yk, 0.0), (yk, 0.0), (yt, zt)] if dih else [(-half, 0.0), (half, 0.0)]
    fins = []
    fin = tup(feat.get("fin"))
    if fin and fin[0] == "tip":
        c = math.radians(fin[1])
        for sgn in (1, -1):
            fins.append([(sgn * (yt + dz * math.sin(c)), zt + dz * math.cos(c)) for dz in (FIN_H / 2, -FIN_H / 2)])
    elif fin and fin[0] == "centre":
        fg = fin_geometry_B(spec, fin[1])
        fins.append([(0.0, 0.0), (0.0, fg["h"])])
    return wing, fins


def draw_front(cv, spec, feat, sc):
    wing, fins = front_geometry(spec, feat)
    Q = lambda y, z: (PLAN_CX + y * sc, FRONT_BASE - z * sc)
    cv.poly_outline([Q(*p) for p in wing], INK, 2, closed=False)
    for f in fins:
        cv.poly_outline([Q(*p) for p in f], GOOD, 2, closed=False)
    cv.text(PLAN_CX - 190, FRONT_BASE - 62, "FRONT", MUTED, 1)


def param_lines(r):
    p = r["params"]
    if r["family"] == "T":
        return [f"AR {p['ar']:g} SWEEP {p['sweep']:g} TAPER {p['taper']:g}"]
    if r["family"] == "B":
        return [f"AR {p['ar_target']:g} HAND {p['hand_shift']:+g} WRIST {p['wrist_eta']:.3f}", f"TAIL {p['tail_area_frac']:.3f} X S"]
    return ["SCAN OUTLINE, CURL 30% 2.2 DEG"]


def pf(ok):
    return GOOD if ok else BAD


def lines_stage1(r, tw_base, base_sw):
    ru = r["rule"]
    L = [("REFERENCE (NOT ELIGIBLE)" if r["fixed"] else "STAGE 1  FLAT", ACCENT), (r["spec_name"], INK)]
    L += [(t, MUTED) for t in param_lines(r)]
    L.append((f"CG {r['cg']:.1f}  NP {r['xnp']:.1f} MM", MUTED))
    L.append((f"TWIST R30 {r['tw30']['b']:.3f} DEG", pf(ru["r3"])))
    L.append((f"= {r['tw30']['b'] / tw_base:.1f}X B45  NEED 3", pf(ru["r3"])))
    L.append(("SPIRAL " + spiral_short(r), pf(ru["r1"])))
    L.append((f"ZETA {r['modes']['dr_zeta']:+.2f} CNB {r['deriv']['Cnb']:+.4f}", pf(ru["r2"])))
    L.append((f"SM {r['sm_pct']:.1f}% ALPHA {r['alpha']:.1f} CL {r['cl_trim']:.2f}", pf(ru["r4"])))
    L.append((f"{surface_name(r).upper()} {r['delta_shown']:.1f} DEG UP", MUTED))
    L.append((f"MASS {r['mass_g']:.1f} G WL {r['wing_loading_N_m2']:.1f} N/M2", MUTED))
    L.append((f"GAMMA {r['gamma']:.1f} V {r['V']:.1f} M/S", MUTED))
    L.append((f"BED FIT {'YES' if ru['r5'] else 'NO'} {r['bed_w_h'][0]:.0f}X{r['bed_w_h'][1]:.0f}", pf(ru["r5"])))
    L.append(("PARTS " + " ".join(f"{i + 1}{'P' if ru[f'r{i + 1}'] else 'F'}" for i in range(5)), INK))
    L.append(("CANDIDATE" if ru["candidate"] else f"{ru['n_pass']} OF 5 PARTS", GOOD if ru["candidate"] else ACCENT))
    if flagged(r):
        L += [("CANCELLATION: TWIST NUMBER", BAD), ("NOT CREDIBLE", BAD)]
    return L


def lines_stage2(grp, tw_base):
    r0 = grp[0]
    L = [("STAGE 2  FEATURES", ACCENT), (r0["outline"], INK), (f"DIHEDRAL {r0['feat']['dih']:g} DEG OUTER 40%", MUTED), (fin_text(r0["feat"]).upper(), MUTED)]
    for r in sorted(grp, key=lambda x: x["feat"]["cl"]):
        ru = r["rule"]
        L.append((f"CL {r['feat']['cl']:.2f}  TWIST {r['tw30']['b']:.3f} DEG", pf(ru["r3"])))
        L.append((f" X{r['tw30']['b'] / tw_base:.1f} B45  ALPHA {r['alpha']:.1f}", pf(ru["r4"])))
        L.append((f" SPIRAL {'STABLE' if r['modes']['spiral'] < 0 else 'UNSTABLE'} T {math.log(2) / abs(r['modes']['spiral']):.0f} S", pf(ru["r1"])))
        L.append((f" ZETA {r['modes']['dr_zeta']:+.2f} CNB {r['deriv']['Cnb']:+.4f}", pf(ru["r2"])))
        L.append((f" SS {r['rawdon']['ss']:.1f}  PARTS {' '.join(('P' if ru[f'r{i + 1}'] else 'F') for i in range(5))}", INK))
        L.append((" CANDIDATE" if ru["candidate"] else f" {ru['n_pass']} OF 5 PARTS", GOOD if ru["candidate"] else ACCENT))
        if flagged(r):
            L.append((" CANCELLATION: TWIST NOT CREDIBLE", BAD))
    return L


def render_frames(state, specmap, tw_base):
    """List of (name, bytes) frames: references, 54 Stage-1 outlines, Stage-2 (outline x dihedral x fin) groups."""
    refs, res1, res2 = state["refs"], state["res1"], state["res2"]
    allspecs = [specmap[r["spec_name"]] for r in refs + res1]
    xmin, xmax = frame_extent(allspecs)
    sc = min(0.95, 300.0 / (xmax - xmin))
    frames = []

    def put(spec, feat, lines, r, col, flag=False):
        cv = Canvas(FW, FH)
        if flag:                                              # red frame border: at least one twist number on this frame is a cancellation artefact
            for k in range(3):
                cv.poly_outline([(k, k), (FW - 1 - k, k), (FW - 1 - k, FH - 1 - k), (k, FH - 1 - k)], BAD, 1)
        draw_plan(cv, spec, sc, xmin, r, col)
        draw_front(cv, spec, feat, sc)
        for i, (t, c) in enumerate(lines):
            cv.text(440, 8 + i * 20, t, c, 2)
        cv.text(440, FH - 12, "FRAMES: SPAN 400 MM, COMMON SCALE", GRID, 1)
        return bytes(cv.buf)
    for r in refs + res1:
        sp = specmap[r["spec_name"]]
        col = FILL_G if r["rule"]["candidate"] else FILL
        frames.append((r["name"], put(sp, r["feat"], lines_stage1(r, tw_base, None), r, col, flagged(r))))
    groups = {}
    for r in res2:
        groups.setdefault((r["outline"], r["feat"]["dih"], tup(r["feat"]["fin"])), []).append(r)
    for (o, dih, fin), grp in groups.items():
        sp = specmap[o]
        col = FILL_G if any(x["rule"]["candidate"] for x in grp) else FILL
        frames.append((f"{o}__d{dih:g}__{FIN_TAG[fin]}", put(sp, grp[0]["feat"], lines_stage2(grp, tw_base), grp[0], col, any(flagged(x) for x in grp))))
    return frames


# ---------------------------------------------------------------------------------------------- svg contact sheet, pareto, html
SVG_COL = dict(bg="#14181d", panel="#1c2229", line="#333c46", ink="#dde3ea", muted="#9aa7b4", accent="#e89a55", good="#6cc08b", bad="#ff8a80", blue="#7fb4ff", fill="#27313c")


def contact_sheet(state, specmap, tw_base, per_row=8):
    items = state["refs"] + state["res1"]
    xmin, xmax = frame_extent([specmap[r["spec_name"]] for r in items])
    cw, ch = 172, 168
    sc = 0.36
    rows = math.ceil(len(items) / per_row)
    W, H = per_row * cw + 20, rows * ch + 56
    C = SVG_COL
    s = [f"<svg xmlns='http://www.w3.org/2000/svg' width='{W}' height='{H}' viewBox='0 0 {W} {H}' font-family='Segoe UI,Arial,sans-serif'>", f"<rect width='{W}' height='{H}' fill='{C['bg']}'/>",
         f"<text x='10' y='20' fill='{C['ink']}' font-size='14'>Shape sweep v1, Stage 1 outlines (400 mm span, common scale) and the 3 references. Border: green = candidate, orange = 4 of 5 parts, grey = fewer, red = cancellation (twist number not credible, ratio below 0.25, a post-hoc cut). Twist = tolerance for R = 30 m (B45 as flown = {tw_base:.3f} deg).</text>",
         f"<text x='10' y='38' fill='{C['muted']}' font-size='11'>MODEL (AVL, inviscid). Stage-1 outlines are flat; T carries tip fins, B none. Not a flight result.</text>"]
    for i, r in enumerate(items):
        sp = specmap[r["spec_name"]]
        gx, gy = 10 + (i % per_row) * cw, 50 + (i // per_row) * ch
        ru = r["rule"]
        bc = C["good"] if ru["candidate"] else (C["accent"] if ru["n_pass"] >= 4 else C["line"])
        if flagged(r):
            bc = C["bad"]
        s.append(f"<rect x='{gx}' y='{gy}' width='{cw - 6}' height='{ch - 6}' fill='{C['panel']}' stroke='{bc}' stroke-width='{2 if bc != C['line'] else 1}'/>")
        cx, top = gx + (cw - 6) / 2, gy + 8
        pts = " ".join(f"{cx + y * sc:.1f},{top + (x - xmin) * sc:.1f}" for x, y in sp["full_outline"])
        s.append(f"<polygon points='{pts}' fill='{C['fill']}' stroke='{C['ink']}' stroke-width='0.8'/>")
        s.append(f"<text x='{gx + 5}' y='{gy + ch - 36}' fill='{C['ink']}' font-size='9.5'>{r['spec_name']}</text>")
        s.append(f"<text x='{gx + 5}' y='{gy + ch - 24}' fill='{C['muted']}' font-size='9'>twist {r['tw30']['b']:.3f} deg ({r['tw30']['b'] / tw_base:.1f}x)  alpha {r['alpha']:.1f}</text>")
        if flagged(r):
            s.append(f"<text x='{gx + 5}' y='{gy + ch - 50}' fill='{C['bad']}' font-size='8' textLength='158' lengthAdjust='spacingAndGlyphs'>{FLAG_TEXT}</text>")
        s.append(f"<text x='{gx + 5}' y='{gy + ch - 12}' fill='{C['muted']}' font-size='9'>zeta {r['modes']['dr_zeta']:+.2f}  spiral {'S' if r['modes']['spiral'] < 0 else 'U'}  parts {ru['n_pass']}/5</text>")
    s.append("</svg>")
    return "\n".join(s)


def pareto_svg(allr, top3_names, front_names, tw_base, W=760, H=470):
    C = SVG_COL
    pts = [r for r in allr if r["trim_feasible"] or r["fixed"]]
    xs = [r["tw30"]["b"] for r in pts]
    lo, hi = max(1e-3, min(xs) * 0.8), max(xs) * 1.25
    ys = [pitch_margin(r) for r in pts]
    ylo, yhi = min(ys + [0.0]) - 0.5, max(ys) + 0.5
    L, R, T, B = 60, W - 20, 30, H - 50
    X = lambda v: L + (math.log10(v) - math.log10(lo)) / (math.log10(hi) - math.log10(lo)) * (R - L)
    Y = lambda v: B - (v - ylo) / (yhi - ylo) * (B - T)
    s = [f"<svg xmlns='http://www.w3.org/2000/svg' width='{W}' height='{H}' viewBox='0 0 {W} {H}' font-family='Segoe UI,Arial,sans-serif'><rect width='{W}' height='{H}' fill='{C['panel']}'/>"]
    e = math.floor(math.log10(lo))
    while 10 ** e < hi:
        for m in (1, 2, 5):
            v = m * 10 ** e
            if lo <= v <= hi:
                s.append(f"<line x1='{X(v):.1f}' y1='{T}' x2='{X(v):.1f}' y2='{B}' stroke='{C['line']}' stroke-width='0.6'/><text x='{X(v):.1f}' y='{B + 14}' fill='{C['muted']}' font-size='10' text-anchor='middle'>{v:g}</text>")
        e += 1
    for v in range(int(math.ceil(ylo)), int(yhi) + 1):
        s.append(f"<line x1='{L}' y1='{Y(v):.1f}' x2='{R}' y2='{Y(v):.1f}' stroke='{C['line']}' stroke-width='0.6'/><text x='{L - 6}' y='{Y(v) + 3:.1f}' fill='{C['muted']}' font-size='10' text-anchor='end'>{v}</text>")
    s.append(f"<line x1='{X(TW_FACTOR * tw_base):.1f}' y1='{T}' x2='{X(TW_FACTOR * tw_base):.1f}' y2='{B}' stroke='{C['accent']}' stroke-dasharray='5 4'/><text x='{X(TW_FACTOR * tw_base) + 4:.1f}' y='{T + 12}' fill='{C['accent']}' font-size='10'>rule 3: 3 x B45</text>")
    s.append(f"<line x1='{L}' y1='{Y(ALPHA_BREAK - ALPHA_MAX):.1f}' x2='{R}' y2='{Y(ALPHA_BREAK - ALPHA_MAX):.1f}' stroke='{C['accent']}' stroke-dasharray='5 4'/><text x='{R - 4}' y='{Y(ALPHA_BREAK - ALPHA_MAX) - 4:.1f}' fill='{C['accent']}' font-size='10' text-anchor='end'>rule 4: alpha 5 deg</text>")
    fr = sorted([r for r in pts if r["name"] in front_names], key=lambda r: r["tw30"]["b"])
    if len(fr) > 1:
        pl = " ".join("%.1f,%.1f" % (X(q["tw30"]["b"]), Y(pitch_margin(q))) for q in fr)
        s.append(f"<polyline points='{pl}' fill='none' stroke='{C['blue']}' stroke-width='1' stroke-dasharray='2 3'/>")
    for r in pts:
        x, y = X(r["tw30"]["b"]), Y(pitch_margin(r))
        col = C["blue"] if r["family"] == "T" else (C["accent"] if r["family"] == "B" else C["ink"])
        if r["fixed"]:
            s.append(f"<polygon points='{x:.1f},{y - 6:.1f} {x + 6:.1f},{y:.1f} {x:.1f},{y + 6:.1f} {x - 6:.1f},{y:.1f}' fill='{C['ink']}' stroke='{C['bg']}'><title>{r['name']}</title></polygon>")
            s.append(f"<text x='{x + 8:.1f}' y='{y + 3:.1f}' fill='{C['ink']}' font-size='9'>{r['name'].replace('_as_flown', '').replace('_400', '')}</text>")
            continue
        op = 0.9 if r["stage"] == "2" else 0.55
        s.append(f"<circle cx='{x:.1f}' cy='{y:.1f}' r='{3.2 if r['stage'] == '1' else 2.6}' fill='{col}' fill-opacity='{op}'><title>{r['name']} twist {r['tw30']['b']:.3f} alpha {r['alpha']:.2f} parts {r['rule']['n_pass']}</title></circle>")
        if r["rule"]["candidate"]:
            s.append(f"<circle cx='{x:.1f}' cy='{y:.1f}' r='7' fill='none' stroke='{C['good']}' stroke-width='2'/>")
    for k, nm in enumerate(top3_names):
        r = next((q for q in pts if q["name"] == nm), None)
        if r:
            s.append(f"<text x='{X(r['tw30']['b']) + 9:.1f}' y='{Y(pitch_margin(r)) - 8:.1f}' fill='{C['good']}' font-size='11' font-weight='bold'>top {k + 1}</text>")
    s.append(f"<text x='{(L + R) / 2}' y='{H - 8}' fill='{C['ink']}' font-size='12' text-anchor='middle'>twist tolerance for R = 30 m (wing-half incidence difference, deg, log axis, larger is better)</text>")
    s.append(f"<text x='14' y='{(T + B) / 2}' fill='{C['ink']}' font-size='12' transform='rotate(-90 14 {(T + B) / 2})' text-anchor='middle'>pitch margin = 8 deg - trim alpha (deg)</text>")
    lx = L + 8
    for txt, col in (("T family", C["blue"]), ("B family", C["accent"])):
        s.append(f"<circle cx='{lx}' cy='{T + 10}' r='4' fill='{col}'/><text x='{lx + 8}' y='{T + 14}' fill='{C['ink']}' font-size='11'>{txt}</text>")
        lx += 80
    s.append(f"<polygon points='{lx},{T + 4} {lx + 6},{T + 10} {lx},{T + 16} {lx - 6},{T + 10}' fill='{C['ink']}'/><text x='{lx + 10}' y='{T + 14}' fill='{C['ink']}' font-size='11'>reference</text>")
    s.append("</svg>")
    return "\n".join(s)


HTML_CSS = ss.DARK_CSS + "\nth{cursor:pointer}.cand{background:#1d3b2a}.near{background:#3a3220}.cancel{background:#3b1d1d}img,svg{max-width:100%;height:auto}"
SORT_JS = """<script>document.querySelectorAll('table.sortable').forEach(t=>{t.querySelectorAll('th').forEach((th,i)=>{th.addEventListener('click',()=>{const b=t.tBodies[0],rows=[...b.rows],d=th.dataset.d==='1'?-1:1;th.dataset.d=d;
rows.sort((a,c)=>{const x=a.cells[i].dataset.v??a.cells[i].textContent,y=c.cells[i].dataset.v??c.cells[i].textContent,nx=parseFloat(x),ny=parseFloat(y);return(isNaN(nx)||isNaN(ny)?x.localeCompare(y):nx-ny)*d});rows.forEach(r=>b.appendChild(r))})})});</script>"""


def html_index(rows, allr, top3, pareto, tw_base, wall, n_frames):
    cols = ["config", "stage", "family", "dihedral_outer40_deg", "fin", "trim_CL", "static_margin_pct_mac", "trim_alpha_deg", "pitch_margin_to_8deg", "trim_surface_deg", "gamma_deg", "mass_g", "wing_loading_N_m2",
            "Cnb", "Clb", "Cnr", "spiral_eig_per_s", "osc_zeta", "twist_tol_R30_deg_b", "twist_tol_ratio_vs_B45", "twist_tol_cancellation_ratio", "twist_flag", "Rawdon_SS", "parts_passed", "CANDIDATE", "bed_fits_250"]
    P = ["<!doctype html><html lang='en'><head><meta charset='utf-8'><title>Shape sweep v1</title><style>" + HTML_CSS + "</style></head><body><main>",
         "<h1>Shape sweep v1: planform families and stability features</h1>",
         "<p class='note'>MODEL = inviscid AVL 3.52 (flat plate, thickness ignored). Rank, do not predict: AVL's Clb is unreliable for thin low-aspect-ratio plates at Re 10-30k and CLa is uncertain by about +-30 %. "
         "Nothing here is a flight result. The pre-stated question, method and rule are in <a href='README.md'>README.md</a> (hash-checked). "
         f"Twist tolerance = wing-half incidence difference that gives a circle of R = 30 m; B45 as flown = {tw_base:.3f} deg, rule 3 needs 3 x that. Model time {wall / 60:.0f} min.</p>",
         "<h2>Animated sweep</h2>", f"<p><img src='sweep.gif' alt='sweep.gif' width='800'><br><span class='note'>{n_frames} frames, about 0.6 s each: references, the 54 Stage-1 outlines, then Stage-2 dihedral / fin variants (front view under the plan view). "
         "Also: <a href='contact_sheet.svg'>contact_sheet.svg</a>, <a href='configs.csv'>configs.csv</a>, <a href='sheet_builder_top3/'>sheet_builder_top3/</a>.</span></p>",
         f"<p class='note'>Rows shaded red carry the marker '{FLAG_TEXT}' (cancellation ratio below {CANCEL_CUT}, a post-hoc cut): their twist tolerance is an artefact of the r_hat numerator nearly cancelling. The pre-stated top 3 all carry it.</p>", "<h2>Top 3</h2>"]
    if top3:
        P.append("<div class='scroll'><table><tr><th>rank</th><th>label</th><th>config</th><th>twist deg (R30)</th><th>x B45</th><th>cancel ratio</th><th>alpha</th><th>zeta</th><th>Cnb</th><th>spiral</th><th>parts</th></tr>")
        for k, (lab, r) in enumerate(top3):
            P.append(f"<tr class='{'cand' if lab == 'CANDIDATE' else 'near'}'><td>{k + 1}</td><td>{lab}</td><td>{r['name']}</td><td class='n'>{r['tw30']['b']:.3f}</td><td class='n'>{r['tw30']['b'] / tw_base:.1f}</td>"
                     f"<td class='n'>{cancel_ratio(r):.3f} {'(' + FLAG_TEXT + ')' if flagged(r) else ''}</td><td class='n'>{r['alpha']:.1f}</td><td class='n'>{r['modes']['dr_zeta']:+.3f}</td><td class='n'>{r['deriv']['Cnb']:+.4f}</td><td>{r['spiral_text']}</td><td>{r['rule']['n_pass']}/5</td></tr>")
        P.append("</table></div>")
    P += ["<h2>Pareto: twist tolerance against pitch margin</h2>", pareto,
          "<h2>All configurations (click a header to sort)</h2><div class='scroll'><table class='sortable'><tr>" + "".join(f"<th>{c}</th>" for c in cols) + "</tr>"]
    for r, row in zip(allr, rows):
        cls = "cand" if row["CANDIDATE"] else ("near" if row["parts_passed"] >= 4 else "")
        if row["twist_flag"]:
            cls = "cancel"
        P.append(f"<tr class='{cls}'>" + "".join(f"<td class='n' data-v='{row[c]}'>{row[c]}</td>" for c in cols) + "</tr>")
    P.append("</table></div>" + SORT_JS + "</main></body></html>")
    return "\n".join(P)


# ================================================================================================ README results, sheet_builder configs, driver
def med(v):
    v = sorted(v)
    n = len(v)
    return v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2])


def fmt_stat(vals, kind="diff", nd=3):
    if not vals:
        return "n/a"
    lo, hi, m = min(vals), max(vals), med(vals)
    if kind == "ratio":
        return f"x{m:.2f} (x{lo:.2f}..x{hi:.2f}, n={len(vals)})"
    return f"{m:+.{nd}f} ({lo:+.{nd}f}..{hi:+.{nd}f}, n={len(vals)})"


def feature_effects(res2):
    """Matched-pair effects in Stage 2 (median, range, n): dihedral, fin, trim CL."""
    key = lambda r: (r["outline"], r["feat"]["dih"], tup(r["feat"]["fin"]), r["feat"]["cl"])
    look = {key(r): r for r in res2}
    out = []

    def pair_stats(pairs):
        d = lambda f: [f(b) - f(a) for a, b in pairs]
        return dict(tw=[b["tw30"]["b"] / a["tw30"]["b"] for a, b in pairs], zeta=d(lambda r: r["modes"]["dr_zeta"]), spiral=d(lambda r: r["modes"]["spiral"]), cnb=d(lambda r: r["deriv"]["Cnb"]),
                    clb=d(lambda r: r["deriv"]["Clb"]), alpha=d(lambda r: r["alpha"]), n=len(pairs))
    for fam in ("T", "B"):
        sub = [r for r in res2 if r["family"] == fam]
        if not sub:
            continue
        outs = sorted(set(r["outline"] for r in sub))
        fins = S2_FIN[fam]
        for d1 in (10.0, 20.0):                                                  # dihedral
            pairs = [(look[(o, 0.0, tup(f), c)], look[(o, d1, tup(f), c)]) for o in outs for f in fins for c in CL_STAGE2 if (o, 0.0, tup(f), c) in look and (o, d1, tup(f), c) in look]
            out.append((fam, f"tip dihedral 0 -> {d1:g} deg", pair_stats(pairs)))
        steps = [(fins[0], fins[1]), (fins[1], fins[2]), (fins[0], fins[2])]
        for fa, fb in steps:                                                     # fin
            pairs = [(look[(o, d_, tup(fa), c)], look[(o, d_, tup(fb), c)]) for o in outs for d_ in S2_DIH for c in CL_STAGE2 if (o, d_, tup(fa), c) in look and (o, d_, tup(fb), c) in look]
            out.append((fam, f"fin: {FIN_TEXT[tup(fa)]} -> {FIN_TEXT[tup(fb)]}", pair_stats(pairs)))
        pairs = [(look[(o, d_, tup(f), CL_STAGE2[0])], look[(o, d_, tup(f), CL_STAGE2[1])]) for o in outs for d_ in S2_DIH for f in fins if (o, d_, tup(f), CL_STAGE2[0]) in look and (o, d_, tup(f), CL_STAGE2[1]) in look]
        out.append((fam, "trim CL 0.25 -> 0.35 (SM held)", pair_stats(pairs)))
    return out


def rule_table(allr):
    rows = []
    for lab, f in (("Stage 1 T", lambda r: r["stage"] == "1" and r["family"] == "T"), ("Stage 1 B", lambda r: r["stage"] == "1" and r["family"] == "B"),
                   ("Stage 2 T", lambda r: r["stage"] == "2" and r["family"] == "T"), ("Stage 2 B", lambda r: r["stage"] == "2" and r["family"] == "B"), ("references", lambda r: r["stage"] == "ref")):
        g = [r for r in allr if f(r)]
        if not g:
            continue
        c = lambda k: sum(1 for r in g if r["rule"][k])
        rows.append([lab, len(g), c("r1"), c("r2"), c("r3"), c("r4"), c("r5"), sum(1 for r in g if r["rule"]["n_pass"] >= 4), sum(1 for r in g if r["rule"]["candidate"])])
    return rows


def why_text(r, tw_base):
    ru, m, d = r["rule"], r["modes"], r["deriv"]
    bits = []
    bits.append(f"spiral {'passes' if ru['r1'] else 'FAILS'} ({r['spiral_text']})")
    bits.append(f"oscillation {'passes' if ru['r2'] else 'FAILS'} (zeta {m['dr_zeta']:+.3f}, Cnb {d['Cnb']:+.4f})")
    bits.append(f"twist tolerance {'passes' if ru['r3'] else 'FAILS'} ({r['tw30']['b']:.3f} deg = {r['tw30']['b'] / tw_base:.1f} x B45, needs 3)")
    bits.append(f"pitch {'passes' if ru['r4'] else 'FAILS'} (SM {r['sm_pct']:.1f} %, alpha {r['alpha']:.1f} deg, trim {'feasible' if r['trim_feasible'] else 'INFEASIBLE'})")
    bits.append(f"bed {'fits' if ru['r5'] else 'DOES NOT FIT'}")
    return "; ".join(bits)


def spec_by_name(specs):
    return {sp["name"]: sp for sp in specs}


NLC = chr(10)


def write_sheet_builder_top3(out: Path, top3, specmap, log):
    import subprocess
    d = out / "sheet_builder_top3"
    (d / "outlines").mkdir(parents=True, exist_ok=True)
    notes = []
    for k, (lab, r) in enumerate(top3):
        sp = specmap[r["spec_name"]]
        (d / "outlines" / f"{sp['name']}.svg").write_text(svg_half(sp), encoding="utf-8")
        fe = r["feat"]
        fin = tup(fe.get("fin"))
        note_dih = ""
        if fe.get("dih", 0.0) or (fin and fin[0] == "tip" and fin[1]):
            note_dih = f"The sheet_builder kit prints a uniform half dihedral from the rail, not the sweep's outer-40 pct tip dihedral of {fe.get('dih', 0.0):g} deg, and no canted fins; dihedral is set to 0 here, bend the outer panels by hand to the gauge angle. "
        note_flap = f"The reflex flap of the sweep (0.15..0.40 semi-span, 15 pct chord, {r['delta']:.1f} deg up) is not a sheet_builder feature: score and bend the trailing edge by hand. " if r["family"] == "T" else ""
        tag = "posthoc" if lab.startswith("POST-HOC") else f"top{k + 1}"
        cfg = dict(_header="NOT a build candidate (near-miss by cancellation / post-hoc trade); flat kit, dihedral not printed. MODEL study, not a flight result.", _comment=f"{lab} #{k + 1} of the shape sweep v1: {r['name']}. MODEL study, not a flight result. Outline = {sp['name']} at 400 mm span. CG {r['cg']:.1f} mm aft of the root leading edge (SM 12 pct MAC by AVL). "
                            f"The sweep trim used a {surface_name(r)} of {r['delta']:.1f} deg up at trim CL {r['cl_trim']:.2f}. " + note_dih + note_flap,
                   name=f"{tag}_{sp['name']}", span_mm=SPAN, source=dict(type="svg", path=f"outlines/{sp['name']}.svg", kind="half", flip_y=False), dihedral_deg=0.0, root_gap_mm=0.4,
                   sheet=dict(layer_height_mm=0.1, n_layers=2), tail=dict(bend_deg=round(r["delta"], 2) if r["family"] == "B" else 0.0),
                   mass=dict(pla_density_g_cm3=1.24, target_cg_mm=round(r["cg"], 2), cg_reference="root_le", glue_root_g=0.3), fin=dict(type="none"))
        if r["family"] == "B":
            cfg["source"]["hinge_x_mm"] = round(sp["x_hinge"], 3)
        else:                                                                    # the project's B45 practice: nose extension and a forward ballast station (-37.5 mm), far less lead than the default station behind the plate front
            cfg["rail"] = dict(nose_extension_mm=12.0, ballast_station_x_mm=-37.5, window_height_mm=6.0, ballast_block_thickness_mm=4.0, plate_thickness_mm=1.3, print_orientation="side")
            cfg["_comment"] += "Rail: B45-style nose extension 12 mm and ballast station -37.5 mm (sheet_builder_configs/b45_400.json); the sweep's mass model used the kit's default station behind the plate front, so the sweep's mass and lead figures for this family are higher than this kit's. "
        if fin and fin[0] == "tip":
            cfg["fin"] = dict(type="tip", height_mm=FIN_H, chord_mm=round(sp["G"]["ct"], 2), taper=1.0, thickness_mm=0.6, tab_on="right", wing_tab_fracs=[0.15, 0.45])
        elif fin and fin[0] == "centre":
            fg = fin_geometry_B(sp, fin[1])
            cfg["fin"] = dict(type="centre", height_mm=round(fg["h"], 2), chord_mm=round(fg["c"], 2), taper=1.0, thickness_mm=0.6)
        cp = d / f"{tag}.json"
        cp.write_text(json.dumps(cfg, indent=1), encoding="utf-8")
        kit_out = d / "kits" / cfg["name"]
        try:
            rr = subprocess.run([sys.executable, "-B", str(HERE / "sheet_builder.py"), str(cp), "--out", str(kit_out), "--force"], cwd=str(d), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=900)
            if rr.returncode != 0:                                              # ladder: taller / thicker ballast window, then no CG target; each step is written into the JSON comment
                (d / f"{tag}_kit_run_first_attempt.log").write_text(rr.stdout, encoding="utf-8")
                rail = dict(cfg.get("rail", {}), window_height_mm=10.0, ballast_block_thickness_mm=6.0)
                cfg["rail"] = rail
                cfg["_comment"] += "FIRST ATTEMPT FAILED (ballast window did not fit, see the first_attempt log): rail ballast window made 10 mm tall and 6 mm thick. "
                cp.write_text(json.dumps(cfg, indent=1), encoding="utf-8")
                rr = subprocess.run([sys.executable, "-B", str(HERE / "sheet_builder.py"), str(cp), "--out", str(kit_out), "--force"], cwd=str(d), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=900)
            if rr.returncode != 0:
                (d / f"{tag}_kit_run_second_attempt.log").write_text(rr.stdout, encoding="utf-8")
                cfg["mass"]["target_cg_mm"] = None
                cfg["_comment"] += "SECOND ATTEMPT FAILED too: this kit was built WITHOUT a CG target, so no ballast is sized: add lead to reach the CG above. "
                cp.write_text(json.dumps(cfg, indent=1), encoding="utf-8")
                rr = subprocess.run([sys.executable, "-B", str(HERE / "sheet_builder.py"), str(cp), "--out", str(kit_out), "--force"], cwd=str(d), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=900)
            (d / f"{tag}_kit_run.log").write_text(rr.stdout, encoding="utf-8")
            notes.append((cfg["name"], rr.returncode, rr.stdout.strip().splitlines()[-2:] if rr.stdout.strip() else []))
        except Exception as e:
            notes.append((cfg["name"], -1, [repr(e)]))
        log(f"sheet_builder {cfg['name']}: {notes[-1][1]}")
    (d / "kits").mkdir(parents=True, exist_ok=True)
    (d / "kits" / "README.md").write_text("NOT a build candidate (near-miss by cancellation / post-hoc trade); flat kit, dihedral not printed." + NLC + NLC + "These kits are sheet_builder output for the three pre-stated near-misses of shape_sweep_v1 and one post-hoc pick. The sweep configuration includes outer-panel tip dihedral, canted or centre fins and (family T) a reflex flap that sheet_builder cannot print; the kits are flat. MODEL study, not a flight result; see ../../README.md." + NLC, encoding="utf-8")
    return notes


def readme_results(state, allr, rows, tw_base, top3, front, sbnotes, tests_text=None, posthoc_text=""):
    refs, res1, res2 = state["refs"], state["res1"], state["res2"]
    sw_ref = next(r for r in refs if r["name"] == "SwiftA_as_flown")
    b45r = next(r for r in refs if r["name"] == "B45_as_flown")
    zr = next(r for r in refs if r["name"] == "Zanonia_400")
    A = []
    P = A.append
    ncand = sum(1 for r in allr if r["rule"]["candidate"])
    P(f"Filled in {time.strftime('%Y-%m-%d %H:%M')} (local) by `rigid_glider/shape_sweep_v1.py`; AVL model time {state['wall'] / 60:.0f} min (sum of per-configuration times; wall clock of the compute run {state.get('wall_clock', state['wall']) / 60:.0f} min). "
      "MODEL = inviscid AVL 3.52; CALCULATED = sheet_builder STL volumes; ASSUMED = lead ballast, point-mass inertias, CD0, flap / tail-bend effectiveness, asymmetry sizes.\n")
    P(summary_text(state, allr, tw_base, b45r, sw_ref, top3))
    P("### Counts\n")
    nconv = sum(1 for r in allr if r["converged"])
    ninf = sum(1 for r in allr if not r["trim_feasible"])
    P(f"{len(allr)} AVL configurations: {len(refs)} references + {len(res1)} Stage-1 outlines (27 T + 27 B) + {len(res2)} Stage-2 variants (10 outlines x 3 dihedral x 3 fin options x 2 trim CL); cap 300. "
      f"Trim converged in {nconv}, trim infeasible (delta above 25 deg or not converged) in {ninf}, failed runs {len(state['fails'])}{(' ' + str(state['fails'])) if state['fails'] else ''}. "
      f"Stage-2 outlines chosen by the pre-stated rule: {', '.join(state['selected'])}.{(' Notes: ' + '; '.join(state['notes'])) if state['notes'] else ''}\n")
    P(f"**Candidates passing all five parts: {ncand} of {len([r for r in allr if not r['fixed']])} eligible configurations.**\n")
    P("### Baselines (same pipeline, same frame)\n")
    head = ["configuration", "twist tolerance R=30 m (deg)", "x B45", "spiral", "osc zeta", "Cnb", "Clb", "SM % MAC", "trim alpha", "trim CL", "gamma", "mass g", "parts passed"]
    rr = []
    for r in (b45r, sw_ref, zr):
        rr.append([r["name"], f"{r['tw30']['b']:.4f}", f"{r['tw30']['b'] / tw_base:.2f}", r["spiral_text"], f"{r['modes']['dr_zeta']:+.3f}", f"{r['deriv']['Cnb']:+.4f}", f"{r['deriv']['Clb']:+.4f}", f"{r['sm_pct']:.2f}", f"{r['alpha']:.2f}", f"{r['cl_trim']:.3f}", f"{r['gamma']:.2f}", f"{r['mass_g']:.1f}", f"{r['rule']['n_pass']}/5"])
    P(md_t(head, rr))
    P(f"Rule 3 threshold = 3 x {tw_base:.4f} = {TW_FACTOR * tw_base:.4f} deg of wing-half incidence difference for R = 30 m. (`straight_smooth_v1` gives 0.033 deg at R = 31.7 m for B45, i.e. 0.035 at 30 m: the reproduction is within 2 %, see Tests.)\n")
    P("### Parts passed by group\n")
    P(md_t(["group", "n", "1 spiral", "2 oscillation", "3 twist", "4 pitch", "5 bed", ">= 4 parts", "candidates"], rule_table(allr)))
    # ---- top 3
    P("### Top 3\n")
    if not top3:
        P("No eligible configuration.\n")
    head = ["rank", "label", "configuration", "twist deg (R30)", "x B45", "x Swift A", "cancel ratio", "spiral", "zeta", "Cnb", "Clb", "SM %", "alpha deg", "trim CL", "surface deg up", "gamma", "mass g", "wing loading N/m2", "bed", "parts"]
    rr = []
    for k, (lab, r) in enumerate(top3):
        rr.append([k + 1, lab, r["name"], f"{r['tw30']['b']:.3f}", f"{r['tw30']['b'] / tw_base:.1f}", f"{r['tw30']['b'] / sw_ref['tw30']['b']:.1f}", f"{cancel_ratio(r):.3f}", r["spiral_text"], f"{r['modes']['dr_zeta']:+.3f}", f"{r['deriv']['Cnb']:+.4f}", f"{r['deriv']['Clb']:+.4f}",
                   f"{r['sm_pct']:.1f}", f"{r['alpha']:.2f}", f"{r['cl_trim']:.3f}", f"{r['delta']:.1f}", f"{r['gamma']:.2f}", f"{r['mass_g']:.1f}", f"{r['wing_loading_N_m2']:.1f}", "yes" if r["bed_fits"] else "NO", f"{r['rule']['n_pass']}/5"])
    P(md_t(head, rr))
    for k, (lab, r) in enumerate(top3):
        P(f"- **#{k + 1} {r['name']} ({lab})**: {why_text(r, tw_base)}. Versus B45 as flown (twist {tw_base:.3f} deg, spiral {b45r['spiral_text']}, zeta {b45r['modes']['dr_zeta']:+.3f}) and Swift A as flown (twist {sw_ref['tw30']['b']:.3f} deg, {sw_ref['spiral_text']}, zeta {sw_ref['modes']['dr_zeta']:+.3f}). "
          f"Rawdon SS {r['rawdon']['ss']:.2f}; trim {surface_name(r)} {r['delta']:.1f} deg; mass {r['mass_g']:.1f} g ({r['ballast_g']:.1f} g of it lead ballast).")
    P("")
    if sbnotes:
        P("sheet_builder configs and kit runs: `sheet_builder_top3/top1.json` .. `top3.json`, kits in `sheet_builder_top3/kits/` (new subfolder; nothing existing was overwritten). Results: " + "; ".join(f"{n}: exit {c}{kit_mass_text(lines, r_)}" for (n, c, lines), (_, r_) in zip(sbnotes, top3)) + ". The kit cannot print the outer-panel dihedral, the canted fins or the T-family reflex flap: see the `_comment` in each JSON.\n")
    # ---- near misses
    nm = sorted([r for r in allr if not r["fixed"] and r["trim_feasible"]], key=lambda r: (-r["rule"]["n_pass"], -r["tw30"]["b"]))[:12]
    P("### Closest configurations (parts passed, then twist tolerance)\n")
    head = ["configuration", "parts", "failed parts", "twist deg", "x B45", "spiral", "zeta", "Cnb", "alpha", "SM %"]
    rr = [[r["name"], f"{r['rule']['n_pass']}/5", ",".join(str(i) for i in range(1, 6) if not r["rule"][f"r{i}"]) or "-", f"{r['tw30']['b']:.3f}", f"{r['tw30']['b'] / tw_base:.1f}", r["spiral_text"], f"{r['modes']['dr_zeta']:+.3f}",
           f"{r['deriv']['Cnb']:+.4f}", f"{r['alpha']:.2f}", f"{r['sm_pct']:.1f}"] for r in nm]
    P(md_t(head, rr))
    if posthoc_text:
        P(posthoc_text)
    # ---- stage 1
    P("### Stage 1: what the outlines do (flat, SM 12 %, CL 0.30)\n")
    for fam, nmx in (("T", "Family T (trapezoid, tip fins)"), ("B", "Family B (bird lineage)")):
        g_all = [r for r in res1 if r["family"] == fam]
        g = [r for r in g_all if r["trim_feasible"]]
        if not g:
            continue
        inf = [r["spec_name"] for r in g_all if not r["trim_feasible"]]
        best_tw = sorted(g, key=lambda r: -r["tw30"]["b"])[:3]
        best_a = sorted(g, key=lambda r: r["alpha"])[:3]
        P(f"- {nmx} ({len(g)} of {len(g_all)} with a feasible trim; infeasible: {', '.join(inf) or 'none'}; ranges below are over the feasible ones): trim alpha {min(r['alpha'] for r in g):.2f}-{max(r['alpha'] for r in g):.2f} deg ({sum(1 for r in g if r['alpha'] <= ALPHA_MAX)} of {len(g)} at or below 5 deg), reflex / bend {min(r['delta'] for r in g):.1f}-{max(r['delta'] for r in g):.1f} deg, "
          f"twist tolerance {min(r['tw30']['b'] for r in g):.3f}-{max(r['tw30']['b'] for r in g):.3f} deg ({min(r['tw30']['b'] for r in g) / tw_base:.1f}-{max(r['tw30']['b'] for r in g) / tw_base:.1f} x B45), spiral stable in {sum(1 for r in g if r['modes']['spiral'] < 0)} of {len(g)}, "
          f"oscillation zeta {min(r['modes']['dr_zeta'] for r in g):+.3f}..{max(r['modes']['dr_zeta'] for r in g):+.3f}, Cnb > 0 in {sum(1 for r in g if r['deriv']['Cnb'] > 0)}, mass {min(r['mass_g'] for r in g):.1f}-{max(r['mass_g'] for r in g):.1f} g. "
          f"Best twist tolerance: {', '.join(f'{r['spec_name']} ({r['tw30']['b']:.3f})' for r in best_tw)}. Lowest alpha: {', '.join(f'{r['spec_name']} ({r['alpha']:.2f})' for r in best_a)}.")
    P("")
    P("### Stage 2: feature effects (matched pairs, median with range)\n")
    head = ["family", "change", "twist tolerance", "delta zeta", "delta spiral eig 1/s", "delta Cnb", "delta Clb", "delta trim alpha deg"]
    rr = []
    for fam, lab, st_ in feature_effects(res2):
        rr.append([fam, lab, fmt_stat(st_["tw"], "ratio"), fmt_stat(st_["zeta"]), fmt_stat(st_["spiral"], nd=4), fmt_stat(st_["cnb"], nd=4), fmt_stat(st_["clb"], nd=4), fmt_stat(st_["alpha"], nd=2)])
    P(md_t(head, rr))
    P("Twist-tolerance ratios have long upper tails: where the r_hat numerator nearly cancels the tolerance explodes (see the cancellation ratio in configs.csv), so read the median, not the range; the zeta, Cnb and Clb differences are not affected. Dihedral raises -Clb (more dihedral effect) by about "
      "0.06-0.07 per 10 deg of outer-panel dihedral (medians in the table) while Cnb falls: that is the trade the rule parts 2 and 3 pull on.\n")
    # ---- failures
    P("### What fails and why\n")
    el = [r for r in allr if not r["fixed"]]
    only = {k: sum(1 for r in el if r["rule"]["n_pass"] == 4 and not r["rule"][f"r{k}"]) for k in range(1, 6)}
    P("Eligible configurations failing each part: " + "; ".join(f"part {k}: {sum(1 for r in el if not r['rule'][f'r{k}'])}" for k in range(1, 6)) + f". Failing exactly one part (the part): {only}. "
      f"Trim alpha above 5 deg: {sum(1 for r in el if r['alpha'] > ALPHA_MAX)} of {len(el)}; Cnb <= 0: {sum(1 for r in el if r['deriv']['Cnb'] <= 0)}; spiral unstable beyond a 30 s doubling time: {sum(1 for r in el if not r['rule']['r1'])}; "
      f"oscillation zeta below 0.10: {sum(1 for r in el if r['modes']['dr_zeta'] < DR_ZETA_MIN)}.\n")
    nT = [r for r in res1 if r["family"] == "T"]
    P(f"Reading the failures: (i) family T is limited by its reflex device: a flap of 15 % chord over 0.15..0.40 of the semi-span needs {min(r['delta'] for r in nT):.1f}-{max(r['delta'] for r in nT):.1f} deg to trim CL 0.30 at SM 12 % "
      f"({sum(1 for r in nT if not r['trim_feasible'])} of 27 infeasible beyond 25 deg, none at or below 5 deg alpha in Stage 1); a larger reflex or a tip washout was not swept, so 'T fails' means 'T with this flap'. "
      "(ii) Holding SM at 12 % with a reflex trim costs alpha: configurations above 5 deg are nearly all T and the CL 0.35 variants; CL 0.25 is what brings alpha down. "
      "(iii) The printed-T masses come from the kit's default ballast station right behind the plate front, which needs a lot of lead; with the B45-style nose extension and station -37.5 mm the lead would be a fraction of that "
      "(a development check gave 25 g all-up instead of 34.7 g for T_AR4.5_S15_L0.3), so T inertias and speeds here are conservative. "
      "(iv) Dihedral buys twist tolerance and spiral stability but costs oscillation damping and Cnb (tables above); a fin buys Cnb and damping back.\n")
    # ---- SS correlation
    conv = [r for r in el if r["converged"]]
    s2 = [r for r in conv if r["stage"] == "2"]
    rho_all = spearman([r["rawdon"]["ss"] for r in conv], [r["tw30"]["b"] for r in conv])
    rho_s2 = spearman([r["rawdon"]["ss"] for r in s2], [r["tw30"]["b"] for r in s2])
    ties = lambda v: len(v) - len(set(v))
    P("### Rawdon SS against the AVL twist tolerance\n")
    f_ = lambda x: "n/a" if x is None else f"{x:+.2f}"
    P(f"Spearman rank correlation between SS and TW30, Stage 2 only: {f_(rho_s2)} (n={len(s2)}). Over all converged configurations it is {f_(rho_all)} (n={len(conv)}), but that figure includes {ties([round(r['rawdon']['ss'], 6) for r in conv])} ties, mostly the flat Stage-1 outlines with SS = 0 (EDA 0), so it flatters the agreement. "
      "A high rank correlation would say the textbook spiral number and the AVL turn sensitivity order the configurations alike; it is not a check on either.\n")
    P("### Rank, do not predict\n")
    P("AVL's Clb for thin, low-aspect-ratio plates at Re of 10-30k is unreliable (measured roll due to sideslip departs from vortex-lattice predictions because tip-vortex roll-up is destabilising; Linehan & Mohseni, cited in the robustness report) and CLa is uncertain by about +-30 %. "
      "Twist tolerances, spiral eigenvalues and damping ratios here therefore order configurations; they are not predicted radii or tolerances, and a configuration that 'passes' has passed a model rule, not a flight. "
      "The reflex flap and tail-bend effectiveness at low Re is below AVL's, the CG height and inertias are point-mass assumptions, and the 1 deg asymmetry is a unit of comparison, not a measured build error. "
      "Measured calibration (a B45 with a gauged wing-half twist, the circle radius timed on repeated drops) is the missing data point.\n")
    P("### Files\n")
    P("`configs.csv`, `sweep.gif`, `contact_sheet.svg`, `index.html`, `sheet_builder_top3/`, `runs.json`, `prestated_section.sha256`. Code: `rigid_glider/shape_sweep_v1.py`; tests `rigid_glider/test_shape_sweep_v1.py`. AVL scratch files: outside this folder (`--work`).\n")
    if tests_text:
        P(tests_text)
    return "\n".join(A)


def kit_mass_text(lines, r):
    m = re.search(r"all-up ([0-9.]+) g, ballast ([0-9.]+) g", " ".join(lines))
    if not m:
        return ""
    return f" (kit all-up {m.group(1)} g with {m.group(2)} g lead; the sweep's mass model said {r['mass_g']:.1f} g with {r['ballast_g']:.1f} g lead)"


FLAG_TEXT = "cancellation: twist number not credible"
CANCEL_CUT = 0.25                # POST-HOC cut (chosen after seeing the results; B45 and Swift A as flown are about 0.6)
GAMMA_ALT = -11.0                # deg: swoop_model_v1 glide angle (L/D about 6); the sweep itself uses per-configuration gamma from CD0 = 0.0307 (about -6.6 for Swift A)
CLB_FACTORS = (0.7, 1.0, 1.3)


def flagged(r):
    return cancel_ratio(r) < CANCEL_CUT


def tw_at(r, gamma, clb_factor=1.0):
    """Twist tolerances (deg for R = 30 m) of asymmetries a, b, c from the stored dCl, dCn and derivatives at glide angle gamma, with Clb scaled by clb_factor (closed form only, no AVL)."""
    d = r["deriv"]
    out = {}
    for k, v in r["asym"].items():
        rh = ss.rhat_gamma(d["Clb"] * clb_factor, d["Cnb"], d["Clr"], d["Cnr"], d["Clp"], d["Cnp"], v["dCl"], v["dCn"], gamma)[1]
        out[k] = (Z_B / abs(rh)) * ASYM if rh else float("inf")
    return out


def gamma_rerun(spec, st, r, work, gamma):
    """AVL system matrix at base-state elevation alpha + gamma (Ixz included) for an already trimmed configuration; returns the lateral modes and the mode shape."""
    has_ctl = spec.get("ctl", True)
    asm = mass_for(spec, st, r["cg"], work)
    deck = build_deck(spec, st, r["cg"], r["zcg"])
    extra = [f"E {r['alpha'] + gamma:.5f}", f"IZX {-asm['jxz_kgm2']:.6e}"]
    wd = work / "g11" / (r["name"] + f"_g{abs(gamma):g}")
    ss.avl_session(wd, "shp", deck, [(oper_cmds(r["alpha"], r["delta"], r["delta"], has_ctl), "base")], mass=ss.mass_file(asm), V=r["V"], mode_extra=extra)
    rows = ss.parse_sysmat_full(wd / "sys.out")
    modes = lat_modes([q[:12] for q in rows])
    A4, B4 = ss.lat_blocks(rows)
    cplx = [e for e in modes["eigs"] if e[1] > 1e-9]
    shape = ss.mode_shape(A4, r["V"], complex(*cplx[0])) if cplx else None
    return dict(spiral=modes["spiral"], zeta=modes["dr_zeta"], roll=modes["roll"], shape=shape)


def gamma_check(items, specmap, work: Path, log=print):
    """items: list of (label, result).  Per item: modes at its own gamma (a check that the rerun reproduces the stored values) and at GAMMA_ALT; closed-form twist tolerances at both."""
    cp = work / "gamma11.pkl"
    cache = pickle.load(open(cp, "rb")) if cp.is_file() else {}
    rows = {}
    for lab, r in items:
        spec = specmap[r["spec_name"]]
        fe = r["feat"]
        st = dict(name=r["name"], fin=tup(fe.get("fin")), dih=fe.get("dih", 0.0), cl=fe.get("cl", CL_TARGET))
        key = (r["name"], GAMMA_ALT)
        if key not in cache:
            cache[key] = (gamma_rerun(spec, st, r, work, r["gamma"]), gamma_rerun(spec, st, r, work, GAMMA_ALT))
            pickle.dump(cache, open(cp, "wb"))
            log(f"gamma check {r['name']}: spiral own {cache[key][0]['spiral']:+.4f} (stored {r['modes']['spiral']:+.4f}), at {GAMMA_ALT:g}: {cache[key][1]['spiral']:+.4f}")
        own, alt = cache[key]
        t0, t1 = tw_at(r, r["gamma"]), tw_at(r, GAMMA_ALT)
        p1 = lambda sp: bool(sp < 0 or sp < math.log(2) / SPIRAL_T2_MIN)
        rows[r["name"]] = dict(config=r["name"], role=lab, gamma_own_deg=round(r["gamma"], 3), gamma_alt_deg=GAMMA_ALT, spiral_stored_per_s=r["modes"]["spiral"], spiral_rerun_own_gamma_per_s=own["spiral"],
                               spiral_alt_per_s=alt["spiral"], part1_own=int(p1(r["modes"]["spiral"])), part1_alt=int(p1(alt["spiral"])), zeta_own=r["modes"]["dr_zeta"], zeta_alt=alt["zeta"],
                               beta_over_phi_own=(r["shape"] or {}).get("beta_over_phi", ""), beta_over_phi_alt=(alt["shape"] or {}).get("beta_over_phi", ""),
                               **{f"tw30_{k}_own": t0[k] for k in t0}, **{f"tw30_{k}_alt": t1[k] for k in t1})
    return rows


def clb_band_rows(items, base_res):
    """TW30(b) with Clb scaled by 0.7 / 1.0 / 1.3 (closed form, own gamma); ratio against B45 as flown at the same factor."""
    rows = []
    for lab, r in items:
        row = dict(config=r["name"], role=lab, cancellation_ratio=round(cancel_ratio(r), 3))
        for f in CLB_FACTORS:
            tw = tw_at(r, r["gamma"], f)["b"]
            tb = tw_at(base_res, base_res["gamma"], f)["b"]
            row[f"tw30_b_clb_x{f:g}"] = round(tw, 4)
            row[f"ratio_vs_B45_same_factor_x{f:g}"] = round(tw / tb, 2)
        row["keeps_3x_B45_at_all_factors"] = int(all(row[f"ratio_vs_B45_same_factor_x{f:g}"] >= TW_FACTOR for f in CLB_FACTORS))
        rows.append(row)
    return rows


def posthoc_section(allr, tw_base, b45r, sw_ref, sbp=None, g11=None):
    """POST-HOC reading (thresholds and the pick are chosen after seeing the results; labelled as such in the README).  Returns (markdown, pick)."""
    el = [r for r in allr if not r["fixed"] and r["trim_feasible"]]
    p134 = [r for r in el if r["rule"]["r1"] and r["rule"]["r3"] and r["rule"]["r4"] and r["rule"]["r5"] and r["deriv"]["Cnb"] > 0]
    robust = [r for r in p134 if cancel_ratio(r) >= 0.25]
    rows = sorted(robust, key=lambda r: -r["modes"]["dr_zeta"])
    L = []
    P = L.append
    P("### POST-HOC reading (NOT part of the pre-stated rule; chosen after seeing the results)\n")
    P(f"**The pre-stated top 3 rest on a cancellation.** Their twist tolerances are large because the two terms of the r_hat numerator (Cnb dCl and Clb dCn for the wing-half incidence) almost cancel: cancellation ratio |Cnb dCl - Clb dCn| / (|Cnb dCl| + |Clb dCn|) = "
      + ", ".join(f"{cancel_ratio(r):.3f}" for l, r in pick_top3(el)) + f" against {cancel_ratio(b45r):.2f} for B45 and {cancel_ratio(sw_ref):.2f} for Swift A as flown. A tolerance of 9 deg is not credible as a build tolerance; it says that, at that trim and dihedral, the model's roll and yaw responses to a twist happen to offset each other. "
      "Read those three rows as 'passes the pre-stated rule parts 1, 3, 4, 5 by cancellation', not as the best shapes.\n")
    P(f"**What stops every configuration is part 2 (oscillation damping >= 0.10).** {sum(1 for r in el if r['modes']['dr_zeta'] >= DR_ZETA_MIN and r['deriv']['Cnb'] > 0)} of {len(el)} feasible configurations reach zeta 0.10 with Cnb > 0 ({', '.join(r['name'] for r in el if r['modes']['dr_zeta'] >= DR_ZETA_MIN and r['deriv']['Cnb'] > 0) or 'none'}); "
      f"B45 and Swift A as flown have {b45r['modes']['dr_zeta']:+.3f} and {sw_ref['modes']['dr_zeta']:+.3f}. {len(p134)} configurations pass parts 1, 3, 4, 5 with Cnb > 0 and fail only on damping; {len(robust)} of them are not cancellation-dominated (ratio >= 0.25). "
      "The 0.10 threshold is borrowed from manned-aircraft practice (the robustness report: no small-glider criterion exists) and the damping depends on the ASSUMED point-mass inertias, so this is the least certain of the five parts.\n")
    head = ["configuration", "zeta", "twist deg (R30)", "x B45", "cancel ratio", "Cnb", "alpha deg", "spiral", "SM %", "mass g", "Rawdon SS"]
    P(md_t(head, [[r["name"], f"{r['modes']['dr_zeta']:+.3f}", f"{r['tw30']['b']:.3f}", f"{r['tw30']['b'] / tw_base:.1f}", f"{cancel_ratio(r):.2f}", f"{r['deriv']['Cnb']:+.4f}", f"{r['alpha']:.2f}", r["spiral_text"], f"{r['sm_pct']:.1f}", f"{r['mass_g']:.1f}", f"{r['rawdon']['ss']:.1f}"] for r in rows[:10]]))
    P("Count of configurations passing parts 1, 3, 4, 5 with Cnb > 0, not cancellation-dominated, and a relaxed damping threshold: " + "; ".join(f"zeta >= {t:g}: {sum(1 for r in robust if r['modes']['dr_zeta'] >= t)}" for t in (0.10, 0.07, 0.05, 0.04, 0.03)) + ".\n")
    pick = rows[0] if rows else None
    if pick:
        fe = pick["feat"]
        ke = lambda r: r["mass_g"] * r["V"] ** 2
        shp = (pick["shape"] or {}).get("beta_over_phi")
        g = (g11 or {}).get(pick["name"])
        P(f"**The post-hoc pick is a trade, not a candidate: {pick['name']}** (highest zeta among the {len(robust)} configurations that pass parts 1, 3, 4, 5, have Cnb > 0 and a cancellation ratio >= {CANCEL_CUT}; that cut is POST-HOC). "
          f"Outline {pick['outline']} ({', '.join(f'{k} {v:g}' if isinstance(v, float) else f'{k} {v}' for k, v in pick['params'].items() if k != 'levels')}), tip dihedral {fe['dih']:g} deg on the outer 40 %, {fin_text(fe)}, trim CL {fe['cl']:.2f}, SM 12 % MAC, {surface_name(pick)} {pick['delta']:.1f} deg.")
        P("- What it gains: " f"wing-half twist tolerance {pick['tw30']['b']:.3f} deg ({pick['tw30']['b'] / tw_base:.1f} x B45, {pick['tw30']['b'] / sw_ref['tw30']['b']:.1f} x Swift A); spiral {pick['spiral_text']}; Cnb {pick['deriv']['Cnb']:+.4f} (B45 {b45r['deriv']['Cnb']:+.4f}, Swift A {sw_ref['deriv']['Cnb']:+.4f}); trim alpha {pick['alpha']:.2f} deg.")
        P("- What it costs: " f"fin-toe tolerance TW30(c) {pick['tw30'].get('c', float('nan')):.3f} deg against {b45r['tw30'].get('c', float('nan')):.3f} deg for B45 (worse); oscillation damping zeta {pick['modes']['dr_zeta']:+.3f} below B45 {b45r['modes']['dr_zeta']:+.3f} (above Swift A {sw_ref['modes']['dr_zeta']:+.3f}), and it fails part 2; "
          f"trim speed {pick['V']:.1f} m/s against {b45r['V']:.1f} (B45 foam) and {sw_ref['V']:.1f} (Swift A foam), wing loading {pick['wing_loading_N_m2']:.1f} N/m2 ({pick['wing_loading_N_m2'] / b45r['wing_loading_N_m2']:.1f} x B45, {pick['wing_loading_N_m2'] / sw_ref['wing_loading_N_m2']:.1f} x Swift A), "
          f"kinetic energy m V^2 {ke(pick) / ke(b45r):.0f} x B45 and {ke(pick) / ke(sw_ref):.0f} x Swift A (consequences for pull-out height, impact and propeller strike); mass {pick['mass_g']:.1f} g with {pick['ballast_g']:.1f} g lead, set by the SM 12 % target and the kit's ballast station (a lighter layout was not swept)"
          + (f"; mode shape |beta|/|phi| {shp:.3f}, close to the 0.5 threshold used for calling the oscillation a Dutch roll" if shp is not None else "") + ".")
        if g:
            P(f"- Glide-angle sensitivity: at gamma {GAMMA_ALT:g} deg (instead of {pick['gamma']:.1f}) the spiral eigenvalue is {g['spiral_alt_per_s']:+.4f} 1/s (part 1 {'passes' if g['part1_alt'] else 'FAILS'}) and TW30(b) is {g['tw30_b_alt']:.3f} deg.")
        P("- Status: it is not a candidate (part 2 fails); whether to build it is the owner's decision. A sheet_builder config and flat kit are in `sheet_builder_top3/posthoc.json` (an addition to the three required, labelled post-hoc)"
          + (f"{kit_mass_text(sbp[0][2], pick)}." if sbp else "."))
        P("")
    return "\n".join(L), pick


def summary_text(state, allr, tw_base, b45r, sw_ref, top3):
    el = [r for r in allr if not r["fixed"]]
    fe = {(f, lab): st_ for f, lab, st_ in feature_effects(state["res2"])}
    ncand = sum(1 for r in el if r["rule"]["candidate"])
    feas = [r for r in el if r["trim_feasible"]]
    zbest = max((r for r in feas if r["deriv"]["Cnb"] > 0), key=lambda r: r["modes"]["dr_zeta"])
    pick = posthoc_section(allr, tw_base, b45r, sw_ref)[1]
    L = []
    P = L.append
    P("### Summary (read this first)\n")
    P(f"- **No configuration passes the pre-stated rule: {ncand} candidates of {len(el)} eligible configurations** (54 outlines flat, 180 Stage-2 variants). Part 5 (bed) passes everywhere; part 2 (oscillation damping >= 0.10 with Cnb > 0) fails in "
      f"{sum(1 for r in el if not r['rule']['r2'])}; the best damping of any feasible configuration with Cnb > 0 is {zbest['modes']['dr_zeta']:+.3f} ({zbest['name']}), against {b45r['modes']['dr_zeta']:+.3f} for B45 and {sw_ref['modes']['dr_zeta']:+.3f} for Swift A as flown. "
      "Part 2 is also the least certain part (point-mass inertias are ASSUMED).")
    P(f"- **Pre-stated top 3** (no candidates, so the near-miss rule fills it): " + "; ".join(f"{r['name']} (twist {r['tw30']['b']:.2f} deg, zeta {r['modes']['dr_zeta']:+.3f}, alpha {r['alpha']:.1f})" for l, r in top3)
      + ". All three are Swift-lineage (family B) with 20 deg tip dihedral and a 2 % S centre fin and carry the marker 'cancellation: twist number not credible'; their huge twist tolerances are cancellations of the r_hat numerator (cancellation ratio "
      + ", ".join(f"{cancel_ratio(r):.2f}" for l, r in top3) + f" against {cancel_ratio(b45r):.2f} for B45), so they are not good evidence. See the POST-HOC reading below.")
    if pick:
        P(f"- **Post-hoc pick (a trade, not a candidate; details in the POST-HOC reading):** {pick['name']}: twist tolerance {pick['tw30']['b'] / tw_base:.1f} x B45, but fin-toe tolerance worse than B45, zeta {pick['modes']['dr_zeta']:+.3f} below B45, trim speed {pick['V']:.1f} m/s and mass {pick['mass_g']:.1f} g (printed, {pick['ballast_g']:.1f} g lead). It fails part 2.")
    g = lambda f, lab, k, kind="diff", nd=3: fmt_stat(fe[(f, lab)][k], kind, nd).split(" (")[0]
    P("- **Features (Stage 2 medians):** tip dihedral 0 -> 20 deg multiplies the twist tolerance by " + g("T", "tip dihedral 0 -> 20 deg", "tw", "ratio") + " (T) and " + g("B", "tip dihedral 0 -> 20 deg", "tw", "ratio") + " (B) but moves zeta by "
      + g("T", "tip dihedral 0 -> 20 deg", "zeta") + " and " + g("B", "tip dihedral 0 -> 20 deg", "zeta") + " and Cnb by " + g("T", "tip dihedral 0 -> 20 deg", "cnb", "diff", 4) + " and " + g("B", "tip dihedral 0 -> 20 deg", "cnb", "diff", 4)
      + "; tip fins on T add " + g("T", "fin: no fin -> tip fins cant 0", "cnb", "diff", 4) + " to Cnb, canting them 20 deg outward changes zeta by " + g("T", "fin: tip fins cant 0 -> tip fins cant 20", "zeta") + " (no gain); a centre fin on B adds "
      + g("B", "fin: no fin -> centre fin 2% S", "cnb", "diff", 4) + " (2 % S) and " + g("B", "fin: no fin -> centre fin 4% S", "cnb", "diff", 4) + " (4 % S) to Cnb and " + g("B", "fin: no fin -> centre fin 2% S", "zeta") + " / " + g("B", "fin: no fin -> centre fin 4% S", "zeta")
      + " to zeta; trim CL 0.25 -> 0.35 raises the trim alpha by " + g("T", "trim CL 0.25 -> 0.35 (SM held)", "alpha", "diff", 2) + " (T) / " + g("B", "trim CL 0.25 -> 0.35 (SM held)", "alpha", "diff", 2) + " deg and does not move the twist tolerance systematically (median ratio "
      + g("B", "trim CL 0.25 -> 0.35 (SM held)", "tw", "ratio") + " for B).")
    bf = [r for r in feas if r["stage"] == "1" and r["family"] == "B"]
    tf = [r for r in feas if r["stage"] == "1" and r["family"] == "T"]
    P(f"- **Outlines (Stage 1, flat):** family B trims at alpha {min(r['alpha'] for r in bf):.1f}-{max(r['alpha'] for r in bf):.1f} deg with a tail bend of {min(r['delta'] for r in bf):.0f}-{max(r['delta'] for r in bf):.0f} deg and is spiral-stable in {sum(1 for r in bf if r['modes']['spiral'] < 0)} of {len(bf)}; "
      f"family T with a 15 % flap trims at {min(r['alpha'] for r in tf):.1f}-{max(r['alpha'] for r in tf):.1f} deg (never at or below 5 deg) and {sum(1 for r in el if r['stage'] == '1' and r['family'] == 'T' and not r['trim_feasible'])} of 27 T outlines cannot be trimmed within 25 deg of flap. "
      f"No flat outline reaches the twist tolerance of rule 3 ({TW_FACTOR * tw_base:.3f} deg); the best flat outline reaches {max(r['tw30']['b'] for r in feas if r['stage'] == '1'):.3f} deg. Dihedral is what moves it.")
    P(f"- **Printed mass:** printed versions are {min(r['mass_g'] for r in feas):.0f}-{max(r['mass_g'] for r in feas):.0f} g (lead ballast {min(r['ballast_g'] for r in feas):.0f}-{max(r['ballast_g'] for r in feas):.0f} g) because the CG target of SM 12 % is far ahead of the bare printed sheet's CG; B45 foam is {b45r['mass_g']:.1f} g and Swift A foam {sw_ref['mass_g']:.1f} g. The dynamic numbers carry that mass and inertia. "
      "A lighter build (foam, or moving mass into the fin and tail) was not swept.")
    P(f"- **Glide angle is unmeasured.** This sweep uses a per-configuration gamma from CD0 = 0.0307 (about -6.6 deg for Swift A); `swoop_model_v1` gives about {GAMMA_ALT:g} deg (L/D about 6). The modes and closed form were rerun at {GAMMA_ALT:g} deg for the references, the top 3 and the post-hoc pick (section 'Post-hoc checks' below; `gamma11_check.csv`).")
    P("- **Direction is a lateral-only ranking.** Swift A as flown passes rule 4 (alpha " + f"{sw_ref['alpha']:.2f}" + " deg) yet swooped heavily with stalls in flight, and neither pitch model explains that, so pitch behaviour of the B family (and of every configuration here) is unvalidated; rule 4 and the pitch margin are screening quantities only.")
    P("- **Next flight day:** record the flights from the side or from behind to check for a yaw-roll wobble; the existing video is from beneath and says nothing about lateral oscillation. Measured glide angle and a gauged wing-half twist test would calibrate the two largest model uncertainties.")
    P("- **Caveat, rank do not predict:** inviscid AVL on thin low-aspect-ratio plates (Clb unreliable at Re 10-30k, CLa about +-30 %); every number here ranks configurations and is not a flight prediction.\n")
    return "\n".join(L)


def review_text(g11, band, b45r):
    L = []
    P = L.append
    P("### Post-hoc checks: glide angle and the Clb band (POST-HOC, not part of the pre-stated rule)")
    P("")
    P(f"**Glide angle.** Each configuration's own gamma comes from CD0 = 0.0307 (Swift A about -6.6 deg); `swoop_model_v1` gives about {GAMMA_ALT:g} deg. Modes were rerun in AVL at theta0 = alpha + gamma (Ixz included) at both angles, and the closed-form twist tolerance TW30 (deg for R = 30 m) was recomputed for asymmetries a (differential trim surface), b (wing-half incidence), c (fin toe). "
      "`spiral own` is the rerun at the own gamma and reproduces the stored value (a check of the rerun). Part 1 is the 30 s spiral rule. Full numbers: `gamma11_check.csv`.")
    P("")
    head = ["configuration", "role", "gamma own", "spiral own 1/s", "part 1 own", f"spiral at {GAMMA_ALT:g} 1/s", f"part 1 at {GAMMA_ALT:g}", "zeta own", f"zeta at {GAMMA_ALT:g}", "TW30 a/b/c own", f"TW30 a/b/c at {GAMMA_ALT:g}"]
    fmt = lambda x: "-" if x is None else f"{x:.3f}"
    rows = []
    for g in g11.values():
        rows.append([g["config"], g["role"], f"{g['gamma_own_deg']:.2f}", f"{g['spiral_rerun_own_gamma_per_s']:+.4f}", "pass" if g["part1_own"] else "FAIL", f"{g['spiral_alt_per_s']:+.4f}", "pass" if g["part1_alt"] else "FAIL",
                     f"{g['zeta_own']:+.3f}", f"{g['zeta_alt']:+.3f}", " / ".join(fmt(g.get(f"tw30_{k}_own")) for k in "abc"), " / ".join(fmt(g.get(f"tw30_{k}_alt")) for k in "abc")])
    P(md_t(head, rows))
    P("")
    P(f"**Clb band.** AVL's Clb is uncertain on thin low-aspect-ratio plates, so TW30(b) was recomputed with Clb scaled by 0.7 and 1.3 (closed form only, no AVL rerun; own gamma); the ratio is against B45 as flown at the same factor. "
      f"The 'robust' set uses a cancellation-ratio cut of {CANCEL_CUT}, which is POST-HOC (chosen after seeing the results). Full table: `clb_band.csv`.")
    P("")
    head = ["configuration", "role", "cancel ratio", "TW30(b) Clb x0.7", "x B45", "TW30(b) Clb x1.0", "x B45", "TW30(b) Clb x1.3", "x B45", "keeps 3x at all factors"]
    sel = [b for b in band if not b["role"].startswith("robust")]
    rows = [[b["config"], b["role"], f"{b['cancellation_ratio']:.3f}", f"{b['tw30_b_clb_x0.7']:.3f}", f"{b['ratio_vs_B45_same_factor_x0.7']:.1f}", f"{b['tw30_b_clb_x1']:.3f}", f"{b['ratio_vs_B45_same_factor_x1']:.1f}",
             f"{b['tw30_b_clb_x1.3']:.3f}", f"{b['ratio_vs_B45_same_factor_x1.3']:.1f}", "yes" if b["keeps_3x_B45_at_all_factors"] else "no"] for b in sel]
    P(md_t(head, rows))
    rb = [b for b in band if b["role"].startswith("robust")]
    if rb:
        P("")
        P(f"Of the {len(rb)} 'robust' configurations, {sum(b['keeps_3x_B45_at_all_factors'] for b in rb)} keep at least 3 x the B45 twist tolerance (same Clb factor) at all three factors; at Clb x0.7: {sum(1 for b in rb if b['ratio_vs_B45_same_factor_x0.7'] >= TW_FACTOR)}, at x1.3: {sum(1 for b in rb if b['ratio_vs_B45_same_factor_x1.3'] >= TW_FACTOR)}. "
          f"Range of the ratio over the set: x0.7 {min(b['ratio_vs_B45_same_factor_x0.7'] for b in rb):.1f}-{max(b['ratio_vs_B45_same_factor_x0.7'] for b in rb):.1f}, x1.3 {min(b['ratio_vs_B45_same_factor_x1.3'] for b in rb):.1f}-{max(b['ratio_vs_B45_same_factor_x1.3'] for b in rb):.1f}.")
    P("")
    return "\n".join(L)


def md_t(head, rows):
    return ss.md_table(head, rows)


def write_outputs(state, out: Path, work: Path, log=lambda m: print(m, flush=True)):
    t0 = time.time()
    tw_base, allr = prepare(state)
    rows = config_rows(allr, tw_base)
    specs = outline_specs() + [sp for sp, st in ref_specs()]
    specmap = spec_by_name(specs)
    write_csv_rows(out / "configs.csv", rows)
    el = [r for r in allr if not r["fixed"]]
    top3 = pick_top3(el)
    pts = [(r["tw30"]["b"], pitch_margin(r), r["name"]) for r in el if r["trim_feasible"]]
    front = pareto_front(pts)
    log(f"top3: {[(l, r['name']) for l, r in top3]}; pareto front {len(front)}")
    frames = render_frames(state, specmap, tw_base)
    log(f"frames {len(frames)} rendered in {time.time() - t0:.0f} s; encoding")
    gif = gif_bytes([f for _, f in frames], FW, FH, delay_cs=60)
    (out / "sweep.gif").write_bytes(gif)
    (out / "contact_sheet.svg").write_text(contact_sheet(state, specmap, tw_base), encoding="utf-8")
    top_names = [r["name"] for _, r in top3]
    (out / "index.html").write_text(html_index(rows, allr, top3, pareto_svg(allr, top_names, set(front), tw_base), tw_base, state["wall"], len(frames)), encoding="utf-8")
    refs_ = {r["name"]: r for r in state["refs"]}
    _, pick = posthoc_section(allr, tw_base, refs_["B45_as_flown"], refs_["SwiftA_as_flown"])
    items = list(top3) + ([("POST-HOC PICK", pick)] if pick else [])
    chk_items = [("reference", r) for r in state["refs"]] + [(l, r) for l, r in items]
    g11 = gamma_check(chk_items, specmap, work, log)
    write_csv_rows(out / "gamma11_check.csv", list(g11.values()))
    p134_ = [r for r in el if r["trim_feasible"] and r["rule"]["r1"] and r["rule"]["r3"] and r["rule"]["r4"] and r["rule"]["r5"] and r["deriv"]["Cnb"] > 0 and cancel_ratio(r) >= CANCEL_CUT]
    band_items = [("reference", r) for r in state["refs"]] + [(l, r) for l, r in items] + [("robust (ratio >= 0.25, post-hoc cut)", r) for r in p134_]
    band = clb_band_rows(band_items, refs_["B45_as_flown"])
    write_csv_rows(out / "clb_band.csv", band)
    sbn_all = write_sheet_builder_top3(out, items, specmap, log) if items else []
    sbn, sbp = sbn_all[:len(top3)], sbn_all[len(top3):]
    ph_text, _ = posthoc_section(allr, tw_base, refs_["B45_as_flown"], refs_["SwiftA_as_flown"], sbp, g11)
    ph_text += review_text(g11, band, refs_["B45_as_flown"])
    slim = [{k: v for k, v in r.items() if k not in ("inertia",)} for r in allr]
    (out / "runs.json").write_text(json.dumps(ss._json_safe(dict(tw_base=tw_base, runs=slim, selected=state["selected"], notes=state["notes"], fails=state["fails"], top3=[(l, r["name"]) for l, r in top3], pareto_front=front, wall_s=state["wall"])), indent=1), encoding="utf-8")
    txt = readme_results(state, allr, rows, tw_base, top3, front, sbn, posthoc_text=ph_text)
    rp = out / "README.md"
    raw = rp.read_bytes()
    marker = b"\nRESULTS\n"
    head = raw.split(marker)[0]
    rp.write_bytes(head + marker + txt.encode("utf-8") + b"\n")
    log(f"outputs written in {time.time() - t0:.0f} s; gif {len(gif) / 1e6:.1f} MB, {len(frames)} frames")


def write_csv_rows(path, rows):
    names = list(dict.fromkeys(k for r in rows for k in r))
    with Path(path).open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=names)
        w.writeheader()
        w.writerows(rows)


if __name__ == "__main__":
    main()
