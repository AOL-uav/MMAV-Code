"""Pull-out simulation for the nose-down drop start.

Question: a glider hangs nose-down from a string under a quad and is released at rest.  Does it pull out into a glide by itself, how much height does it
lose, and how does that depend on the CG position and on a built-in nose-up moment (an up-bent trailing-edge tab on the flat wings, a tail bend on the
bird outline)?

MODEL (kept small and inspectable; every piece is tagged on the page):
  * 3-DOF longitudinal rigid body in still air: state (x, z, theta, vx, vz, q), fixed-step RK4.  No quad downwash, no unsteady aerodynamics, no added mass.
  * Normal and axial force coefficients and the pitching moment vs angle of attack: MEASURED Ananda 2012 rectangular-plate tables (read through plate_balance.py),
    made exactly symmetric (odd/even parts), nearest tested aspect ratio and taper, lowest tested Reynolds number (60 000).  Carry-over to our tapered, swept,
    finned outlines: ASSUMED.  Beyond the tabulated angle (about 29 deg) a stated flat-plate formula (ASSUMED) is blended in continuously out to 90 deg.
  * Aerodynamic centre moved to the AVL neutral point (MODEL) -- or, as an alternative case, the table's quarter chord is placed on the neutral point.
  * Zero-lift moment and lift change from the tab / tail bend, and pitch damping Cmq: AVL (MODEL).  The measured zero-lift moment offset range and the AC
    alternative are sensitivity cases.
  * Mass, CG and pitch inertia: CALCULATED from the kit mass breakdowns (flat wings) or the bird_outline_v2 sheet / rail masses.
Nothing here is a flight result.  AVL runs are serial and light.
    py -B rigid_glider/pullout_sim.py [--out DIR] [--quick]
"""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import argparse
import bisect
import csv
import html
import json
import math
import re
import subprocess
from pathlib import Path

import bird_outline as bo
import bird_outline_v2 as v2
import flat_test as ft
import plate_balance as pb

ROOT = ft.ROOT
OUT_DEFAULT = ROOT / "results/runs/rigid_glider/pullout"
KIT = ROOT / "results/runs/rigid_glider/flat_test_v2"
AVL_EXE = ft.AVL_EXE
RHO, NU, G = ft.RHO_AIR, ft.NU_AIR, 9.80665
RE_MEASURED_MIN = 60000.0
CN90 = 1.2                         # ASSUMED normal-force coefficient of a flat plate broadside (handbook order of magnitude 1.2 for aspect ratios up to about 6)
BLEND_DEG = 15.0                   # ASSUMED: the measured table fades into the flat-plate formula over this many degrees beyond its last point
TAB_SPAN_FRAC, TAB_CHORD_FRAC = 0.5, 0.20      # JUDGEMENT: tab = aft 20 % of the chord over the inner 50 % of each semi-span, hinge at 80 % chord
TABS = (0.0, 2.0, 4.0, 6.0, 8.0)               # deg, trailing edge UP
BENDS = (0.0, 2.5, 5.0, 7.5)                   # deg, tail bend, trailing edge up
DT = 0.002
T_MAX = 30.0
GAMMA_LEVEL = -20.0                # deg: 'first level-off' = first time the flight path is shallower than this
SETTLE_BAND = 5.0                  # deg: settled = flight path stays within +-5 deg of its final value
GLIDE_MAX_STEEP = -45.0            # deg: a settled flight path steeper than this is a dive, not a glide
DEEP_STALL_MARGIN_DEG = 10.0       # JUDGEMENT: a settled glide counts as DEEP STALL only if its angle of attack exceeds the first lift peak by more than this
CANNOT_SAY_SM_PCT = 3.0            # cases with |static margin| below this (% MAC) are marked 'model cannot say'
EFFECTIVENESS = (0.75, 0.5)        # tab / bend effectiveness sensitivity: fraction of the AVL value
CM_OFFSETS = (-0.008, 0.003)       # measured zero-lift moment range (plate_balance step 2), sensitivity cases
SPECS = {"F45_span250": dict(key="F45", thick="0.6", kind="printed"), "F60_span250": dict(key="F60", thick="0.6", kind="printed"), "B45_span400": dict(key="B45", thick="5.0", kind="foam")}


# =============================================================================================== measured plate table
def smooth(t):
    t = min(max(t, 0.0), 1.0)
    return t * t * (3 - 2 * t)


class PlateTable:
    """Symmetric normal-force / axial-force / moment (about c/4, nose-up positive) tables from one Ananda run, plus the flat-plate extension."""

    def __init__(self, ar_target, taper_target, rows=None, runs=None):
        rows = rows or pb.read_rows()
        runs = runs or pb.group_runs(rows)
        tested = sorted({(k[0], k[1]) for k in runs})
        near = pb.nearest_wings(ar_target, taper_target, tested)[0]
        cands = [v for k, v in runs.items() if (k[0], k[1]) == near]
        run = min(cands, key=lambda v: v["Re"])
        self.wing, self.run, self.Re = pb.wing_id(*near), run["run"], run["Re"]
        self.ar, self.taper = near
        legs = [p for p in run["sweeps"].values() if len(p) > 15]
        grid = [0.5 * i for i in range(0, 61)]                                       # 0 .. 30 deg
        tabs = []
        acs = []
        for pts in legs:
            al = [p["alpha"] for p in pts]
            f = lambda key: (lambda a: _interp(al, [p[key] for p in pts], a))
            fcl, fcd, fcm = f("CL"), f("CD"), f("CM")
            amax_pos, amax_neg = max(al), -min(al)
            a_sym = min(amax_pos, amax_neg)
            col = {"CL": [], "CD": [], "CM": []}
            # odd / even parts inside the symmetric range; beyond it the positive side is used with the constant even part at the edge removed
            e_cl = 0.5 * (fcl(a_sym) + fcl(-a_sym)); e_cm = 0.5 * (fcm(a_sym) + fcm(-a_sym))
            for a in grid:
                if a > amax_pos:
                    break
                if a <= a_sym:
                    col["CL"].append(0.5 * (fcl(a) - fcl(-a))); col["CM"].append(0.5 * (fcm(a) - fcm(-a))); col["CD"].append(0.5 * (fcd(a) + fcd(-a)))
                else:
                    col["CL"].append(fcl(a) - e_cl); col["CM"].append(fcm(a) - e_cm); col["CD"].append(fcd(a))
            tabs.append((col, len(col["CL"])))
            fit = pb.fit_low_angle(pb.annotate(pts))
            if fit:
                acs.append(fit["ac"])
        n = min(t[1] for t in tabs)
        self.a = grid[:n]
        avg = lambda key: [sum(t[0][key][i] for t in tabs) / len(tabs) for i in range(n)]
        cl, cd, cm = avg("CL"), avg("CD"), avg("CM")
        self.cl, self.cd, self.cm4 = cl, cd, cm
        self.cn = [cl[i] * math.cos(math.radians(self.a[i])) + cd[i] * math.sin(math.radians(self.a[i])) for i in range(n)]
        self.ca = [cd[i] * math.cos(math.radians(self.a[i])) - cl[i] * math.sin(math.radians(self.a[i])) for i in range(n)]
        self.a_end = self.a[-1]
        self.ac_meas = sum(acs) / len(acs)                                           # fraction of MAC from the leading edge, from the low-angle fit
        self.cl_alpha_deg = (cl[8] - cl[0]) / (self.a[8] - self.a[0]) if n > 9 else 0.0       # per degree over 0..4 deg
        # first lift peak: the plate_balance.py rule (slope falls below 25 % of the low-angle slope), applied to the symmetric table
        br = [dict(alpha=self.a[k], CL=cl[k]) for k in range(n)]
        k_pk, self.peak_kind = pb.first_peak(br)
        a_pk = self.a[k_pk]
        self.alpha_stall = a_pk
        self.cl_peak = cl[k_pk]
        end_cn, end_cm = self.cn[-1], self.cm4[-1]
        self.xcp_end = min(max(0.25 - end_cm / end_cn, 0.2), 0.5) if abs(end_cn) > 0.05 else 0.35

    def coeffs(self, alpha_deg):
        """-> (CN, CA, CM4) for a signed angle in [-90, 90]: CN and CM4 odd, CA even."""
        s = 1.0 if alpha_deg >= 0 else -1.0
        a = min(abs(alpha_deg), 90.0)
        if a <= self.a_end:
            cn, ca, cm = (_interp(self.a, arr, a) for arr in (self.cn, self.ca, self.cm4))
        else:
            w = smooth((a - self.a_end) / BLEND_DEG)
            cn = self.cn[-1] * (1 - w) + CN90 * math.sin(math.radians(a)) * w
            ca = self.ca[-1] * math.cos(math.radians(a)) / math.cos(math.radians(self.a_end))
            xcp = self.xcp_end + (0.5 - self.xcp_end) * (a - self.a_end) / (90.0 - self.a_end)
            cm = -cn * (xcp - 0.25)
        return s * cn, ca, s * cm

    def xcp_table(self, alpha_deg):
        """Centre of pressure (fraction of MAC, table coordinates) -- only used for reversed flow; the AC where the normal force is tiny."""
        cn, ca, cm = self.coeffs(alpha_deg)
        return self.ac_meas if abs(cn) < 0.05 else min(max(0.25 - cm / cn, 0.0), 1.0)


def _interp(xs, ys, x):
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    i = bisect.bisect_right(xs, x) - 1
    t = (x - xs[i]) / (xs[i + 1] - xs[i])
    return ys[i] + t * (ys[i + 1] - ys[i])


# =============================================================================================== AVL (MODEL)
NUM = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[EeDd][-+]?\d+)?"


def _f(text, key, last=False):
    m = re.findall(rf"\b{re.escape(key)}\s*=\s*({NUM})", text)
    if not m:
        return None
    return float((m[-1] if last else m[0]).replace("D", "E"))


def avl_session(deck_text, name, workdir: Path, xcg_m=None, has_control=False):
    """One AVL session at alpha = 0: totals (CL, Cm about the reference point) and the stability / control derivatives.  Serial, light."""
    workdir.mkdir(parents=True, exist_ok=True)
    deck = workdir / f"{name}.avl"
    deck.write_text(deck_text, encoding="ascii")
    keys = ["PLOP", "G", "", "OPER"]
    if xcg_m is not None:
        keys += ["M", f"X {xcg_m:.9f}", ""]
    keys += ["A A 0"] + (["D1 D1 0"] if has_control else []) + ["X", "ST", "", "", "QUIT"]
    (workdir / f"{name}.keys").write_text("\n".join(keys) + "\n")
    r = subprocess.run([str(AVL_EXE), deck.name], input="\n".join(keys) + "\n", text=True, cwd=workdir, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=300)
    (workdir / f"{name}.log").write_text(r.stdout, encoding="utf-8", errors="replace")
    t = r.stdout
    i = t.index("Stability-axis derivatives")
    st = t[i:]
    tot = t[t.rindex("Vortex Lattice Output -- Total Forces", 0, i):i]
    out = dict(CL0=_f(tot, "CLtot"), Cm0_ref=_f(tot, "Cmtot"), Xref=_f(tot, "Xref"), Sref=_f(tot, "Sref"), Cref=_f(tot, "Cref"), Bref=_f(tot, "Bref"),
               CLa_rad=_f(st, "CLa"), Cma_rad=_f(st, "Cma"), Cmq=_f(st, "Cmq"), Xnp=_f(st, "Xnp"))
    if has_control:
        out["CLd_per_deg"], out["Cmd_per_deg"] = _f(st, "CLd01"), _f(st, "Cmd01")
    return out


def inject_tab(deck_text, span_frac=TAB_SPAN_FRAC, hinge=1.0 - TAB_CHORD_FRAC):
    """Copy of an F-kit deck with a trailing-edge tab (AVL CONTROL named tab) on the inner part of each semi-span.  Reads the two wing SECTION lines."""
    m = re.search(r"SECTION\n([^\n]+)\nNACA\n0000\nSECTION\n([^\n]+)\nNACA\n0000\n", deck_text)
    assert m, "wing sections not found"
    r = [float(v) for v in m.group(1).split()]
    t = [float(v) for v in m.group(2).split()]
    mid = [r[i] + span_frac * (t[i] - r[i]) for i in range(5)]
    sec = lambda v: " ".join(f"{x:.9f}" for x in v)
    ctl = f"CONTROL\ntab 1.0 {hinge:.3f} 0.0 0.0 0.0 1.0\n"
    new = f"SECTION\n{sec(r)}\nNACA\n0000\n{ctl}SECTION\n{sec(mid)}\nNACA\n0000\n{ctl}SECTION\n{sec(t)}\nNACA\n0000\n"
    return deck_text.replace(m.group(0), new)


# =============================================================================================== mass properties (CALCULATED)
def poly_moments(poly):
    """Area, centroid x, and the second moment about the centroid of x (int (x-xc)^2 dA) of a simple polygon."""
    A = cx = i2 = 0.0
    n = len(poly)
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        c = x0 * y1 - x1 * y0
        A += c / 2
        cx += (x0 + x1) * c / 6
        i2 += (x0 * x0 + x0 * x1 + x1 * x1) * c / 12
    cx /= A
    return A, cx, i2 - A * cx * cx


class MassModel:
    """Components: (mass_g, x_centroid_mm, own pitch inertia about its centroid in g mm^2).  CG and pitch inertia about the CG follow."""

    def __init__(self):
        self.parts = []

    def add_plate(self, name, mass_g, poly_mm):
        A, cx, i2 = poly_moments(poly_mm)
        self.parts.append((name, mass_g, cx, mass_g * i2 / A))

    def add_bar(self, name, mass_g, x0, x1):
        self.parts.append((name, mass_g, 0.5 * (x0 + x1), mass_g * (x1 - x0) ** 2 / 12))

    def add_point(self, name, mass_g, x):
        self.parts.append((name, mass_g, x, 0.0))

    def mass_g(self):
        return sum(p[1] for p in self.parts)

    def cg_mm(self):
        return sum(p[1] * p[2] for p in self.parts) / self.mass_g()

    def inertia_kg_m2(self, cg_mm=None):
        cg = self.cg_mm() if cg_mm is None else cg_mm
        return sum(i + m * (x - cg) ** 2 for _, m, x, i in self.parts) * 1e-3 * 1e-6          # g mm^2 -> kg m^2


def flat_specimen(spec, out: Path, log=print):
    info = SPECS[spec]
    G_ = json.loads((KIT / spec / "geometry.json").read_text())
    v = lambda k: G_[k]["value"]
    scale = next(r for r in csv.DictReader((KIT / "scale_table.csv").open(encoding="utf-8")) if r["wing"] == info["key"] and r[[k for k in r if k.startswith("thickness_mm")][0]] == info["thick"])
    col = lambda prefix: float(scale[[k for k in scale if k.startswith(prefix)][0]])
    ballast_rows = [r for r in csv.DictReader((KIT / "ballast_table.csv").open(encoding="utf-8")) if r["wing"] == info["key"] and r["thickness_mm"] == info["thick"]]
    bk = list(ballast_rows[0].keys())
    cgs_all = sorted({float(r[bk[2]]) for r in ballast_rows})
    nose = v("stick_nose_ahead_of_root_le_mm")
    station_want = -ft.BALLAST_STATION_FRAC_OF_NOSE * nose
    stations = sorted({float(r[bk[4]]) for r in ballast_rows})
    station = min(stations, key=lambda s: abs(s - station_want))
    span, cr, ct, tip_x, mac, mac_le = v("span_mm"), v("root_chord_mm"), v("tip_chord_mm"), v("tip_le_x_mm"), v("mac_mm"), v("mac_le_x_mm")
    poly = [(0, 0), (tip_x, span / 2), (tip_x + ct, span / 2), (cr, 0), (tip_x + ct, -span / 2), (tip_x, -span / 2)]
    S_m2 = v("area_mm2") * 1e-6
    # CG settings: 6 values spread over the kit table (both ends included)
    idx = sorted({round(i * (len(cgs_all) - 1) / 5) for i in range(6)})
    cgs = [cgs_all[i] for i in idx]

    def masses(cg):
        ball = next(float(r[bk[5]]) for r in ballast_rows if float(r[bk[2]]) == cg and float(r[bk[4]]) == station)
        m = MassModel()
        m.add_plate("plate", col("plate_mass_g"), poly)
        if col("rib_mass_g") > 0:
            m.add_point("rib", col("rib_mass_g"), v("rib_x_centroid_mm"))
        m.add_point("2 tip fins", col("fins_mass_g"), tip_x + ct / 2)
        m.add_bar("stick", col("stick_mass_g"), -nose, cr)
        m.add_point("ballast", ball, station)
        return m, ball
    # AVL: fins deck + tab, at three CGs
    deck0 = (KIT / "avl_work" / spec / "fins" / "cl03" / "fins.avl").read_text()
    deck_tab = inject_tab(deck0).replace("fins\n", f"{spec}_tab\n", 1)
    avl = {}
    for cg in (cgs[0], cgs[len(cgs) // 2], cgs[-1]):
        avl[cg] = avl_session(deck_tab, f"{spec}_tab_cg{cg:g}", out / "avl", cg * 1e-3, True)
        log(f"  AVL {spec} cg {cg:g}: Xnp {avl[cg]['Xnp'] * 1000:.2f} mm, Cmq {avl[cg]['Cmq']:.3f}")
    return dict(name=spec, kind=info["kind"], key=info["key"], ar=v("aspect_ratio"), taper=v("taper"), S=S_m2, mac_m=mac * 1e-3, mac_le_mm=mac_le, cgs=cgs, cgs_all=cgs_all, masses=masses, avl=avl,
                station=station, nose=nose, span_mm=span, cg_table=(cgs_all[0], cgs_all[-1]), tab_note=f"inner {100 * TAB_SPAN_FRAC:g}% of each semi-span, aft {100 * TAB_CHORD_FRAC:g}% of the chord")


def bird_specimen(size_key, out: Path, log=print):
    k = v2.SIZES[size_key]["span_mm"]
    on = {"curved_te": 1, "root_blend": 1, "tip_round": 1, "neck": 1, "nose": 1}          # the v2 BS final outline
    p = v2.feature_params("BS", on)
    g = v2.build_geometry2(p)
    v2.AVL_NCHORD, v2.AVL_NSPAN = 10, 80
    d0 = v2.avl_deck2(g, k, 0.0, name="bs_b0")
    d5 = v2.avl_deck2(g, k, 5.0, name="bs_b5")
    a0 = avl_session(d0, f"bird{int(k)}_b0", out / "avl")
    a5 = avl_session(d5, f"bird{int(k)}_b5", out / "avl")
    mac_mm, le_mm = g["mac"] * k, g["x_le_mac"] * k
    xnp = a0["Xnp"] * 1000
    cm_np = lambda a: a["Cm0_ref"] + a["CL0"] * (xnp * 1e-3 - a["Xref"]) / a["Cref"]            # moment about the neutral point
    dcl = (a5["CL0"] - a0["CL0"]) / 5.0
    dcm_ac = (cm_np(a5) - cm_np(a0)) / 5.0
    trim_cg = xnp - mac_mm * (a5["Cm0_ref"] + a5["CL0"] * (xnp * 1e-3 - a5["Xref"]) / a5["Cref"]) / 0.3          # trim CG at CL 0.3 at 5 deg bend (recomputed here)
    cgs = [trim_cg - 0.05 * mac_mm, trim_cg, trim_cg + 0.05 * mac_mm]
    poly = [(x * k, y * k) for x, y in g["full_outline"]]

    def masses(cg):
        res = {"trim": {c: dict(x_cg_span=cg / k, static_margin=0.0) for c in v2.TRIM_CLS}}
        n = v2.size_numbers(g, res, size_key, 5.0)
        m = MassModel()
        m.add_plate("sheet", n["m_sheet"], poly)
        m.add_bar("rail", n["rail"]["mass_g"], n["rail"]["x_lo"], n["rail"]["saddle_end"])
        m.add_point("ballast", n["ballast_g"] if n["reachable"] else 0.0, n["station"] if n["station"] is not None else 0.0)
        return m, n["ballast_g"]
    cgd = {cg: dict(a0, Cmq=a0["Cmq"], Cma_rad=a0["Cma_rad"]) for cg in cgs}
    return dict(name=f"BS_{size_key}_{int(k)}", kind=size_key, key="BS", ar=g["ar"], taper=0.5, S=g["S"] * (k * 1e-3) ** 2, mac_m=mac_mm * 1e-3, mac_le_mm=le_mm, cgs=cgs, masses=masses,
                avl={cg: dict(a0, CLd_per_deg=dcl, Cmd_per_deg=None, dcm_ac_per_deg=dcm_ac, sign=+1) for cg in cgs}, bird_dcl=dcl, bird_dcm_ac=dcm_ac, station=None, nose=0.0, span_mm=k,
                tab_note="tail bend (trailing edge up), whole fan tail, section incidence = minus the bend", xnp_mm=xnp, trim_cg=trim_cg, g=g)


# =============================================================================================== the vehicle: aerodynamics + dynamics
class Vehicle:
    def __init__(self, spec, plate: PlateTable, cg_mm, up_deg=0.0, ac_case="np", cm_off=0.0, no_aero=False, eff=1.0, inertia_scale=1.0, drag_off=False):
        self.spec, self.plate = spec, plate
        self.drag_off = drag_off
        m, ball = spec["masses"](cg_mm)
        self.mass = m.mass_g() * 1e-3
        self.cg_mm = cg_mm
        self.cg_calc_mm = m.cg_mm()
        self.inertia = m.inertia_kg_m2(cg_mm) * inertia_scale
        self.ballast_g = ball
        self.S, self.c = spec["S"], spec["mac_m"]
        self.up = up_deg
        self.cm_off = cm_off
        self.no_aero = no_aero
        keys = sorted(spec["avl"])
        ref = min(keys, key=lambda k: abs(k - cg_mm))                                     # AVL derivatives at the nearest of the three CGs, Cmq interpolated linearly
        lo = max([k for k in keys if k <= cg_mm] or [keys[0]]); hi = min([k for k in keys if k >= cg_mm] or [keys[-1]])
        w = 0.0 if hi == lo else (cg_mm - lo) / (hi - lo)
        a = spec["avl"][ref]
        self.cmq = spec["avl"][lo]["Cmq"] * (1 - w) + spec["avl"][hi]["Cmq"] * w
        self.xnp_frac = (a["Xnp"] * 1000 - spec["mac_le_mm"]) / (spec["mac_m"] * 1000)
        self.xcg_frac = (cg_mm - spec["mac_le_mm"]) / (spec["mac_m"] * 1000)
        cla_deg = a["CLa_rad"] / 57.29578
        if "dcm_ac_per_deg" in a:                                                          # bird: bend = TE up, lift change and moment at the neutral point per degree
            dcl_up, dcm_up = a["CLd_per_deg"], a["dcm_ac_per_deg"]
        else:                                                                              # flat wings: AVL control derivatives are per degree TE DOWN; TE up is the negative
            dcl_up = -a["CLd_per_deg"]
            cm_np_down = a["Cmd_per_deg"] + a["CLd_per_deg"] * (a["Xnp"] - a["Xref"]) / a["Cref"]          # Cm_x = Cm_ref + CL (x - x_ref) / c
            dcm_up = -cm_np_down
        self.eff = eff
        dcl_up, dcm_up = eff * dcl_up, eff * dcm_up                                        # effectiveness sensitivity: both the lift shift and the couple scale together
        self.tau = -dcl_up / cla_deg                                                      # alpha shift per degree (lift reduction expressed as an angle)
        self.cm_tab = dcm_up                                                              # nose-up couple per degree, about the AC
        self.dcl_up, self.cla_deg = dcl_up, cla_deg
        self.T = self.xnp_frac - (plate.ac_meas if ac_case == "np" else 0.25)            # table coordinates -> model coordinates
        self.ac_case = ac_case
        self.xg_tab = self.xcg_frac - self.T
        self.alpha_stall = plate.alpha_stall

    def aero(self, u, w, q):
        """-> (F_forward, F_normal, M, alpha_deg, V).  Forces in body axes (N), moment about the CG (N m), nose-up positive."""
        V = math.hypot(u, w)
        if V < 1e-9 or self.no_aero:
            return 0.0, 0.0, 0.0, 0.0, V
        alpha = math.degrees(math.atan2(-w, u))
        rev = abs(alpha) > 90.0
        a_r = math.copysign(180.0 - abs(alpha), alpha) if rev else alpha
        a_eff = a_r - self.tau * self.up
        cn, ca, cm4 = self.plate.coeffs(a_eff)
        if self.drag_off and abs(a_eff) <= self.plate.a_end:                              # explanation test only: table lift with the drag coefficient set to zero
            cl_ = _interp(self.plate.a, self.plate.cl, abs(a_eff))
            cn, ca = math.copysign(cl_ * math.cos(math.radians(a_eff)), a_eff), -cl_ * math.sin(math.radians(abs(a_eff)))
        qs = 0.5 * RHO * V * V * self.S
        if not rev:
            cm = cm4 + cn * (self.xg_tab - 0.25)          # moment about the CG: M_x = M_c/4 + N (x - 0.25 c); x_cp = 0.25 - CM/CN is where it is zero
            ff = -qs * ca
        else:
            xcp = 1.0 - (self.plate.xcp_table(a_eff) + self.T)
            cm = -cn * (xcp - self.xcg_frac)
            ff = qs * ca
        cm += self.cm_tab * self.up + self.cm_off
        m = qs * self.c * cm + 0.25 * RHO * V * self.S * self.c ** 2 * self.cmq * q
        return ff, qs * cn, m, alpha, V

    def deriv(self, s):
        x, z, th, vx, vz, q = s
        ct, st = math.cos(th), math.sin(th)
        u, w = vx * ct + vz * st, -vx * st + vz * ct
        ff, fn, m, _, _ = self.aero(u, w, q)
        ax = (ff * ct - fn * st) / self.mass
        az = (ff * st + fn * ct) / self.mass - G
        return [vx, vz, q, ax, az, m / self.inertia]


def rk4(veh, s, dt):
    k1 = veh.deriv(s)
    k2 = veh.deriv([s[i] + 0.5 * dt * k1[i] for i in range(6)])
    k3 = veh.deriv([s[i] + 0.5 * dt * k2[i] for i in range(6)])
    k4 = veh.deriv([s[i] + dt * k3[i] for i in range(6)])
    return [s[i] + dt / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) for i in range(6)]


def simulate(veh, theta0_deg=-90.0, v0=(0.0, 0.0), dt=DT, t_max=T_MAX, sample=0.02, tol=1.0):
    s = [0.0, 0.0, math.radians(theta0_deg), v0[0], v0[1], 0.0]
    t, n, k_s = 0.0, 0, max(1, int(round(sample / dt)))
    rec = []
    steps = int(round(t_max / dt))
    settle_win = int(2.0 / dt)
    gam_hist = []
    for i in range(steps + 1):
        if i % k_s == 0:
            x, z, th, vx, vz, q = s
            ct, st = math.cos(th), math.sin(th)
            ff, fn, m, alpha, V = veh.aero(vx * ct + vz * st, -vx * st + vz * ct, q)
            gam = math.degrees(math.atan2(vz, vx)) if V > 0.05 else None
            rec.append((i * dt, x, z, math.degrees(th), vx, vz, V, alpha, gam, math.hypot(ff, fn) / (veh.mass * G), q))
            if rec and i % int(0.5 / dt) == 0 and t > 4.0 and _settled(rec, 2.0 / sample, tol):
                break
        if abs(s[0]) > 1e4 or abs(s[1]) > 1e4 or any(map(math.isnan, s)):
            break
        s = rk4(veh, s, dt)
        t += dt
    return rec


def _settled(rec, nlast, tol=1.0):
    w = rec[-int(nlast):]
    if len(w) < int(nlast) or any(r[8] is None for r in w):
        return False
    g = [r[8] for r in w]
    return max(g) - min(g) < tol and max(r[3] for r in w) - min(r[3] for r in w) < tol and max(r[6] for r in w) - min(r[6] for r in w) < 0.03 * tol * max(r[6] for r in w)


def metrics(rec, veh):
    """Headline numbers and the verdict for one trajectory.  HEADLINE = first-swoop depth: the height lost at the first time the flight path reaches zero
    (the first local minimum of height); where the glider never climbs, the height at which the flight path first reaches its final angle."""
    T = [r[0] for r in rec]
    z = [r[2] for r in rec]
    V = [r[6] for r in rec]
    gam = [r[8] for r in rec]
    th = [r[3] for r in rec]
    n_g = [r[9] for r in rec]
    out = {}
    out["t_end"] = T[-1]
    out["peak_speed"] = max(V)
    out["peak_n"] = max(n_g)
    out["height_lost_at_end_of_run_m"] = -min(z)                                        # where the run stopped: NOT a headline number
    out["theta_range_deg"] = max(th) - min(th)
    out["max_climb_deg"] = max([g for g in gam if g is not None] or [0.0])
    out["stall_time_frac"] = sum(1 for r in rec if r[6] > 0.5 and abs(r[7]) > veh.alpha_stall) / max(1, sum(1 for r in rec if r[6] > 0.5))
    out["re_max"] = max(V) * veh.c / NU
    out["re_frac_below_measured"] = sum(1 for r in rec if r[6] * veh.c / NU < RE_MEASURED_MIN) / len(rec)
    t1 = next((r for r in rec if r[8] is not None and r[8] >= GAMMA_LEVEL and r[0] > 0.05 and r[6] > 0.3), None)
    out["h_first_level_m"] = -t1[2] if t1 else None
    tail = [r for r in rec if r[0] >= T[-1] - 2.0 and r[8] is not None]
    settled = len(tail) > 10 and (max(r[8] for r in tail) - min(r[8] for r in tail)) < 2 * SETTLE_BAND
    gss = sum(r[8] for r in tail) / len(tail) if tail else None
    out["gamma_final_deg"] = gss
    out["speed_final"] = sum(r[6] for r in tail) / len(tail) if tail else None
    out["alpha_final_deg"] = sum(r[7] for r in tail) / len(tail) if tail else None
    out["settled"] = bool(settled)
    if settled and gss is not None:
        k = len(rec) - 1
        while k > 0 and rec[k][8] is not None and abs(rec[k][8] - gss) <= SETTLE_BAND:
            k -= 1
        out["h_settled_m"] = -rec[min(k + 1, len(rec) - 1)][2]                          # secondary: includes the glide descent while the oscillation decays
    else:
        out["h_settled_m"] = None
    out["margin_to_first_peak_deg"] = (veh.alpha_stall - out["alpha_final_deg"]) if out["alpha_final_deg"] is not None else None      # positive = below the first lift peak
    rotations = out["theta_range_deg"] / 360.0
    if rotations >= 1.0:
        verdict = "TUMBLES"
    elif settled and gss is not None and gss <= GLIDE_MAX_STEEP:
        verdict = "DIVES"
    elif settled and gss is not None and -out["margin_to_first_peak_deg"] > DEEP_STALL_MARGIN_DEG:
        verdict = "DEEP STALL"
    elif settled and gss is not None:
        verdict = "SETTLES TO A GLIDE"
    elif t1 is not None and all((r[8] is None or r[8] > -80.0) for r in rec if r[0] > t1[0]):
        verdict = "NOT SETTLED (climbs out, still oscillating)"
    else:
        verdict = "DIVES AGAIN / NO CLEAN PULL-OUT"
    out["verdict"] = verdict
    # first-swoop depth
    swoop, kind = None, None
    if verdict not in ("DIVES", "TUMBLES"):
        first = next((r for r in rec if r[0] > 0.05 and r[6] > 0.3 and r[5] >= 0.0), None)          # first time the vertical speed reaches zero = first local minimum of height
        if first is not None:
            swoop, kind = -first[2], "flight path first reaches zero"
        elif settled and gss is not None:
            fr = next((r for r in rec if r[8] is not None and r[0] > 0.05 and r[8] >= gss - 1.0), None)
            if fr is not None:
                swoop, kind = -fr[2], "never climbs: flight path first reaches its final angle"
    out["swoop_depth_m"], out["swoop_kind"] = swoop, kind
    out["glide_ratio_final"] = (1.0 / math.tan(math.radians(-gss))) if (gss is not None and gss < -0.5) else None
    # constant-CL relation: depth ~ 3 (W/S) / (rho g CL)
    cl_final = None
    if out["alpha_final_deg"] is not None and out["speed_final"] and out["speed_final"] > 0.3:
        a = out["alpha_final_deg"]
        cn, ca, _ = veh.plate.coeffs(a - veh.tau * veh.up)
        cl_final = cn * math.cos(math.radians(a)) - ca * math.sin(math.radians(a))
    out["cl_final"] = cl_final
    ws = veh.mass * G / veh.S
    out["wing_loading_N_m2"] = ws
    out["depth_formula_m"] = (3.0 * ws / (RHO * G * cl_final)) if (cl_final and cl_final > 0.02) else None
    out["depth_over_formula"] = (swoop / out["depth_formula_m"]) if (swoop is not None and out["depth_formula_m"]) else None
    out["depth_minus_formula_m"] = (swoop - out["depth_formula_m"]) if (swoop is not None and out["depth_formula_m"]) else None
    return out


def point_mass_const_cl(cl, ws_n_m2, dt=0.0005, t_max=60.0):
    """Reference model for the constant-CL relation: zero drag, constant CL, the body always along the velocity (no pitch dynamics), released from (almost) rest, flight path straight down.
    -> depth at the first time the vertical speed returns to zero.  The relation is depth = 3 (W/S) / (rho g CL), i.e. three trim speed-heights V^2 / 2g with V^2 = 2 (W/S) / (rho CL)."""
    k = 0.5 * RHO * cl / (ws_n_m2 / G)                                                  # lift acceleration per V^2, 1/m
    x, z, vx, vz = 0.0, 0.0, 0.0, -1e-3
    t = 0.0

    def f(st):
        x_, z_, vx_, vz_ = st
        V = math.hypot(vx_, vz_)
        ax = k * V * (-vz_)                                                             # lift is perpendicular to V: direction (-vz, vx)/V, magnitude k V^2
        az = k * V * vx_ - G
        return [vx_, vz_, ax, az]
    st = [x, z, vx, vz]
    for _ in range(int(t_max / dt)):
        k1 = f(st)
        k2 = f([st[i] + 0.5 * dt * k1[i] for i in range(4)])
        k3 = f([st[i] + 0.5 * dt * k2[i] for i in range(4)])
        k4 = f([st[i] + dt * k3[i] for i in range(4)])
        nst = [st[i] + dt / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) for i in range(4)]
        t += dt
        if st[3] < 0 <= nst[3] and t > 0.1:
            return -nst[1]
        st = nst
    return None


# =============================================================================================== cases
def run_case(spec, plate, cg, up, tag, ac_case="np", cm_off=0.0, theta0=-90.0, v0=(0.0, 0.0), dt=DT, eff=1.0, inertia_scale=1.0, drag_off=False):
    veh = Vehicle(spec, plate, cg, up, ac_case, cm_off, eff=eff, inertia_scale=inertia_scale, drag_off=drag_off)
    rec = simulate(veh, theta0, v0, dt)
    m = metrics(rec, veh)
    m.update(spec=spec["name"], cg_mm=cg, up_deg=up, tag=tag, mass_g=veh.mass * 1e3, inertia_g_mm2=veh.inertia * 1e9, static_margin_pct=100 * (veh.xnp_frac - veh.xcg_frac),
             cm_tab_per_deg=veh.cm_tab, tau=veh.tau, plate=plate.wing, ac_case=ac_case, cm_off=cm_off, theta0=theta0, v0=f"{v0[0]:g},{v0[1]:g}", eff=eff, is_bird=spec["name"].startswith("BS"))
    return m, rec


SENS_TAGS = ("disturbed -80 deg", "0.5 m/s sideways", "AC: table c/4 on NP", "zero-lift offset -0.008", "zero-lift offset +0.003", "effectiveness 75%", "effectiveness 50%")
KIT_TAGS = ("kit as built: -80 deg start", "kit as built: 0.5 m/s sideways")
KIT_STARTS = (("-80 deg start", dict(theta0=-80.0)), ("0.5 m/s sideways", dict(v0=(0.5, 0.0))))
KIT_VARS = (("AC c/4", dict(ac_case="c4")), ("offset -0.008", dict(cm_off=CM_OFFSETS[0])), ("offset +0.003", dict(cm_off=CM_OFFSETS[1])))
KIT_VAR_TAGS = tuple(f"kit as built [{v}]: {st}" for v, _ in KIT_VARS for st, _ in KIT_STARTS)
LAG_TAGS = ("lag test: pitch inertia x0.5", "lag test: drag off", "lag test: both")
LAG_CASES = (("F45_span250", 0, 4.0), ("B45_span400", 0, 4.0))                  # (specimen, index in its CG list, input deg)


def flag_cannot_say(cases):
    """'Model cannot say' marks (reasons are kept in the case): within CANNOT_SAY_SM_PCT of the neutral point; verdict flips in any sensitivity run of the same case; every bird-outline
    verdict except the exact zero-input symmetry result (a dive)."""
    base = {(c["spec"], c["cg_mm"], c["up_deg"]): c for c in cases if c["tag"] == "base"}
    flips = {}
    for c in cases:
        if c["tag"] in SENS_TAGS:
            b = base.get((c["spec"], c["cg_mm"], c["up_deg"]))
            if b is not None and b["verdict"] != c["verdict"]:
                flips.setdefault((c["spec"], c["cg_mm"], c["up_deg"]), []).append(c["tag"])
    for c in cases:
        why = []
        if abs(c["static_margin_pct"]) < CANNOT_SAY_SM_PCT:
            why.append(f"within {CANNOT_SAY_SM_PCT:g}% MAC of the neutral point")
        key = (c["spec"], c["cg_mm"], c["up_deg"])
        if c["tag"] == "base" and key in flips:
            why.append("verdict flips in a sensitivity run (" + ", ".join(flips[key]) + ")")
        if c["tag"] in SENS_TAGS and key in base and base[key]["verdict"] != c["verdict"]:
            why.append("verdict differs from the base case")
        if c["tag"] in KIT_TAGS:                                                              # kit as built: verdict must survive the AC placement and both zero-lift offsets
            st_ = c["tag"].split(": ")[-1]
            vs = [x for x in cases if x["spec"] == c["spec"] and x["cg_mm"] == c["cg_mm"] and x["tag"] in KIT_VAR_TAGS and x["tag"].split(": ")[-1] == st_]
            if not vs:
                why.append("band position not sensitivity-tested")
            both = [x for x in cases if x["spec"] == c["spec"] and x["cg_mm"] == c["cg_mm"] and x["tag"] in KIT_VAR_TAGS]
            for x in both:                                                                  # a flip in EITHER disturbed start makes the band position at this CG uncertain
                b0 = next((y for y in cases if y["spec"] == x["spec"] and y["cg_mm"] == x["cg_mm"] and y["tag"] == "kit as built: " + x["tag"].split(": ")[-1]), None)
                if b0 is not None and b0["verdict"] != x["verdict"]:
                    why.append("verdict flips with " + x["tag"].split("[")[1].split("]")[0] + (" (other start)" if x["tag"].split(": ")[-1] != st_ else ""))
        if c["tag"] in KIT_VAR_TAGS:
            b0 = next((x for x in cases if x["spec"] == c["spec"] and x["cg_mm"] == c["cg_mm"] and x["tag"] == "kit as built: " + c["tag"].split(": ")[-1]), None)
            if b0 is not None and b0["verdict"] != c["verdict"]:
                why.append("verdict differs from the base kit run")
        if c["tag"] in KIT_TAGS:                                                              # the two disturbed starts of the kit-as-built case must agree
            other = [x for x in cases if x["spec"] == c["spec"] and x["cg_mm"] == c["cg_mm"] and x["tag"] in KIT_TAGS and x is not c]
            if any(x["verdict"] != c["verdict"] for x in other):
                why.append("the two disturbed starts disagree")
        if c["is_bird"] and not (c["up_deg"] == 0 and c["verdict"] == "DIVES"):
            why.append("bird outline: only the zero-input symmetry result is claimed")
        c["cannot_say"] = bool(why)
        c["cannot_say_why"] = "; ".join(why)


def all_cases(specs, plates, quick=False, log=print):
    cases, trajs = [], {}
    for spec in specs:
        pl = plates[spec["name"]]
        ups = BENDS if spec["name"].startswith("BS") else TABS
        cgs = spec["cgs"]
        mid = cgs[len(cgs) // 2]
        if quick:
            ups, cgs = ups[::2], [cgs[0], mid]
        for cg in cgs:
            for up in ups:
                m, rec = run_case(spec, pl, cg, up, "base")
                cases.append(m)
                trajs[(spec["name"], cg, up, "base")] = rec
        for up in ups:                                                                       # sensitivity at the middle CG
            for tag, kw in (("disturbed -80 deg", dict(theta0=-80.0)), ("0.5 m/s sideways", dict(v0=(0.5, 0.0))), ("AC: table c/4 on NP", dict(ac_case="c4")),
                            ("zero-lift offset -0.008", dict(cm_off=CM_OFFSETS[0])), ("zero-lift offset +0.003", dict(cm_off=CM_OFFSETS[1])),
                            ("effectiveness 75%", dict(eff=EFFECTIVENESS[0])), ("effectiveness 50%", dict(eff=EFFECTIVENESS[1]))):
                if up == 0 and tag.startswith("effectiveness"):
                    continue
                m, rec = run_case(spec, pl, mid, up, tag, **kw)
                cases.append(m)
                trajs[(spec["name"], mid, up, tag)] = rec
        if not quick:                                                                         # effectiveness at two forward CGs as well (where the depth is set by the trim lift coefficient)
            for cg_e in sorted({cgs[0], cgs[2] if len(cgs) > 3 else cgs[1]}):
                if cg_e == mid:
                    continue
                for up in ups:
                    if up == 0:
                        continue
                    for e in EFFECTIVENESS:
                        m, rec = run_case(spec, pl, cg_e, up, f"effectiveness {e * 100:.0f}%", eff=e)
                        cases.append(m)
        if not spec["name"].startswith("BS") and not quick:                                  # the kit as actually built (no tab): every CG of the kit table, disturbed starts
            for cg in spec["cgs_all"]:
                for st, skw in KIT_STARTS:
                    m, rec = run_case(spec, pl, cg, 0.0, f"kit as built: {st}", **skw)
                    cases.append(m)
                    for vn, vkw in KIT_VARS:                                                  # alternative AC placement and both zero-lift offsets, at every kit CG
                        m, rec = run_case(spec, pl, cg, 0.0, f"kit as built [{vn}]: {st}", **dict(skw, **vkw))
                        cases.append(m)
        for sp_, i_, up_ in LAG_CASES:
            if spec["name"] == sp_ and not quick:
                cg_ = spec["cgs"][i_]
                for tag, kw in ((LAG_TAGS[0], dict(inertia_scale=0.5)), (LAG_TAGS[1], dict(drag_off=True)), (LAG_TAGS[2], dict(inertia_scale=0.5, drag_off=True))):
                    m, rec = run_case(spec, pl, cg_, up_, tag, **kw)
                    cases.append(m)
        log(f"  {spec['name']}: {sum(1 for c in cases if c['spec'] == spec['name'])} cases")
    flag_cannot_say(cases)
    return cases, trajs


# =============================================================================================== outputs
def write_csv(path: Path, header, rows):
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def fv(v, n=3):
    if v is None:
        return ""
    return f"{v:.{n}g}" if isinstance(v, float) else str(v)


CASE_COLS = ["spec", "tag", "cg_mm", "up_deg", "eff", "static_margin_pct", "mass_g", "verdict", "cannot_say", "cannot_say_why", "swoop_depth_m", "swoop_kind", "wing_loading_N_m2", "cl_final", "depth_formula_m",
             "depth_over_formula", "depth_minus_formula_m", "h_settled_m", "h_first_level_m", "height_lost_at_end_of_run_m", "alpha_final_deg", "margin_to_first_peak_deg", "glide_ratio_final", "gamma_final_deg", "speed_final", "peak_speed",
             "peak_n", "theta_range_deg", "max_climb_deg", "stall_time_frac", "re_max", "re_frac_below_measured", "plate", "ac_case", "cm_off", "theta0", "v0"]


def svg_traj(title, series, w=560, h=380, xl="distance travelled forward, m", yl="height change, m"):
    """series: list of (label, colour, [(x, z)]); equal-scale axes."""
    xs = [p[0] for _, _, pts in series for p in pts]
    zs = [p[1] for _, _, pts in series for p in pts]
    x0, x1, z0, z1 = min(xs + [0]), max(xs + [1]), min(zs + [-1]), max(zs + [1])
    span = max(x1 - x0, z1 - z0, 1.0)
    L, R, T_, B = 56, 12, 28, 40
    sc = min((w - L - R) / span, (h - T_ - B) / span)
    X = lambda x: L + (x - x0) * sc
    Z = lambda z: T_ + (z1 - z) * sc
    o = [f"<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 {w} {h}' width='{w}' height='{h}' style='max-width:100%;background:#14181d;border:1px solid #333c46;border-radius:6px'>",
         f"<text x='{L}' y='18' font-family='Segoe UI,Arial' font-size='13' fill='#dde3ea'>{html.escape(title)}</text>"]
    step = 1 if span <= 12 else (5 if span <= 60 else (10 if span <= 150 else 50))
    k = math.floor(x0 / step) * step
    while k <= x1:
        o.append(f"<line x1='{X(k):.1f}' y1='{T_}' x2='{X(k):.1f}' y2='{h - B}' stroke='#262e37'/><text x='{X(k):.1f}' y='{h - B + 14}' font-size='10' fill='#9aa7b4' text-anchor='middle' font-family='Segoe UI'>{k:g}</text>")
        k += step
    k = math.floor(z0 / step) * step
    while k <= z1:
        o.append(f"<line x1='{L}' y1='{Z(k):.1f}' x2='{w - R}' y2='{Z(k):.1f}' stroke='#262e37'/><text x='{L - 4}' y='{Z(k) + 3:.1f}' font-size='10' fill='#9aa7b4' text-anchor='end' font-family='Segoe UI'>{k:g}</text>")
        k += step
    for i, (lab, col, pts) in enumerate(series):
        o.append(f"<polyline fill='none' stroke='{col}' stroke-width='1.6' points='{' '.join(f'{X(x):.1f},{Z(z):.1f}' for x, z in pts)}'/>")
        o.append(f"<text x='{w - R - 4}' y='{T_ + 14 + 13 * i}' font-size='11' fill='{col}' text-anchor='end' font-family='Segoe UI'>{html.escape(lab)}</text>")
    o.append(f"<text x='{L + (w - L - R) / 2}' y='{h - 6}' font-size='11' fill='#9aa7b4' text-anchor='middle' font-family='Segoe UI'>{xl}</text>")
    o.append(f"<text x='12' y='{T_ + (h - T_ - B) / 2}' font-size='11' fill='#9aa7b4' text-anchor='middle' font-family='Segoe UI' transform='rotate(-90 12 {T_ + (h - T_ - B) / 2})'>{yl}</text></svg>")
    return "".join(o)


COLS = ["#7fb4ff", "#6cc08b", "#e89a55", "#c39bff", "#e0b83a", "#ff8a80"]
VCOL = {"SETTLES TO A GLIDE": "#1d3b2a", "NOT SETTLED (climbs out, still oscillating)": "#2b3a1d", "DEEP STALL": "#3b2d1d", "DIVES": "#3b1d1d", "TUMBLES": "#3b1d33", "DIVES AGAIN / NO CLEAN PULL-OUT": "#3b251d"}
VSHORT = {"SETTLES TO A GLIDE": "settles to a glide", "NOT SETTLED (climbs out, still oscillating)": "climbs out, not settled in 30 s", "DEEP STALL": "deep stall", "DIVES": "dives",
          "TUMBLES": "tumbles", "DIVES AGAIN / NO CLEAN PULL-OUT": "dives again"}
GREY = "#2a2f36"


def traj_pts(rec):
    return [(r[1], r[2]) for r in rec[::2]]


def build(out: Path, quick=False, log=print):
    out.mkdir(parents=True, exist_ok=True)
    flats = [flat_specimen(n, out, log) for n in SPECS]
    birds = [bird_specimen(sk, out, log) for sk in ("printed", "foam")]
    specs = flats + birds
    rows_all = pb.read_rows()
    runs_all = pb.group_runs(rows_all)
    plates = {s["name"]: PlateTable(s["ar"], s["taper"], rows_all, runs_all) for s in specs}
    log("simulating ...")
    cases, trajs = all_cases(specs, plates, quick, log)
    write_csv(out / "cases.csv", CASE_COLS, [[fv(c.get(k)) for k in CASE_COLS] for c in cases])
    sp_rows = []
    for s in specs:
        pl = plates[s["name"]]
        m_mid, _ = s["masses"](s["cgs"][len(s["cgs"]) // 2])
        a = s["avl"][s["cgs"][len(s["cgs"]) // 2]]
        sp_rows.append([s["name"], s["tab_note"], pl.wing, pl.run, f"{pl.Re:.0f}", f"{pl.a_end:g}", f"{pl.alpha_stall:g}", pl.peak_kind, f"{pl.cl_peak:.3f}", f"{pl.ac_meas:.3f}", f"{s['S'] * 1e4:.1f}", f"{s['mac_m'] * 1e3:.1f}",
                        f"{(a['Xnp'] * 1000 - s['mac_le_mm']) / (s['mac_m'] * 1000):.3f}", f"{m_mid.mass_g():.2f}", f"{m_mid.inertia_kg_m2() * 1e9:.0f}", ",".join(f"{c:g}" for c in s["cgs"])])
    write_csv(out / "specimens.csv", ["specimen", "built-in nose-up input", "plate table wing", "table run", "table Re", "table last alpha deg", "first lift peak alpha deg", "first peak kind", "CL at first peak",
                                      "measured AC (fraction MAC)", "area cm2", "MAC mm", "AVL neutral point (fraction MAC)", "mass at middle CG g", "pitch inertia g mm2", "CG settings mm"], sp_rows)
    av = []
    for s in specs:
        for cg, a in sorted(s["avl"].items()):
            av.append([s["name"], f"{cg:g}", fv(a.get("Xnp")), fv(a.get("CLa_rad")), fv(a.get("Cma_rad")), fv(a.get("Cmq")), fv(a.get("CLd_per_deg")), fv(a.get("Cmd_per_deg")), fv(a.get("dcm_ac_per_deg"))])
    write_csv(out / "avl_derivatives.csv", ["specimen", "CG mm (AVL reference)", "Xnp m", "CLa per rad", "Cma per rad", "Cmq per rad", "CL per deg of control (tab: per deg TE DOWN; bird: per deg bend TE UP)", "Cm_ref per deg (tab)", "Cm at NP per deg bend (bird)"], av)
    kb = kit_bands(specs, cases)
    write_csv(out / "kit_band.csv", ["specimen", "variant", "settling CG settings mm (both disturbed starts settle to a glide)", "AVL neutral point mm", "offset aft of NP, min mm", "offset aft of NP, max mm", "offset min % MAC", "offset max % MAC"],
              [[b["spec"], b["variant"], " ".join(f"{c:g}" for c in b["cgs"]), f"{b['xnp_mm']:.2f}", fv(b["off_mm"][0] if b["off_mm"] else None, 3), fv(b["off_mm"][1] if b["off_mm"] else None, 3),
                fv(b["off_pct"][0] if b["off_pct"] else None, 3), fv(b["off_pct"][1] if b["off_pct"] else None, 3)] for b in kb])
    page(out, specs, plates, cases, trajs, quick)
    return dict(specs=specs, plates=plates, cases=cases, trajs=trajs)


def med(v):
    v = sorted(v)
    return v[len(v) // 2] if v else None


def cell(c):
    """Table cell text and colour for one case.  First-swoop depth leads."""
    if c["cannot_say"] and not c["is_bird"]:
        return "model cannot say<br><small>" + html.escape(c["cannot_say_why"]) + "</small>", GREY
    if c["is_bird"]:
        if c["up_deg"] == 0 and c["verdict"] == "DIVES":
            return "dives (symmetry result)", VCOL["DIVES"]
        d = f"first swoop {c['swoop_depth_m']:.0f} m" if c["swoop_depth_m"] is not None else "no swoop depth"
        return f"{d}<br><small>if the AR 5 table carried over; verdict: model cannot say</small>", GREY
    if c["verdict"] in ("DIVES", "TUMBLES"):
        return VSHORT[c["verdict"]], VCOL[c["verdict"]]
    d = f"<b>first swoop {c['swoop_depth_m']:.1f} m</b>" if c["swoop_depth_m"] is not None else "<b>no swoop depth</b>"
    if c["verdict"].startswith("NOT SETTLED") or c["verdict"].startswith("DIVES AGAIN"):
        return f"{d}<br><small>{VSHORT[c['verdict']]}</small>", VCOL[c["verdict"]]
    gr = f"L/D {c['glide_ratio_final']:.1f}" if c["glide_ratio_final"] else ""
    mg = c["margin_to_first_peak_deg"]
    sec = f"settled glide: alpha {c['alpha_final_deg']:.1f} deg, {gr}, {abs(mg):.1f} deg {'below' if mg >= 0 else 'ABOVE'} the first peak" + (f"<br>to settled {c['h_settled_m']:.0f} m (incl. glide descent)" if c["h_settled_m"] is not None else "")
    return f"{d}<br><small>{VSHORT[c['verdict']]}; {sec}</small>", VCOL[c["verdict"]]


def kit_bands(specs, cases):
    """For each kit wing and each variant (base, AC c/4, offset -0.008, offset +0.003): the CG settings where BOTH disturbed starts settle to a glide, as offsets aft of the AVL neutral point.
    -> list of dict(spec, variant, cgs, off_mm (min, max), off_pct (min, max))."""
    out = []
    for s in specs:
        if s["name"].startswith("BS"):
            continue
        a0 = s["avl"][s["cgs"][0]]
        xnp = a0["Xnp"] * 1000.0
        mac = s["mac_m"] * 1000.0
        for vn in ["base"] + [v for v, _ in KIT_VARS]:
            good = []
            for cg in s["cgs_all"]:
                vs = []
                for st, _ in KIT_STARTS:
                    tag = f"kit as built: {st}" if vn == "base" else f"kit as built [{vn}]: {st}"
                    c = next((x for x in cases if x["spec"] == s["name"] and x["cg_mm"] == cg and x["tag"] == tag), None)
                    vs.append(c is not None and c["verdict"] == "SETTLES TO A GLIDE")
                if all(vs):
                    good.append(cg)
            offs = [cg - xnp for cg in good]
            out.append(dict(spec=s["name"], variant=vn, cgs=good, xnp_mm=xnp, mac_mm=mac, off_mm=(min(offs), max(offs)) if offs else None,
                            off_pct=(100 * min(offs) / mac, 100 * max(offs) / mac) if offs else None))
    return out


def robust_points(specs, base, cases):
    ws = [c["wing_loading_N_m2"] for c in base]
    lo, hi = min(ws), max(ws)
    f = lambda w, cl: 3.0 * w / (RHO * G * cl)
    kit = [c for c in cases if c["tag"] in KIT_TAGS]
    good = [c for c in kit if c["static_margin_pct"] >= CANNOT_SAY_SM_PCT]
    dive = [c for c in good if c["verdict"] == "DIVES"]
    zero_base = [c for c in base if c["up_deg"] == 0 and not c["is_bird"]]
    ok = [c for c in base if c["swoop_depth_m"] is not None and c["depth_over_formula"] is not None and not c["cannot_say"] and not c["is_bird"]]
    ratios = [c["depth_over_formula"] for c in ok]
    nflag = sum(1 for c in cases if c["cannot_say"])
    items = [
        f"<li><b>(i) Symmetry.</b> A symmetric plate released exactly nose-down has zero angle of attack and no moment to start pitching: all {len(zero_base)} exactly-nose-down, no-input base cases (three kit wings, every CG tried) dive straight down. "
        f"With a small disturbance (start at -80 deg, or 0.5 m/s sideways), the kit wings as built (no tab) at the {len(good)} CG settings that sit at least {CANNOT_SAY_SM_PCT:g}% MAC ahead of the neutral point: {len(dive)} of {len(good)} still dive (the rest: see the table below). This is the symmetry of the plate plus a CG ahead of the low-angle lift point; it does not depend on the measured tables.</li>",
        f"<li><b>(ii) With any nose-up input, the first-swoop depth is about 3 (W/S) / (rho g CL_trim)</b> (an exact relation for a constant lift coefficient and no drag, checked in the tests; the simulation gives a median of {med(ratios):.2f} times it, range {min(ratios):.2f} to {max(ratios):.2f}, over {len(ratios)} cases that are not marked 'cannot say'). At the wing loadings here ({lo:.1f} to {hi:.1f} N/m&sup2;, CALCULATED) that is "
        f"{f(lo, 0.5):.0f} to {f(hi, 0.3):.0f} m at CL 0.3 to 0.5, and {f(lo, 0.15):.0f} to {f(hi, 0.1):.0f} m at CL 0.10 to 0.15. Equivalently, the depth is 1.5 times the constant-CL radius 2 (W/S) / (rho g CL); the earlier 7 m and 3 m figures were radii, not depths.</li>",
        f"<li><b>(iii) A 10 m drop</b> therefore cannot tell &quot;never pulls out&quot; from &quot;needs 30 m&quot; at the forward CG settings, where CL_trim is low and the depth is {f(lo, 0.15):.0f} to {f(hi, 0.1):.0f} m.</li>",
        f"<li><b>(iv) Near the neutral point the model cannot say.</b> {nflag} of the {len(cases)} cases are marked 'model cannot say' (within {CANNOT_SAY_SM_PCT:g}% MAC of the neutral point, verdict flips in a sensitivity run, or a bird-outline verdict other than the symmetry result).</li>"]
    return items


def page(out: Path, specs, plates, cases, trajs, quick):
    h = html.escape
    base = [c for c in cases if c["tag"] == "base"]
    P = [f"<!doctype html><html lang='en'><head><meta charset='utf-8'><title>Pull-out study: released nose-down at rest</title><style>{ft.CSS}\n.scroll{{overflow-x:auto}}table.t td,table.t th{{padding:4px 7px;font-size:13px;vertical-align:top}}"
         f".warn{{background:#2c2418;border:2px solid #e89a55;padding:10px 16px}}figure{{margin:12px 0}}.plots{{display:grid;grid-template-columns:repeat(auto-fit,minmax(420px,1fr));gap:12px}}td.c{{text-align:center}}small{{color:#9aa7b4}}</style></head><body><main>"]
    P.append("<h1>Does it pull out? Released nose-down at rest</h1>")
    P.append("<p class='note'>An open, source-tagged simulation. Tags: <b>MEASURED</b> (Ananda 2012 wind-tunnel plates), <b>MODEL</b> (AVL, outline only), <b>CALCULATED</b> (masses and inertia from the kit breakdowns), <b>ASSUMED</b> (stated guesses), <b>JUDGEMENT</b> (our definitions). "
             "Nothing here is a flight result. The headline number is the <b>first-swoop depth</b>: the height lost at the first time the flight path reaches zero (the first local minimum of height); where the glider never climbs, the height at which the flight path first reaches its final angle." + (" <b>QUICK MODE: reduced case list.</b>" if quick else "") + "</p>")
    P.append("<section><h2>Verdict: the points that hold up</h2><ul>" + "".join(robust_points(specs, base, cases)) + "</ul>"
             "<p><b>Drop height, and whether to add a nose-up input, are Kai's decisions.</b> Nothing on this page recommends a change to the kit tables, the stop rule, the drop height or the hardware.</p></section>")
    # ---- cannot say summary
    cs = [c for c in cases if c["cannot_say"]]
    P.append(f"<section><h2>Where the model cannot say</h2><p>{len(cs)} of {len(cases)} cases are marked. Reasons (a case can have several): " +
             "; ".join(f"{k}: {sum(1 for c in cs if k in c['cannot_say_why'])}" for k in (f"within {CANNOT_SAY_SM_PCT:g}% MAC of the neutral point", "verdict flips in a sensitivity run", "verdict differs from the base case", "verdict flips with", "verdict differs from the base kit run", "band position not sensitivity-tested", "the two disturbed starts disagree", "bird outline")) +
             ". Their verdicts are shown greyed and are not part of the points above. Bird-outline first-swoop depths are shown only as &quot;if the AR 5 table carried over&quot; (the bird is AR 9.5; the tested plates stop at AR 5).</p></section>")
    P.append("<section><h2>Definitions</h2><ul>"
             "<li><b>First-swoop depth (headline)</b>: height below the release point at the first time the vertical speed returns to zero. If the glider never climbs, the height at which the flight path first reaches its final angle. Not defined for dives and tumbles.</li>"
             f"<li><b>Settles to a glide</b>: in the last 2 s the flight path stays within a 10 deg band and is shallower than 45 deg down. The result reports the final angle of attack, glide ratio and the margin to the table's first lift peak (positive = below the peak). <b>Deep stall</b> is reserved for a settled glide more than {DEEP_STALL_MARGIN_DEG:g} deg ABOVE the first lift peak (JUDGEMENT; an angle of attack of 29.8 deg against an 11 deg peak counts, 11.9 deg against 11 deg does not).</li>"
             "<li><b>Dives</b>: settles steeper than 45 deg down. <b>Tumbles</b>: pitch angle turns through 360 deg or more. <b>Climbs out, not settled</b>: after the flight path first gets shallower than 20 deg down it never again dives steeper than 80 deg, but has not settled in 30 s. <b>Dives again</b>: anything else.</li>"
             "<li><b>Height to settled</b> is a secondary column: it includes the glide descent while the oscillation decays, so it keeps growing with the glide ratio and is not the pull-out depth.</li>"
             "<li>Load factor = aerodynamic force / weight. The constant-CL formula column is 3 (W/S) / (rho g CL_final) with CL_final from the table at the final angle of attack; its ratio to the simulated depth is also listed.</li></ul></section>")
    P.append("<section><h2>What is measured, modelled, assumed</h2><div class='scroll'><table class='t'><tr><th>piece</th><th>source</th><th>detail</th></tr>"
             "<tr><td>lift, drag, moment vs angle</td><td>MEASURED on rectangular plates, carry-over ASSUMED</td><td>Ananda 2012 tables through plate_balance.py; nearest tested aspect ratio and taper, lowest tested Reynolds number (60 000); made exactly symmetric (odd / even parts). Our outlines are tapered, swept, finned: not tested. The first lift peak uses the plate_balance rule.</td></tr>"
             f"<tr><td>beyond the table</td><td>ASSUMED</td><td>The table ends near 29 deg. Beyond it the normal force fades (over {BLEND_DEG:g} deg) into CN = {CN90:g} sin(alpha), the axial force follows cos(alpha), and the centre of pressure moves linearly to mid-chord at 90 deg. Flow from behind is the mirror of forward flow. Negative angles mirror positive ones.</td></tr>"
             "<tr><td>aerodynamic centre, damping, tab / bend effect</td><td>MODEL (AVL)</td><td>Neutral point with the tip fins (flat wings) or of the v2 outline (bird); Cmq at the CG; tab: AVL control surface (new decks in <code>avl/</code>); bird: tail incidence = minus the bend. The table's centre-of-pressure curve is translated so its aerodynamic centre sits on the AVL neutral point (main case); the alternative case puts the table's quarter chord there.</td></tr>"
             f"<tr><td>tab definition</td><td>JUDGEMENT</td><td>Aft {100 * TAB_CHORD_FRAC:g}% of the chord over the inner {100 * TAB_SPAN_FRAC:g}% of each semi-span, deflected trailing-edge up by 0, 2, 4, 6, 8 deg. Moment and lift change per degree come from AVL (<a href='avl_derivatives.csv'>avl_derivatives.csv</a>); AVL is inviscid and probably optimistic for a thin plate at Re 15 000 to 30 000, so 75% and 50% effectiveness cases are included. The tab acts as a pure moment about the aerodynamic centre plus a shift of the effective angle of attack: <b>the angle-shift treatment moves the stall angle by about 0.33 deg per degree of tab as a side effect of the method, not a measurement.</b></td></tr>"
             "<tr><td>masses, CG, pitch inertia</td><td>CALCULATED</td><td>Kit scale and ballast tables (flat wings, ballast at the nose hole nearest 75% of the nose), bird_outline_v2 sheet and rail masses; inertia from the plate polygon, rail bar and point masses. Kai's single measured number (&quot;about 15 g per full printed wing&quot;, which wing unconfirmed) is not used.</td></tr>"
             "<tr><td>zero-lift moment offset</td><td>MEASURED range, sensitivity only</td><td>-0.008 to +0.003 (plate_balance step 2) added as a constant moment.</td></tr>"
             "<tr><td>air</td><td>ASSUMED</td><td>density 1.225 kg/m&sup3;, viscosity 1.5e-5 m&sup2;/s (kit assumptions).</td></tr></table></div></section>")
    P.append("<section><h2>Limits of the model</h2><ul>"
             "<li>Still air, no quad downwash, no string or release transient, 3 degrees of freedom only. A real release is never exactly nose-down.</li>"
             "<li>Quasi-static aerodynamics: no unsteady lift, no added mass.</li>"
             f"<li>Reynolds number: the tables start at Re {min(p.Re for p in plates.values()):,.0f}. This glider starts at zero and spends much of the swoop below that range. Everything below is <b>extrapolation</b>.</li>"
             "<li>Pitch damping Cmq is AVL's linear small-rate value (bird: at the file's reference point, not at each CG).</li>"
             "<li>Height is measured from the release point; the ground is not modelled.</li></ul></section>")
    # ---- grids
    P.append("<section><h2>Results: cases that start exactly nose-down</h2><p>Each cell: first-swoop depth (bold), then the settled-glide numbers. SM = AVL neutral point minus CG, % of MAC. Grey = model cannot say.</p>")
    for s in specs:
        cs_ = [c for c in base if c["spec"] == s["name"]]
        ups = sorted({c["up_deg"] for c in cs_})
        cgs = sorted({c["cg_mm"] for c in cs_})
        head = "<tr><th>built-in input, deg</th>" + "".join(f"<th>CG {cg:g} mm<br>SM {next(c['static_margin_pct'] for c in cs_ if c['cg_mm'] == cg):.1f}%</th>" for cg in cgs) + "</tr>"
        body = ""
        for up in ups:
            body += f"<tr><td class='n'>{up:g}</td>"
            for cg in cgs:
                c = next(c for c in cs_ if c["cg_mm"] == cg and c["up_deg"] == up)
                txt, col = cell(c)
                body += f"<td class='c' style='background:{col}'>{txt}</td>"
            body += "</tr>"
        pl = plates[s["name"]]
        P.append(f"<h3>{s['name']}: {h(s['tab_note'])}</h3><p>Plate table: {pl.wing} Re {pl.Re:,.0f} (carry-over ASSUMED), first lift peak at {pl.alpha_stall:g} deg ({pl.peak_kind}), CL {pl.cl_peak:.2f}. Mass {s['masses'](s['cgs'][0])[0].mass_g():.1f} to {s['masses'](s['cgs'][-1])[0].mass_g():.1f} g across the CG range (CALCULATED)."
                 + (" <b>Bird outline: depths only, if the AR 5 table carried over.</b>" if s["name"].startswith("BS") else "") + f"</p><div class='scroll'><table class='t'>{head}{body}</table></div>")
    P.append("</section>")
    # ---- kit as built: described by behaviour and by offset from the neutral point
    bands = kit_bands(specs, cases)
    P.append("<section><h2>The kit as built: no tab, disturbed starts</h2><p>This is the regime the flat test is about: no built-in nose-up input, CG ahead of or just behind the low-angle lift point. The exactly nose-down start always dives (symmetry). "
             "With a small disturbance (start at -80 deg, or 0.5 m/s sideways), the model gives three behaviours, depending on where the CG is relative to the AVL neutral point:</p><ul>"
             "<li><b>CG well ahead of the neutral point:</b> the disturbance dies out and it keeps diving.</li>"
             "<li><b>A band of CG aft of the neutral point:</b> it swings out of the dive and settles into a slow near-stall glide (numbers below).</li>"
             "<li><b>CG further aft, and the stretch around the neutral point:</b> it wanders, dives again or tumbles; the model cannot say.</li></ul>")
    rows = ""
    for s in specs:
        if s["name"].startswith("BS"):
            continue
        bb = [b for b in bands if b["spec"] == s["name"]]
        base_b = next(b for b in bb if b["variant"] == "base")
        gl = [c for c in cases if c["spec"] == s["name"] and c["tag"] in KIT_TAGS and c["cg_mm"] in base_b["cgs"] and c["verdict"] == "SETTLES TO A GLIDE"]
        allv = [b for b in bb if b["off_mm"] is not None]
        def rng(b):
            return "none" if b["off_mm"] is None else f"{b['off_mm'][0]:+.0f} to {b['off_mm'][1]:+.0f} mm ({b['off_pct'][0]:+.1f} to {b['off_pct'][1]:+.1f}% MAC)"
        lo_all = min(b["off_mm"][0] for b in allv) if allv else None
        hi_all = max(b["off_mm"][1] for b in allv) if allv else None
        beh = (f"settles at {med([c['speed_final'] for c in gl]):.1f} m/s, angle of attack {med([c['alpha_final_deg'] for c in gl]):.0f} deg (first lift peak {plates[s['name']].alpha_stall:.0f} deg), glide ratio {med([c['glide_ratio_final'] for c in gl]):.1f}, first swoop {med([c['swoop_depth_m'] for c in gl if c['swoop_depth_m'] is not None]):.1f} m ({len(gl)} runs)" if gl else "no settling band found")
        shifts = [max(abs(b["off_mm"][0] - base_b["off_mm"][0]), abs(b["off_mm"][1] - base_b["off_mm"][1])) for b in bb if b["off_mm"] is not None and base_b["off_mm"] is not None and b is not base_b]
        shift_txt = (f"{max(shifts):.0f} (and the band vanishes for the -0.008 offset)" if shifts and any(b["off_mm"] is None for b in bb if b is not base_b) else (f"{max(shifts):.0f}" if shifts else "no base band; variants put one anywhere in the range shown"))
        rows += (f"<tr><td>{s['name']}</td><td>{beh}</td><td>{rng(base_b)}</td>" + "".join(f"<td>{rng(next(b for b in bb if b['variant'] == v))}</td>" for v, _ in KIT_VARS)
                 + f"<td class='n'>{('%.0f to %.0f' % (lo_all, hi_all)) if allv else '-'}</td><td>{shift_txt}</td></tr>")
    P.append("<div class='scroll'><table class='t'><tr><th>wing</th><th>behaviour in the band</th><th>band, base (offset aft of the AVL neutral point)</th><th>alternative AC placement</th><th>zero-lift offset -0.008</th><th>zero-lift offset +0.003</th><th>band over all four, mm</th><th>largest edge shift, mm</th></tr>" + rows + "</table></div>")
    P.append("<p><b>How far to trust the band.</b> Its position comes from the centre-of-pressure travel of an unswept, 4.3% thick plate at Re 60 000, applied to a swept, finned, ribbed wing at lower Reynolds number. The edge shifts above (1 mm steps of the kit table limit the resolution) are only the sensitivity to the alternative aerodynamic-centre placement and the measured zero-lift offset range; they do not include the carry-over itself, which is not known. "
             "The kit-as-built cases carry 'model cannot say' wherever any of those variants changes the verdict or the two starts disagree. Per-CG results: <a href='cases.csv'>cases.csv</a> and <a href='kit_band.csv'>kit_band.csv</a>.</p></section>")
    # ---- middle CG numbers
    P.append("<section><h2>Numbers for the middle CG setting</h2><div class='scroll'><table class='t'><tr><th>specimen</th><th>input deg</th><th>verdict</th><th><b>first-swoop depth, m</b></th><th>3(W/S)/(rho g CL_final), m</th><th>ratio</th><th>sim minus formula, m</th><th>to settled glide, m (secondary)</th><th>final alpha, deg</th><th>glide ratio</th><th>margin to first peak, deg</th><th>peak speed m/s</th><th>peak load factor</th><th>Re max</th></tr>")
    for s in specs:
        mid = s["cgs"][len(s["cgs"]) // 2]
        for c in [c for c in base if c["spec"] == s["name"] and c["cg_mm"] == mid]:
            say = "model cannot say" if c["cannot_say"] and not c["is_bird"] else VSHORT[c["verdict"]]
            P.append(f"<tr style='opacity:{0.6 if c['cannot_say'] else 1}'><td>{s['name']} CG {mid:g}</td><td class='n'>{c['up_deg']:g}</td><td>{say}</td><td class='n'><b>{fv(c['swoop_depth_m'], 3)}</b></td><td class='n'>{fv(c['depth_formula_m'], 3)}</td><td class='n'>{fv(c['depth_over_formula'], 3)}</td><td class='n'>{fv(c['depth_minus_formula_m'], 3)}</td><td class='n'>{fv(c['h_settled_m'], 3)}</td><td class='n'>{fv(c['alpha_final_deg'], 3)}</td><td class='n'>{fv(c['glide_ratio_final'], 3)}</td><td class='n'>{fv(c['margin_to_first_peak_deg'], 3)}</td><td class='n'>{c['peak_speed']:.1f}</td><td class='n'>{c['peak_n']:.2f}</td><td class='n'>{c['re_max']:,.0f}</td></tr>")
    P.append("</table></div><p>Full list: <a href='cases.csv'>cases.csv</a>. Rows shown dimmed are marked 'model cannot say'. Bird rows: depths if the AR 5 table carried over.</p></section>")
    # ---- rotation-lag test
    lag = []
    for sp_, i_, up_ in LAG_CASES:
        sp = next(x for x in specs if x["name"] == sp_)
        cg_ = sp["cgs"][i_]
        b0 = next(c for c in base if c["spec"] == sp_ and c["cg_mm"] == cg_ and c["up_deg"] == up_)
        runs_ = [next((c for c in cases if c["spec"] == sp_ and c["tag"] == t), None) for t in LAG_TAGS]
        lag.append((sp_, cg_, b0, runs_))
    if lag and all(r is not None for *_, rs in lag for r in rs):
        def cut(b0, c):
            if c["depth_minus_formula_m"] is None or not b0["depth_minus_formula_m"]:
                return None
            return 1 - c["depth_minus_formula_m"] / b0["depth_minus_formula_m"]
        red = [cut(b0, rs[0]) for _, _, b0, rs in lag]
        dred = [cut(b0, rs[1]) for _, _, b0, rs in lag]
        supported = all(r is not None and r >= 0.30 for r in red)
        P.append("<section><h2>Why is the simulated swoop deeper than the constant-CL formula?</h2><p>One suspected reason is rotation lag: the glider needs time to swing its nose up while it is already falling and gaining speed, which the constant-CL formula (the body is always along the flight path) does not include. Test: the same forward-CG cases with the pitch inertia halved, with the drag off, and with both.</p>"
                 "<div class='scroll'><table class='t'><tr><th>case</th><th>run</th><th>first-swoop depth, m</th><th>formula, m</th><th>sim minus formula, m</th><th>ratio</th><th>verdict</th></tr>")
        for sp_, cg_, b0, rs in lag:
            for lab, c in (("as simulated", b0), ("pitch inertia x 0.5", rs[0]), ("drag off", rs[1]), ("both", rs[2])):
                P.append(f"<tr><td>{sp_} CG {cg_:g}, 4 deg tab</td><td>{lab}</td><td class='n'>{fv(c['swoop_depth_m'], 3)}</td><td class='n'>{fv(c['depth_formula_m'], 3)}</td><td class='n'>{fv(c['depth_minus_formula_m'], 3)}</td><td class='n'>{fv(c['depth_over_formula'], 3)}</td><td>{VSHORT[c['verdict']]}</td></tr>")
        fmt_c = lambda v: "no defined result" if v is None else f"{100 * v:+.0f}%"
        P.append("</table></div><p>Halving the pitch inertia changes the difference (sim minus formula) by " + ", ".join(f"{fmt_c(r)} ({lag[k][0]})" for k, r in enumerate(red))
                 + ". " + ("That is a cut of at least 30% in both cases: the rotation-lag explanation is supported." if supported else "That is not a cut of at least 30% in both cases, so the rotation-lag explanation is <b>not supported and stays unverified</b> by these runs.")
                 + " Switching the drag off cuts the difference by " + ", ".join(f"{fmt_c(r)} ({lag[k][0]})" for k, r in enumerate(dred))
                 + "; where a run has no defined result the drag-off case no longer settles to the same kind of glide (its verdict is shown), so it is inconclusive there. "
                 + "On these runs drag accounts for most of the difference for the flat wing and is inconclusive for the foam wing; the cause for the foam wing is not established.</p></section>")
    # ---- sensitivities
    tags = ["disturbed -80 deg", "0.5 m/s sideways", "AC: table c/4 on NP", "zero-lift offset -0.008", "zero-lift offset +0.003", "effectiveness 75%", "effectiveness 50%"]
    P.append("<section><h2>Sensitivity cases (middle CG)</h2><div class='scroll'><table class='t'><tr><th>specimen</th><th>input deg</th><th>base</th>" + "".join(f"<th>{t}</th>" for t in tags) + "</tr>")
    for s in specs:
        mid = s["cgs"][len(s["cgs"]) // 2]
        for up in sorted({c["up_deg"] for c in base if c["spec"] == s["name"]}):
            b0 = next(c for c in base if c["spec"] == s["name"] and c["cg_mm"] == mid and c["up_deg"] == up)
            d0 = "-" if b0["swoop_depth_m"] is None else f"{b0['swoop_depth_m']:.1f} m"
            row = f"<tr><td>{s['name']}</td><td class='n'>{up:g}</td><td class='c' style='background:{VCOL[b0['verdict']]}'>{d0}<br><small>{VSHORT[b0['verdict']]}</small></td>"
            for t in tags:
                c = next((c for c in cases if c["spec"] == s["name"] and c["tag"] == t and c["up_deg"] == up and c["cg_mm"] == mid), None)
                if c is None:
                    row += "<td></td>"
                    continue
                d = "-" if c["swoop_depth_m"] is None else f"{c['swoop_depth_m']:.1f} m"
                flip = " <b>(verdict flips)</b>" if c["verdict"] != b0["verdict"] else ""
                row += f"<td class='c' style='background:{VCOL[c['verdict']]}'>{d}<br><small>{VSHORT[c['verdict']]}{flip}</small></td>"
            P.append(row + "</tr>")
    P.append("</table></div>")
    eff = {}
    for e in EFFECTIVENESS:
        r = []
        for c in cases:
            if c["tag"] == f"effectiveness {e * 100:.0f}%" and c["swoop_depth_m"] is not None:
                b0 = next((x for x in base if x["spec"] == c["spec"] and x["cg_mm"] == c["cg_mm"] and x["up_deg"] == c["up_deg"]), None)
                if b0 and b0["swoop_depth_m"] and not c["is_bird"] and b0["verdict"] == c["verdict"] and c["verdict"] == "SETTLES TO A GLIDE":
                    r.append(c["swoop_depth_m"] / b0["swoop_depth_m"])
        eff[e] = r
    P.append("<p><b>Tab / bend effectiveness.</b> The AVL values are inviscid. Scaling the tab (or bend) effect to 75% and 50% of the AVL value changes the first-swoop depth, for the flat-wing cases that settle to a glide in both runs, by a median factor of "
             + ", ".join(f"{med(v):.2f} (range {min(v):.2f} to {max(v):.2f}, {len(v)} cases) at {e * 100:.0f}%" for e, v in eff.items() if v)
             + ". Depth scales roughly with 1 / CL_trim, i.e. directly with how much nose-up moment the input really supplies, which is not known for a thin plate at Re 15 000 to 30 000.</p></section>")
    P.append("<section><h2>Side-view trajectories (middle CG, base cases)</h2><div class='plots'>")
    for s in specs:
        mid = s["cgs"][len(s["cgs"]) // 2]
        ups = sorted({c["up_deg"] for c in base if c["spec"] == s["name"]})
        ser = [(f"input {up:g} deg", COLS[i % len(COLS)], traj_pts(trajs[(s["name"], mid, up, "base")])) for i, up in enumerate(ups)]
        title = f"{s['name']}, CG {mid:g} mm, release at origin" + (" (if the AR 5 table carried over)" if s["name"].startswith("BS") else "")
        P.append(f"<figure>{svg_traj(title, ser)}</figure>")
        (out / f"trajectories_{s['name']}.svg").write_text(svg_traj(title, ser), encoding="utf-8")
    P.append("</div><p>Trajectory plots near the neutral point are shown for completeness; the model cannot say which verdict is right there.</p></section>")
    P.append("<section><h2>Files</h2><ul><li><a href='cases.csv'>cases.csv</a>, <a href='specimens.csv'>specimens.csv</a>, <a href='avl_derivatives.csv'>avl_derivatives.csv</a>; AVL decks and logs in <code>avl/</code>; trajectory SVGs <code>trajectories_*.svg</code>; "
             "script <code>rigid_glider/pullout_sim.py</code>; tests <code>rigid_glider/test_pullout_sim.py</code>. In cases.csv the column <code>height_lost_at_end_of_run_m</code> is just where the run stopped; it is not a result.</li></ul></section></main></body></html>")
    (out / "index.html").write_text("\n".join(P), encoding="utf-8")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=OUT_DEFAULT)
    ap.add_argument("--quick", action="store_true")
    a = ap.parse_args()
    build(a.out, a.quick)
    print("wrote", a.out)


if __name__ == "__main__":
    main()
