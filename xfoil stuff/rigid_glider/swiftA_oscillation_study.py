"""Swift A foamboard 400 mm: what are the pitch oscillations after the nose-down drop?  MODEL study, nothing here is a flight result.

Reuses (imports, no edits):
  pullout_sim.py            Vehicle / PlateTable / MassModel / avl_session / rk4  (3-DOF longitudinal sim, measured Ananda plate tables, AVL NP / Cmq / bend moment)
  bird_outline_v4_swift.py  Swift A geometry (build_geometry4, GLIDE outline)
  bird_outline_v2.py        avl_deck2, size_numbers (mass, rail, ballast)
  flight_day_page.py        swift_numbers(): the foam 400 mass (9.77 g), ballast (1.21 g at the -12.5 mm hole) and 10 %-margin CG (36.96 mm from root LE) used on the flight-day pages
Frames: all CG / NP numbers in this file are mm from the ROOT leading edge unless the name says _nose (mm behind the nose tip; nose tip is 36.0 mm ahead of the root LE).

    py -B rigid_glider/swiftA_oscillation_study.py [--out DIR] [--coarse] [--jobs N]
"""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import argparse
import csv
import html
import json
import math
import os
import time
from pathlib import Path

import numpy as np

import bird_outline_v2 as v2
import bird_outline_v4_swift as s4
import flight_day_page as fd
import plate_balance as pb
import pullout_sim as ps

ROOT = ps.ROOT
OUT_DEFAULT = ROOT / "results/runs/rigid_glider/swiftA_oscillation"
SPAN_MM = 400.0
NOSE_AHEAD_MM = 36.0             # nose tip ahead of the root LE: g['nose_x'] = -0.09 x 400
G = ps.G
T_MAX = 30.0
DT = 0.002

# ---- analysis thresholds (stated in the README)
HYST_DEG = 0.5                   # a flight-path extremum counts only if it is at least this far from the previous one
SETTLE_WIN_S = 5.0
SETTLE_RANGE_DEG = 5.0           # settled: flight path stays within this peak-to-peak over the last 5 s
SMOOTH_FRAC = 0.25               # smooth: amplitude of cycle 3 below this fraction of cycle 1 (i.e. within 2 cycles)
DIVE_GAMMA = -45.0
DIVE_AGAIN = -80.0
SHRINK_R, GROW_R = 0.8, 1.1      # decay ratio per cycle: below 0.8 shrinks, above 1.1 grows, otherwise persists
PORPOISE_CYCLES = 2
CLASSES = ("dive", "porpoise", "persistent phugoid", "smooth")

CG_NOSE = list(range(69, 77))    # mm behind nose tip
BENDS = list(range(3, 11))       # deg
AS_FLOWN = (73.0, 6.0)

VARIANTS = (
    ("base", dict()),
    ("effectiveness 75%", dict(eff=0.75)),
    ("effectiveness 50%", dict(eff=0.5)),
    ("Cm offset -0.008", dict(cm_off=ps.CM_OFFSETS[0])),
    ("Cm offset +0.003", dict(cm_off=ps.CM_OFFSETS[1])),
    ("AC: table c/4 on NP", dict(ac_case="c4")),
    ("pitch inertia x0.5", dict(inertia_scale=0.5)),
    ("pitch inertia x2", dict(inertia_scale=2.0)),
)

PRESTATED = """- (a) CG 73.0 mm from nose tip with a 6 deg tail bend pulls out of the nose-down drop and goes on to a glide: after the first level-off the flight path never returns steeper than -80 deg, the mean flight path over the last 5 s of the 30 s run is shallower than -45 deg, and the glider does not tumble. (A glide with a large oscillation still passes (a); the size of the oscillation is judged in (c).)
- (b) At CG 73.0 mm, bends of 3 deg AND 4 deg both fail (a) (they dive: no pull-out, tumble, or a steady flight path steeper than -45 deg).
- (c) At 6 deg bend, CG 72.0 and 74.0 mm both pass (a) and both keep a persistent oscillation: not classed "smooth", and the flight-path-angle amplitude over the last full cycle available is between 0.5 and 2 times that of the 73.0 mm case."""


# =============================================================================================== specimen (AVL + masses)
def build_avl_numbers(out: Path, log=print):
    """The only AVL runs: Swift A GLIDE outline at 400 mm, tail bend 0 and 5 deg (the model is linear in the bend, as in bird_outline_v4_swift.evaluate4)."""
    g = s4.build_geometry4(s4.GLIDE)
    k = SPAN_MM
    v2.AVL_NCHORD, v2.AVL_NSPAN = 10, 80
    a = {}
    for b in (0.0, 5.0):
        deck = v2.avl_deck2(g, k, b, name=f"swA400_b{int(b)}")
        a[b] = ps.avl_session(deck, f"swA400_b{int(b)}", out / "avl")
        log(f"  AVL bend {b:g}: Xnp {a[b]['Xnp'] * 1000:.2f} mm, CLa {a[b]['CLa_rad']:.3f}/rad, Cmq {a[b]['Cmq']:.3f}")
    return a


def numbers_from_avl(a):
    """Scale-free derivatives from the two AVL sessions.  Xnp, CLa, Cmq from the 5 deg run (the operating bend range is 3-10 deg)."""
    g = s4.build_geometry4(s4.GLIDE)
    k = SPAN_MM
    mac_mm, le_mm = g["mac"] * k, g["x_le_mac"] * k
    a0, a5 = a[0.0], a[5.0]
    xnp = a5["Xnp"] * 1000
    cm_np = lambda r: r["Cm0_ref"] + r["CL0"] * (xnp * 1e-3 - r["Xref"]) / r["Cref"]
    dcl = (a5["CL0"] - a0["CL0"]) / 5.0
    dcm = (cm_np(a5) - cm_np(a0)) / 5.0
    return dict(xnp_mm=xnp, mac_mm=mac_mm, mac_le_mm=le_mm, np_pct_mac=100 * (xnp - le_mm) / mac_mm, cla_rad=a5["CLa_rad"], cmq=a5["Cmq"], dcl_per_deg=dcl, dcm_ac_per_deg=dcm,
                trim_cg_6deg_mm=xnp - mac_mm * (6.0 * dcm) / 0.3, S_m2=g["S"] * (k * 1e-3) ** 2, ar=g["ar"], xref_mm=a5["Xref"] * 1000, cref_mm=a5["Cref"] * 1000)


def mass_basis():
    """Foam 400 as built: sheet, rail, 10 %-margin CG, ballast at the -12.5 mm hole.  The rail is FIXED at the as-built one; moving the CG moves the ballast mass at that hole (ASSUMED way of 'moving the CG')."""
    g = s4.build_geometry4(s4.GLIDE)
    sn = fd.swift_numbers()["foam"]["10"]                                                 # source of the 9.77 g / 1.21 g / 36.96 mm / -12.5 mm numbers
    r = s4.fake_res(sn["cg"] / SPAN_MM, 0.1)
    r["trim"] = {c: dict(x_cg_span=sn["cg"] / SPAN_MM, static_margin=0.1) for c in v2.TRIM_CLS}
    n = v2.size_numbers(g, r, "foam", 6.0)
    assert abs(n["M"] - sn["M"]) < 1e-6 and abs(n["ballast_g"] - sn["ballast"]) < 1e-6, (n["M"], sn["M"])
    poly = [(x * SPAN_MM, y * SPAN_MM) for x, y in g["full_outline"]]
    return dict(g=g, sn=sn, n=n, poly=poly, m_sheet=n["m_sheet"], rail=n["rail"], station=n["station"], M0=n["M0"], cg0=n["cg0"])


def make_spec(numbers, mb):
    g = mb["g"]
    nm = numbers

    masses = make_spec_masses(mb)
    a = dict(Xnp=nm["xnp_mm"] * 1e-3, CLa_rad=nm["cla_rad"], Cmq=nm["cmq"], CLd_per_deg=nm["dcl_per_deg"], Cmd_per_deg=None, dcm_ac_per_deg=nm["dcm_ac_per_deg"], sign=+1)
    cgs = [30.0, 45.0]
    return dict(name="SwiftA_foam400", kind="foam", key="BS", ar=nm["ar"], taper=0.5, S=nm["S_m2"], mac_m=nm["mac_mm"] * 1e-3, mac_le_mm=nm["mac_le_mm"], cgs=cgs, masses=masses,
                avl={cg: dict(a) for cg in cgs}, station=mb["station"], nose=0.0, span_mm=SPAN_MM, tab_note="tail bend", xnp_mm=nm["xnp_mm"], g=g)


def mass_table(mb, cg_mm=36.96):
    spec_m = make_spec_masses(mb)
    m, ball = spec_m(cg_mm)
    rows = [(p[0], p[1], p[2], p[3]) for p in m.parts]
    return rows, m.mass_g(), m.cg_mm(), m.inertia_kg_m2(cg_mm) * 1e9, ball


def make_spec_masses(mb):
    def masses(cg):
        mb_g = mb["M0"] * (cg - mb["cg0"]) / (mb["station"] - cg)                         # ballast needed at the fixed hole to put the CG at cg
        m = ps.MassModel()
        m.add_plate("sheet", mb["m_sheet"], mb["poly"])
        m.add_bar("rail", mb["rail"]["mass_g"], mb["rail"]["x_lo"], mb["rail"]["saddle_end"])
        m.add_point("ballast", mb_g, mb["station"])
        return m, mb_g
    return masses


# =============================================================================================== simulation + trim + modes
def vehicle(spec, plate, cg_nose, bend, **kw):
    return ps.Vehicle(spec, plate, cg_nose - NOSE_AHEAD_MM, bend, **kw)


def simulate_full(veh, t_max=T_MAX, dt=DT, sample=0.02):
    """Same loop as pullout_sim.simulate but without the early stop at a settled glide (the study needs the whole 30 s)."""
    s = [0.0, 0.0, math.radians(-90.0), 0.0, 0.0, 0.0]
    k_s = max(1, int(round(sample / dt)))
    rec = []
    for i in range(int(round(t_max / dt)) + 1):
        if i % k_s == 0:
            x, z, th, vx, vz, q = s
            ct, st = math.cos(th), math.sin(th)
            ff, fn, m, alpha, V = veh.aero(vx * ct + vz * st, -vx * st + vz * ct, q)
            gam = math.degrees(math.atan2(vz, vx)) if V > 0.05 else None
            rec.append((i * dt, x, z, math.degrees(th), vx, vz, V, alpha, gam, math.hypot(ff, fn) / (veh.mass * G), q))
        if abs(s[0]) > 1e4 or abs(s[1]) > 1e4 or any(map(math.isnan, s)):
            break
        s = ps.rk4(veh, s, dt)
    return rec


def lift_coeff(veh, u, w):
    ff, fn, m, alpha, V = veh.aero(u, w, 0.0)
    ar = math.radians(alpha)
    L = ff * math.sin(ar) + fn * math.cos(ar)
    return L / (0.5 * ps.RHO * V * V * veh.S)


def find_trim(veh):
    """Equilibrium glide (q = 0): alpha where the moment is zero with a stable slope (smallest such alpha >= -5 deg), then V from the weight and theta from the force direction.
    -> dict or None."""
    def mom(a_deg):
        r = math.radians(a_deg)
        return veh.aero(math.cos(r), -math.sin(r), 0.0)[2]
    grid = [-5.0 + 0.05 * i for i in range(0, 1301)]            # -5 .. 60 deg
    vals = [mom(a) for a in grid]
    root = None
    for i in range(len(grid) - 1):
        if vals[i] > 0 >= vals[i + 1]:                           # nose-up moment above, nose-down below: stable (negative slope)
            lo, hi = grid[i], grid[i + 1]
            for _ in range(50):
                mid = 0.5 * (lo + hi)
                if mom(mid) > 0:
                    lo = mid
                else:
                    hi = mid
            root = 0.5 * (lo + hi)
            break
    if root is None:
        return None
    r = math.radians(root)
    ff1, fn1, _, _, _ = veh.aero(math.cos(r), -math.sin(r), 0.0)
    H = math.hypot(ff1, fn1)
    if H < 1e-12:
        return None
    V = math.sqrt(veh.mass * G / H)
    th = math.atan2(ff1, fn1)
    u, w = V * math.cos(r), -V * math.sin(r)
    vx, vz = u * math.cos(th) - w * math.sin(th), u * math.sin(th) + w * math.cos(th)
    ff, fn, m, alpha, _ = veh.aero(u, w, 0.0)
    return dict(alpha=root, V=V, theta=math.degrees(th), gamma=math.degrees(math.atan2(vz, vx)), state=[vx, vz, th, 0.0], cl=lift_coeff(veh, u, w),
                a_eff=root - veh.tau * veh.up, below_peak=(root - veh.tau * veh.up) < veh.alpha_stall)


def jacobian_modes(veh, tr):
    """Numerical (central-difference) Jacobian of the 4-state model (vx, vz, theta, q) at the trim; eigenvalues by numpy."""
    s0 = list(tr["state"])
    f = lambda y: [veh.deriv([0.0, 0.0, y[2], y[0], y[1], y[3]])[i] for i in (3, 4, 2, 5)]      # d(vx,vz,theta,q)/dt
    J = np.zeros((4, 4))
    for j in range(4):
        h = 1e-5 if j < 2 else 1e-6
        yp, ym = list(s0), list(s0)
        yp[j] += h
        ym[j] -= h
        fp, fm = f(yp), f(ym)
        for i in range(4):
            J[i, j] = (fp[i] - fm[i]) / (2 * h)
    ev = np.linalg.eigvals(J)
    cx = sorted([e for e in ev if e.imag > 1e-9], key=lambda e: -e.imag)
    modes = {}
    def desc(e):
        wn = abs(e)
        return dict(re=float(e.real), im=float(e.imag), wn=float(wn), period=float(2 * math.pi / e.imag), zeta=float(-e.real / wn), f_hz=float(e.imag / (2 * math.pi)))
    if len(cx) >= 2:
        modes["short"], modes["phugoid"] = desc(cx[0]), desc(cx[-1])
    elif len(cx) == 1:
        modes["osc_only"] = desc(cx[0])
    modes["real"] = sorted(float(e.real) for e in ev if abs(e.imag) <= 1e-9)
    modes["max_re"] = float(max(e.real for e in ev))
    return modes


def zigzag(vals, hyst):
    """Alternating extrema (index, +1 max / -1 min) with hysteresis."""
    ext = []
    if not vals:
        return ext
    mx = mn = vals[0]; imx = imn = 0
    direction = 0
    for i, v in enumerate(vals):
        if v > mx:
            mx, imx = v, i
        if v < mn:
            mn, imn = v, i
        if direction >= 0 and v < mx - hyst:
            if direction == 0 or not ext or ext[-1][1] != 1:
                ext.append((imx, 1))
            direction = -1
            mn, imn = v, i
        elif direction <= 0 and v > mn + hyst:
            if direction == 0 or not ext or ext[-1][1] != -1:
                ext.append((imn, -1))
            direction = 1
            mx, imx = v, i
    return ext


def analyze(rec, veh):
    T = np.array([r[0] for r in rec])
    z = np.array([r[2] for r in rec])
    th = np.array([r[3] for r in rec])
    V = np.array([r[6] for r in rec])
    al = np.array([r[7] for r in rec])
    gam = np.array([np.nan if r[8] is None else r[8] for r in rec])
    out = {}
    out["t_end"] = float(T[-1])
    out["tumbles"] = bool((th.max() - th.min()) >= 360.0)
    i_po = next((i for i in range(len(rec)) if T[i] > 0.05 and V[i] > 0.3 and not np.isnan(gam[i]) and gam[i] >= ps.GAMMA_LEVEL), None)
    out["pulls_out"] = i_po is not None and not out["tumbles"]
    out["t_pullout_s"] = float(T[i_po]) if i_po is not None else None
    out["h_first_level_m"] = float(-z[i_po]) if i_po is not None else None
    nlast = int(SETTLE_WIN_S / 0.02)
    tail = gam[-nlast:]
    out["gamma_final_deg"] = float(np.nanmean(tail)) if not np.all(np.isnan(tail)) else None
    out["speed_final"] = float(np.nanmean(V[-nlast:]))
    out["alpha_final_deg"] = float(np.nanmean(al[-nlast:]))
    out["settled"] = bool(out["pulls_out"] and not np.any(np.isnan(tail)) and (np.nanmax(tail) - np.nanmin(tail)) < SETTLE_RANGE_DEG)
    out["gamma_tail_range_deg"] = float(np.nanmax(tail) - np.nanmin(tail)) if not np.all(np.isnan(tail)) else None
    ret_steep = bool(out["pulls_out"] and np.nanmin(gam[i_po:]) < DIVE_AGAIN)
    out["returns_steep"] = ret_steep
    out["t_settle_2deg_s"] = None
    if out["pulls_out"] and out["gamma_final_deg"] is not None:
        bad = np.where(np.abs(gam - out["gamma_final_deg"]) > 2.0)[0]
        bad = bad[bad >= i_po]
        out["t_settle_2deg_s"] = float(T[bad[-1]]) if len(bad) else float(T[i_po])
        if len(bad) and bad[-1] >= len(T) - 2:
            out["t_settle_2deg_s"] = None                                         # still outside +-2 deg at the end of the run
    out["glide"] = bool(out["pulls_out"] and not ret_steep and out["gamma_final_deg"] is not None and out["gamma_final_deg"] > DIVE_GAMMA)          # pre-stated part (a)
    # oscillation after pull-out
    out.update(period_first_s=None, period_mean_s=None, amp_first_deg=None, amp_last_full_deg=None, decay_ratio=None, decay_word="", n_cycles=0, n_stall_cycles=0, n_stall_after_first=0,
               stall_cycles_total=0, alpha_top_max_deg=None)
    if out["pulls_out"]:
        g2 = gam[i_po:]
        ok = ~np.isnan(g2)
        g2 = np.where(ok, g2, -90.0)
        ext = zigzag(list(g2), HYST_DEG)
        ms = [i + i_po for i, t in ext if t == 1]
        # a first maximum right at the pull-out start that is only a few degrees above the level-off threshold is still counted: it is the top of the first pull-up
        a_eff = al - veh.tau * veh.up
        amps, per_stall = [], []
        for k, m0 in enumerate(ms):
            end = ms[k + 1] if k + 1 < len(ms) else len(rec) - 1
            seg = gam[m0:end + 1]
            amps.append(float(0.5 * (gam[m0] - np.nanmin(seg))))
            per_stall.append(bool(np.nanmax(a_eff[m0:end + 1]) > veh.alpha_stall))
        out["n_cycles"] = len(ms)
        out["cycle_amps_deg"] = amps
        out["cycle_stall"] = per_stall
        if ms:
            out["amp_first_deg"] = amps[0]
            nfull = len(ms) - 1
            out["amp_last_full_deg"] = amps[nfull - 1] if nfull >= 1 else amps[0]
            out["n_stall_cycles"] = int(sum(per_stall))
            out["n_stall_after_first"] = int(sum(per_stall[1:]))
            out["stall_in_first_cycle"] = bool(per_stall[0])
            out["stall_cycles_total"] = len(per_stall)
            out["alpha_top_max_deg"] = float(np.nanmax(a_eff[ms[0]:]))
            if len(ms) >= 2:
                out["period_first_s"] = float(T[ms[1]] - T[ms[0]])
                m = min(len(ms), 5)
                out["period_mean_s"] = float((T[ms[m - 1]] - T[ms[0]]) / (m - 1))
                mm = min(len(ms), 6)
                if amps[0] > 1e-6:
                    out["decay_ratio"] = float((max(amps[mm - 1], 1e-9) / amps[0]) ** (1.0 / (mm - 1)))
            elif out["settled"]:
                out["decay_ratio"] = 0.0
            a3 = amps[2] if len(amps) >= 3 else 0.0
            out["amp_cycle3_deg"] = a3
            out["amp_cycle3_over_1"] = a3 / amps[0] if amps[0] > 1e-6 else 0.0
            r = out["decay_ratio"]
            out["decay_word"] = "" if r is None else ("shrinks" if r < SHRINK_R else ("grows" if r > GROW_R else "persists"))
    # classification
    if not out["glide"]:
        cls = "dive"
    elif out["n_stall_cycles"] >= PORPOISE_CYCLES:
        cls = "porpoise"
    elif out["settled"] and out["n_stall_after_first"] == 0 and out.get("amp_cycle3_over_1", 1.0) < SMOOTH_FRAC:
        cls = "smooth"
    else:
        cls = "persistent phugoid"
    out["class"] = cls
    return out


def run_one(spec, plate, cg_nose, bend, **kw):
    veh = vehicle(spec, plate, cg_nose, bend, **kw)
    rec = simulate_full(veh)
    a = analyze(rec, veh)
    tr = find_trim(veh)
    a.update(cg_nose_mm=cg_nose, bend_deg=bend, mass_g=veh.mass * 1e3, inertia_g_mm2=veh.inertia * 1e9, static_margin_pct=100 * (veh.xnp_frac - veh.xcg_frac), alpha_stall_deg=veh.alpha_stall, tau=veh.tau)
    if tr:
        a.update(trim_alpha=tr["alpha"], trim_cl=tr["cl"], trim_V=tr["V"], trim_gamma=tr["gamma"], trim_below_peak=tr["below_peak"])
        if tr["gamma"] > -85 and tr["below_peak"]:
            m = jacobian_modes(veh, tr)
            a["modes"] = m
            a["T_phugoid_formula_s"] = math.pi * math.sqrt(2) * tr["V"] / G
    return a, rec, veh


# =============================================================================================== parallel driver
_W = {}


def _init(numbers_json):
    nm = json.loads(numbers_json)
    mb = mass_basis()
    _W["spec"] = make_spec(nm, mb)
    _W["plate"] = ps.PlateTable(_W["spec"]["ar"], _W["spec"]["taper"])


def _job(args):
    name, kw, cg, bend = args
    a, rec, veh = run_one(_W["spec"], _W["plate"], cg, bend, **kw)
    a["variant"] = name
    return a


def run_grid(numbers, cgs, bends, jobs, log=print):
    todo = [(name, kw, float(cg), float(b)) for name, kw in VARIANTS for cg in cgs for b in bends]
    t0 = time.time()
    nj = json.dumps(numbers)
    if jobs > 1:
        import multiprocessing as mp
        with mp.get_context("spawn").Pool(jobs, initializer=_init, initargs=(nj,)) as pool:
            res = pool.map(_job, todo, chunksize=4)
    else:
        _init(nj)
        res = [_job(a) for a in todo]
    log(f"  {len(todo)} runs in {time.time() - t0:.0f} s")
    return res


# =============================================================================================== agreement check
def find(cases, variant, cg, bend):
    return next(c for c in cases if c["variant"] == variant and abs(c["cg_nose_mm"] - cg) < 1e-6 and abs(c["bend_deg"] - bend) < 1e-6)


def agreement(cases, variant="base"):
    f = lambda cg, b: find(cases, variant, cg, b)
    a = f(73, 6)
    pa = bool(a["glide"])
    pb_ = (not f(73, 3)["glide"]) and (not f(73, 4)["glide"])
    r72, r74 = f(72, 6), f(74, 6)
    ref = a["amp_last_full_deg"] if a["amp_last_full_deg"] is not None else 0.0
    def part_c(r):
        if not r["glide"] or r["class"] == "smooth" or r["amp_last_full_deg"] is None or ref <= 0:
            return False
        return 0.5 <= r["amp_last_full_deg"] / ref <= 2.0
    pc = part_c(r72) and part_c(r74)
    g_ = lambda v, n=3: "n/a" if v is None else f"{v:.{n}g}"
    detail = dict(
        a=f"73/6: class {a['class']}; pulls out {a['pulls_out']}; mean flight path over the last 5 s {g_(a['gamma_final_deg'])} deg; returns steeper than -80 deg: {a['returns_steep']}",
        b=f"73/3: {f(73, 3)['class']} (glide {f(73, 3)['glide']}); 73/4: {f(73, 4)['class']} (glide {f(73, 4)['glide']})",
        c=f"72/6: {r72['class']}, last-full-cycle amplitude {g_(r72['amp_last_full_deg'])} deg; 74/6: {r74['class']}, {g_(r74['amp_last_full_deg'])} deg; 73/6: {g_(ref)} deg (ratios {g_(r72['amp_last_full_deg'] / ref if (ref and r72['amp_last_full_deg'] is not None) else None)} and {g_(r74['amp_last_full_deg'] / ref if (ref and r74['amp_last_full_deg'] is not None) else None)}; allowed 0.5-2)")
    return dict(a=pa, b=pb_, c=pc, detail=detail)


# =============================================================================================== extras + summary
EXTRA_EFF = (1.0, 0.75, 0.5, 0.4, 0.3, 0.2)
EXTRA_OFF = (0.0, -0.008, -0.016, -0.024, -0.032)
EXTRA_BENDS_LOW = (0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0)


def extras(spec, plate):
    """(1) bend scan at CG 73.0 mm below 3 deg for every variant; (2) what bend effectiveness / Cm offset would turn 3 and 4 deg at CG 73.0 mm into a dive."""
    low = {}
    for name, kw in VARIANTS:
        low[name] = [(b, run_one(spec, plate, 73.0, b, **kw)[0]) for b in EXTRA_BENDS_LOW]
    need = {}
    for bend in (3.0, 4.0, 6.0):
        need[bend] = {(e, o): run_one(spec, plate, 73.0, bend, eff=e, cm_off=o)[0]["class"] for e in EXTRA_EFF for o in EXTRA_OFF}
    return dict(low=low, need=need)


def summarize(cases, cgs, bends):
    names = [v for v, _ in VARIANTS]
    S = {}
    S["agree"] = {v: agreement(cases, v) for v in names}
    cells = [(c, b) for c in cgs for b in bends]
    S["smooth_all"] = [cb for cb in cells if all(find(cases, v, *cb)["class"] == "smooth" for v in names)]
    S["smooth_base"] = [cb for cb in cells if find(cases, "base", *cb)["class"] == "smooth"]
    S["porp_all"] = [cb for cb in cells if all(find(cases, v, *cb)["class"] == "porpoise" for v in names)]
    S["porp_base"] = [cb for cb in cells if find(cases, "base", *cb)["class"] == "porpoise"]
    S["dive_any"] = [cb for cb in cells if any(find(cases, v, *cb)["class"] == "dive" for v in names)]
    S["dive_base"] = [cb for cb in cells if find(cases, "base", *cb)["class"] == "dive"]
    S["count_smooth"] = {cb: sum(find(cases, v, *cb)["class"] == "smooth" for v in names) for cb in cells}
    S["count_porp"] = {cb: sum(find(cases, v, *cb)["class"] == "porpoise" for v in names) for cb in cells}
    S["asflown"] = {v: find(cases, v, *AS_FLOWN) for v in names}
    base_smooth = [find(cases, "base", *cb) for cb in S["smooth_base"]]
    dist = lambda c: math.hypot(c["cg_nose_mm"] - AS_FLOWN[0], c["bend_deg"] - AS_FLOWN[1])
    ranked = sorted([c for c in base_smooth if c["t_settle_2deg_s"] is not None], key=lambda c: (c["t_settle_2deg_s"], dist(c)))
    S["best"] = [(c["cg_nose_mm"], c["bend_deg"]) for c in ranked[:4]]
    # robust-by-variant: cells that are not dive in the base and keep the same class in every variant
    S["same_all"] = [cb for cb in cells if len({find(cases, v, *cb)["class"] for v in names}) == 1]
    return S


# =============================================================================================== SVG / HTML helpers
CLS_CSS = {"dive": "c-dive", "porpoise": "c-porp", "persistent phugoid": "c-phug", "smooth": "c-smooth"}
CLS_LETTER = {"dive": "D", "porpoise": "P", "persistent phugoid": "h", "smooth": "S"}
TAGCLS = {"MODEL": "t-model", "CALCULATED": "t-calc", "ASSUMED": "t-assum", "MEASURED": "t-meas"}


def tag(t):
    return f'<span class="tag {TAGCLS[t]}">{t}</span>'


def fmt(v, n=3):
    if v is None:
        return ""
    if isinstance(v, bool):
        return str(v)
    if isinstance(v, float):
        return f"{v:.{n}g}"
    return str(v)


def svg_map(cases, variant, cgs, bends, small=False, title="", mark=AS_FLOWN, cell_text=True):
    cw, ch = (26, 17) if small else (62, 36)
    L, T_, R, B = (30, 18, 6, 26) if small else (64, 26, 10, 44)
    W, H = L + cw * len(cgs) + R, T_ + ch * len(bends) + B
    o = [f'<svg class="map" viewBox="0 0 {W} {H}" width="{W}" height="{H}" role="img" aria-label="{html.escape(title or variant)}">']
    if title:
        o.append(f'<text class="ttl" x="{L}" y="{T_ - 8}" font-size="{10 if small else 13}">{html.escape(title)}</text>')
    for i, cg in enumerate(cgs):
        for j, b in enumerate(reversed(bends)):
            c = find(cases, variant, cg, b)
            x, y = L + i * cw, T_ + j * ch
            tip = (f"CG {cg:g} mm behind nose tip, bend {b:g} deg: {c['class']}; SM {c['static_margin_pct']:.1f}% MAC; decay ratio/cycle {fmt(c['decay_ratio'], 2)}; "
                   f"stall cycles {c['n_stall_cycles']}; first-cycle amp {fmt(c['amp_first_deg'], 3)} deg; trim CL {fmt(c.get('trim_cl'), 3)}")
            o.append(f'<rect class="{CLS_CSS[c["class"]]}" x="{x}" y="{y}" width="{cw}" height="{ch}"><title>{html.escape(tip)}</title></rect>')
            if cell_text and not small:
                txt = "D" if c["class"] == "dive" else fmt(c["decay_ratio"], 2)
                o.append(f'<text class="cell" x="{x + cw / 2}" y="{y + ch / 2 + 4}" text-anchor="middle" font-size="12">{txt}</text>')
            if mark and abs(cg - mark[0]) < 1e-6 and abs(b - mark[1]) < 1e-6:
                o.append(f'<rect class="mark" x="{x + 1.5}" y="{y + 1.5}" width="{cw - 3}" height="{ch - 3}"/>')
    fs = 9 if small else 12
    for i, cg in enumerate(cgs):
        if not small or i % 2 == 0:
            o.append(f'<text class="ax" x="{L + i * cw + cw / 2}" y="{T_ + ch * len(bends) + (11 if small else 16)}" text-anchor="middle" font-size="{fs}">{cg:g}</text>')
    for j, b in enumerate(reversed(bends)):
        if not small or j % 2 == 0:
            o.append(f'<text class="ax" x="{L - 5}" y="{T_ + j * ch + ch / 2 + 4}" text-anchor="end" font-size="{fs}">{b:g}</text>')
    if not small:
        o.append(f'<text class="ax" x="{L + cw * len(cgs) / 2}" y="{H - 6}" text-anchor="middle" font-size="12">CG, mm behind the nose tip (root leading edge is 36 mm further back)</text>')
        o.append(f'<text class="ax" transform="translate(14 {T_ + ch * len(bends) / 2}) rotate(-90)" text-anchor="middle" font-size="12">tail bend, deg (trailing edge up)</text>')
    o.append("</svg>")
    return "\n".join(o)


def svg_plot(series, xl, yl, w=470, h=250, xlim=None, ylim=None, hlines=(), title=""):
    """series: [(label, css class, [(x, y)])]; hlines: [(y, label)] dashed."""
    xs = [p[0] for _, _, pts in series for p in pts]
    ys = [p[1] for _, _, pts in series for p in pts] + [y for y, _ in hlines]
    x0, x1 = xlim or (min(xs), max(xs))
    y0, y1 = ylim or (min(ys), max(ys))
    if y1 - y0 < 1e-9:
        y1 = y0 + 1
    L, R, T_, B = 52, 10, 22, 36
    fx = lambda x: L + (x - x0) / (x1 - x0) * (w - L - R)
    fy = lambda y: T_ + (1 - (y - y0) / (y1 - y0)) * (h - T_ - B)
    o = [f'<svg class="plot" viewBox="0 0 {w} {h}" width="{w}" height="{h}" role="img" aria-label="{html.escape(title)}">']
    o.append(f'<text class="ttl" x="{L}" y="14" font-size="12">{html.escape(title)}</text>')
    def ticks(a, b, n=5):
        step = (b - a) / n
        mag = 10 ** math.floor(math.log10(step)) if step > 0 else 1
        step = min((m * mag for m in (1, 2, 5, 10) if m * mag >= step), default=step)
        t = math.ceil(a / step) * step
        out = []
        while t <= b + 1e-9:
            out.append(t)
            t += step
        return out
    for t in ticks(x0, x1):
        o.append(f'<line class="grid" x1="{fx(t):.1f}" x2="{fx(t):.1f}" y1="{T_}" y2="{h - B}"/><text class="ax" x="{fx(t):.1f}" y="{h - B + 14}" text-anchor="middle" font-size="11">{t:g}</text>')
    for t in ticks(y0, y1):
        o.append(f'<line class="grid" y1="{fy(t):.1f}" y2="{fy(t):.1f}" x1="{L}" x2="{w - R}"/><text class="ax" x="{L - 5}" y="{fy(t) + 4:.1f}" text-anchor="end" font-size="11">{t:g}</text>')
    for y, lab in hlines:
        o.append(f'<line class="hl" x1="{L}" x2="{w - R}" y1="{fy(y):.1f}" y2="{fy(y):.1f}"/><text class="ax" x="{w - R - 2}" y="{fy(y) - 3:.1f}" text-anchor="end" font-size="10">{html.escape(lab)}</text>')
    for lab, cls, pts in series:
        pts = [p for p in pts if x0 <= p[0] <= x1]
        o.append(f'<polyline class="{cls}" fill="none" points="{" ".join(f"{fx(x):.1f},{fy(y):.1f}" for x, y in pts)}"/>')
    o.append(f'<text class="ax" x="{(L + w - R) / 2}" y="{h - 4}" text-anchor="middle" font-size="12">{html.escape(xl)}</text>')
    o.append(f'<text class="ax" transform="translate(12 {(T_ + h - B) / 2}) rotate(-90)" text-anchor="middle" font-size="12">{html.escape(yl)}</text>')
    o.append("</svg>")
    return "\n".join(o)


def table(header, rows, cls=""):
    h = "".join(f"<th>{c}</th>" for c in header)
    b = "".join("<tr>" + "".join(f"<td>{c}</td>" for c in r) + "</tr>" for r in rows)
    return f'<div class="tw"><table class="{cls}"><thead><tr>{h}</tr></thead><tbody>{b}</tbody></table></div>'


CSS = """:root{color-scheme:dark;--bg:#14181d;--panel:#1c2229;--line:#333c46;--ink:#dde3ea;--muted:#9aa7b4;--link:#7fb4ff;--accent:#e89a55;--thead:#262e37;
--dive:#d9695f;--porp:#e89a55;--phug:#d6c35e;--smooth:#6cc08b;--good:#6cc08b;--bad:#ff8a80;--mark:#ffffff;--l1:#7fb4ff;--l2:#e89a55;--l3:#6cc08b;--l4:#c69cff}
*{box-sizing:border-box}body{background:var(--bg);color:var(--ink);font:15px/1.5 system-ui,Segoe UI,sans-serif;margin:0 auto;max-width:1100px;padding:18px 22px 60px}
h1{font-size:23px;margin:.2em 0}h2{font-size:18px;margin:1.7em 0 .4em;border-top:1px solid var(--line);padding-top:.7em}h3{font-size:15px;margin:1.1em 0 .3em}
p,li{max-width:84ch}a{color:var(--link)}code{background:var(--panel);padding:0 4px;border-radius:3px}.note{color:var(--muted);font-size:13px}
.tw{overflow-x:auto;margin:.5em 0}table{border-collapse:collapse;font-size:13px;background:var(--panel)}th,td{border:1px solid var(--line);padding:3px 8px;text-align:left;vertical-align:top}th{background:var(--thead)}
.tag{font-size:10.5px;font-weight:600;border-radius:3px;padding:0 4px;margin-left:3px;border:1px solid var(--line);white-space:nowrap}
.t-model{color:#7fb4ff}.t-calc{color:#6cc08b}.t-assum{color:#e89a55}.t-meas{color:#c69cff}
.pass{color:var(--good);font-weight:700}.fail{color:var(--bad);font-weight:700}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:6px;padding:10px 14px;margin:.7em 0}
svg{background:var(--panel);border:1px solid var(--line);border-radius:4px;max-width:100%;height:auto}
svg .ttl{fill:var(--ink)}svg .ax{fill:var(--muted)}svg .grid{stroke:var(--line);stroke-width:.5}svg .hl{stroke:var(--bad);stroke-dasharray:4 3;stroke-width:1}
svg .cell{fill:#14181d}svg .mark{fill:none;stroke:var(--mark);stroke-width:2}
.c-dive{fill:var(--dive)}.c-porp{fill:var(--porp)}.c-phug{fill:var(--phug)}.c-smooth{fill:var(--smooth)}
svg polyline{stroke-width:1.6}svg .s1{stroke:var(--l1)}svg .s2{stroke:var(--l2)}svg .s3{stroke:var(--l3)}svg .s4{stroke:var(--l4)}
.sw{display:inline-block;width:12px;height:12px;border-radius:2px;vertical-align:-1px;margin:0 4px 0 12px}
.sw.c-dive{background:var(--dive)}.sw.c-porp{background:var(--porp)}.sw.c-phug{background:var(--phug)}.sw.c-smooth{background:var(--smooth)}
.grid2{display:grid;grid-template-columns:repeat(auto-fit,minmax(450px,1fr));gap:10px}.small-maps{display:flex;flex-wrap:wrap;gap:8px}
"""


def legend():
    return ('<p class="note"><span class="sw c-dive"></span>dive (no clean pull-out, tumble, or steady path steeper than -45 deg)'
            '<span class="sw c-porp"></span>stall porpoise (alpha passes the lift peak on 2+ cycles)'
            '<span class="sw c-phug"></span>persistent phugoid (no repeated stall, decay too slow for "smooth")'
            '<span class="sw c-smooth"></span>smooth. Number in a cell = amplitude ratio per cycle (below 0.8 shrinks, 0.8-1.1 persists, above 1.1 grows). White frame = as flown (73 mm, 6 deg).</p>')


# =============================================================================================== writers
def specimen_rows(numbers, mb, spec, plate, re_txt=""):
    sn = mb["sn"]
    rows_m, M, cg, inertia, ball = mass_table(mb, sn["cg"])
    rows = [
        ("Outline", "Swift A GLIDE outline (bird_outline_v4_swift.GLIDE), foamboard, span 400 mm", "MODEL"),
        ("Wing area S", f"{numbers['S_m2'] * 1e4:.1f} cm2", "MODEL"),
        ("Aspect ratio b^2/S", f"{numbers['ar']:.2f}", "MODEL"),
        ("Mean aerodynamic chord", f"{numbers['mac_mm']:.1f} mm (leading edge at {numbers['mac_le_mm']:.1f} mm behind the root LE)", "MODEL"),
        ("Neutral point Xnp (AVL, 5 deg bend run)", f"{numbers['xnp_mm']:.2f} mm behind root LE = {numbers['xnp_mm'] + NOSE_AHEAD_MM:.2f} mm behind nose tip = {numbers['np_pct_mac']:.1f} % MAC", "MODEL"),
        ("Lift slope CLa", f"{numbers['cla_rad']:.3f} per rad ({numbers['cla_rad'] / 57.29578:.4f} per deg)", "MODEL"),
        ("Pitch damping Cmq (about the CG, per q c/2V)", f"{numbers['cmq']:.3f}", "MODEL"),
        ("Lift change per deg of tail bend", f"{numbers['dcl_per_deg']:+.5f} per deg", "MODEL"),
        ("Cm0 per deg of tail bend (about the NP)", f"{numbers['dcm_ac_per_deg']:+.5f} per deg (v4 outlines.csv: +0.00501)", "MODEL"),
        ("CG that trims CL 0.3 at 6 deg bend", f"{numbers['trim_cg_6deg_mm']:.2f} mm behind root LE = {numbers['trim_cg_6deg_mm'] + NOSE_AHEAD_MM:.2f} mm behind nose tip (flight-day pages: {sn['cg']:.2f} mm)", "CALCULATED"),
        ("Tail hinge", f"{mb['g']['x_h'] * SPAN_MM + NOSE_AHEAD_MM:.1f} mm behind nose tip", "MODEL"),
    ]
    for name, m_, x, i_ in rows_m:
        rows.append((f"Mass: {name}", f"{m_:.3f} g, centroid {x:.1f} mm behind root LE, own inertia {i_:.0f} g mm2", "CALCULATED"))
    rows += [
        ("All-up mass at CG 36.96 mm (73.0 behind nose tip)", f"{M:.3f} g (flight-day pages: {sn['M']:.3f} g; Kai's 'about 9.77 g'); NOT WEIGHED", "CALCULATED"),
        ("Ballast", f"{ball:.3f} g at the {mb['station']:g} mm hole (rail fixed as built; moving the CG changes this mass)", "CALCULATED"),
        ("Pitch inertia about the CG", f"{inertia:.0f} g mm2 = {inertia * 1e-9:.3e} kg m2 (sheet as uniform plate + rail bar + ballast point)", "CALCULATED"),
        ("Wing loading", f"{M * 1e-3 * G / numbers['S_m2']:.2f} N/m2", "CALCULATED"),
        ("Normal/axial force and moment vs alpha", f"Ananda 2012 plate table, run {plate.run}, wing {plate.wing}, Re {plate.Re:g} (lowest tested), made symmetric", "MEASURED"),
        ("Carry-over of the plate table to the Swift outline", "tapered/swept outline and foam surface are NOT what was tested; " + re_txt, "ASSUMED"),
        ("First lift peak (table)", f"alpha {plate.alpha_stall:g} deg, CL {plate.cl_peak:.2f}; the stall test compares alpha - tau x bend with this angle", "MEASURED"),
        ("Table AC; AC alternative", f"AC moved to the AVL NP (default); alternative: table quarter chord on the NP", "ASSUMED"),
        ("Air", "rho 1.225 kg/m3, nu 1.5e-5 m2/s, still air, no quad downwash, no unsteady aerodynamics", "ASSUMED"),
        ("CG of the flown glider", "73.0 mm behind the nose tip ('very close to the marked point', Kai, not measured)", "ASSUMED"),
        ("Tail bend of the flown glider", "6 deg planned; the foam hinge's real bend angle and springback were not measured", "ASSUMED"),
        ("Start", "released from rest, nose straight down (theta -90 deg), 30 s simulated, RK4 dt 0.002 s", "ASSUMED"),
    ]
    return rows


def pass_word(b):
    return '<span class="pass">PASS</span>' if b else '<span class="fail">FAIL</span>'


def fmt_modes(c):
    m = c.get("modes")
    if not m or "phugoid" not in m:
        return None
    return m


def write_all(out: Path, numbers, cases, cgs, bends, log=print):
    t0 = time.time()
    mb = mass_basis()
    spec = make_spec(numbers, mb)
    plate = ps.PlateTable(spec["ar"], spec["taper"])
    names = [v for v, _ in VARIANTS]
    S = summarize(cases, cgs, bends)
    ex = extras(spec, plate)
    nu, mac_m = ps.NU, spec["mac_m"]
    af_ = S["asflown"]["base"]
    a_, rec_, _ = run_one(spec, plate, *AS_FLOWN)
    t_end_sw = (a_["t_pullout_s"] or 0) + (a_["period_first_s"] or 3.0)
    vmax = max(r[6] for r in rec_ if r[0] <= t_end_sw)
    gl = [c["trim_V"] for c in cases if c["variant"] == "base" and c.get("trim_V") and c.get("trim_gamma", -90) > -45 and c.get("trim_below_peak")]
    re_tr, re_pk = af_["trim_V"] * mac_m / nu, vmax * mac_m / nu
    S["re"] = (f"computed Re = V x MAC / nu with MAC {mac_m * 1e3:.1f} mm, nu {nu:g} m2/s: as flown 73/6 trim V {af_['trim_V']:.2f} m/s gives Re {re_tr:,.0f}, "
               f"fastest point of the first swoop V {vmax:.2f} m/s gives Re {re_pk:,.0f}; trim Re over the {len(gl)} base glide cases {min(gl) * mac_m / nu:,.0f}-{max(gl) * mac_m / nu:,.0f}; "
               f"the table's lowest Re is {plate.Re:,.0f}, so the as-flown trim Re is {plate.Re / re_tr:.1f}x below it (peak {plate.Re / re_pk:.1f}x below)")
    log(S["re"])
    base = [c for c in cases if c["variant"] == "base"]
    # ---- csvs
    cols = ["cg_nose_mm", "bend_deg", "mass_g", "static_margin_pct", "class", "pulls_out", "tumbles", "h_first_level_m", "t_pullout_s", "glide", "settled", "t_settle_2deg_s", "trim_cl", "trim_V", "trim_gamma", "trim_alpha",
            "speed_final", "gamma_final_deg", "alpha_final_deg", "period_first_s", "period_mean_s", "amp_first_deg", "amp_cycle3_deg", "amp_last_full_deg", "decay_ratio", "decay_word", "n_cycles", "n_stall_cycles", "n_stall_after_first",
            "stall_in_first_cycle", "alpha_top_max_deg", "alpha_stall_deg"]
    def wcsv(path, rows, cols_):
        with path.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(cols_)
            for r in rows:
                w.writerow([fmt(r.get(k), 5) if not isinstance(r.get(k), float) else f"{r[k]:.5g}" for k in cols_])
    wcsv(out / "cases.csv", base, cols)
    wcsv(out / "sensitivity_cases.csv", [c for c in cases if c["variant"] != "base"], ["variant"] + cols)
    mcols = ["variant", "cg_nose_mm", "bend_deg", "class", "static_margin_pct", "trim_V", "trim_cl", "trim_gamma", "phugoid_period_s", "phugoid_zeta", "phugoid_T_formula_s", "phugoid_sim_period_s", "short_period_s", "short_zeta", "short_freq_hz"]
    mrows = []
    for c in cases:
        m = fmt_modes(c)
        if m:
            mrows.append(dict(variant=c["variant"], cg_nose_mm=c["cg_nose_mm"], bend_deg=c["bend_deg"], **{"class": c["class"]}, static_margin_pct=c["static_margin_pct"], trim_V=c["trim_V"], trim_cl=c["trim_cl"],
                              trim_gamma=c["trim_gamma"], phugoid_period_s=m["phugoid"]["period"], phugoid_zeta=m["phugoid"]["zeta"], phugoid_T_formula_s=c["T_phugoid_formula_s"], phugoid_sim_period_s=c["period_mean_s"],
                              short_period_s=m["short"]["period"], short_zeta=m["short"]["zeta"], short_freq_hz=m["short"]["f_hz"]))
    wcsv(out / "modes.csv", mrows, mcols)
    (out / "cases_raw.json").write_text(json.dumps(cases, indent=1, default=float))
    # ---- example trajectories
    af = S["asflown"]["base"]
    pick = [("as flown: CG 73, bend 6", AS_FLOWN)]
    lower = (73.0, 3.0)
    pick.append((f"same CG, bend 3 (Kai: nose dive; model: {find(cases, 'base', *lower)['class']})", lower))
    porps = sorted([c for c in base if c["class"] == "porpoise"], key=lambda c: (-c["n_stall_cycles"], math.hypot(c["cg_nose_mm"] - AS_FLOWN[0], c["bend_deg"] - AS_FLOWN[1])))
    if porps:
        pick.append((f"stall-porpoise example: CG {porps[0]['cg_nose_mm']:g}, bend {porps[0]['bend_deg']:g}", (porps[0]["cg_nose_mm"], porps[0]["bend_deg"])))
    sm = sorted([c for c in base if c["class"] == "smooth"], key=lambda c: math.hypot(c["cg_nose_mm"] - AS_FLOWN[0], c["bend_deg"] - AS_FLOWN[1]))
    sm = [c for c in sm if (c["cg_nose_mm"], c["bend_deg"]) not in [p[1] for p in pick]]
    if sm:
        pick.append((f"smooth example nearest the flown case: CG {sm[0]['cg_nose_mm']:g}, bend {sm[0]['bend_deg']:g}", (sm[0]["cg_nose_mm"], sm[0]["bend_deg"])))
    pick = pick[:4]
    traj = []
    for lab, (cg, b) in pick:
        a, rec, veh = run_one(spec, plate, cg, b)
        traj.append((lab, cg, b, a, rec, veh))
    # ---- HTML
    ag = S["agree"]["base"]
    H = []
    H.append(f"<!doctype html><html lang='en'><head><meta charset='utf-8'><title>Swift A foam 400: pitch oscillations (model study)</title><meta name='color-scheme' content='dark'><style>{CSS}</style></head><body>")
    H.append("<h1>Swift A foamboard 400 mm: what are the pitch oscillations?</h1>")
    H.append(f"<p class='note'>Generated {time.strftime('%Y-%m-%d')} by <code>rigid_glider/swiftA_oscillation_study.py</code>. Everything on this page is {tag('MODEL')} output of a 3-DOF drop simulation (nose-down release from rest, still air). "
             f"Nothing here is a flight result. Flight evidence used: Kai's 2026-10-20 words, qualitative (<code>results/runs/flight_day_2026-10-20/FLIGHT_LOG.md</code>).</p>")
    H.append("<h2>1. Agreement check (stated before the grid was looked at)</h2>")
    arows = [("(a)", "CG 73.0 mm, bend 6 deg pulls out and goes on to a glide", pass_word(ag["a"]), html.escape(ag["detail"]["a"])),
             ("(b)", "CG 73.0 mm, bend 3 AND 4 deg do not give a glide (dive)", pass_word(ag["b"]), html.escape(ag["detail"]["b"])),
             ("(c)", "6 deg, CG 72 and 74 mm both still oscillate, last-cycle amplitude 0.5-2x the 73 mm case", pass_word(ag["c"]), html.escape(ag["detail"]["c"]))]
    H.append(table(["part", "statement", "result", "numbers"], arows))
    all3 = ag["a"] and ag["b"] and ag["c"]
    H.append(f"<p class='panel'><b>Overall: the model {'agrees' if all3 else 'does NOT agree'} with the flight on all three parts.</b> The pre-stated wording is in README.md. "
             f"{tag('MODEL')} verdicts are against Kai's qualitative report, so a pass here is weak evidence and a fail is a statement about the model's limits (plate-table carry-over, bend effectiveness), not about the flight.</p>")
    # plain summary
    H.append("<h2>2. What the model says, in plain language</h2>")
    H.append(html_summary(S, cases, ex, numbers, af))
    H.append("<h2>3. Map: CG x tail bend (base model)</h2>")
    H.append(legend())
    H.append(svg_map(cases, "base", cgs, bends, title="outcome after a nose-down drop, 30 s simulated"))
    H.append("<p class='note'>Hover a cell for its numbers. All cells: cases.csv (one row per cell, source-tagged in the README).</p>")
    H.append("<h2>4. Example trajectories</h2><p class='note'>Height change vs distance is NOT to equal scale (height is stretched). Alpha is the model's angle of attack; the dashed line is the first lift peak of the plate table shifted by the bend (MEASURED table, ASSUMED carry-over).</p>")
    H.append("<div class='grid2'>")
    for k, (lab, cg, b, a, rec, veh) in enumerate(traj):
        cls = f"s{k + 1}"
        sub = rec[::2]
        stall_alpha = veh.alpha_stall + veh.tau * veh.up
        H.append("<div class='panel'><b>" + html.escape(lab) + f"</b> {tag('MODEL')}<br><span class='note'>class {a['class']}; pulls out {a['pulls_out']}; height lost to first level-off {fmt(a['h_first_level_m'], 3)} m; "
                 f"first-cycle period {fmt(a['period_first_s'], 3)} s; decay ratio/cycle {fmt(a['decay_ratio'], 2)}; stall on {a['n_stall_cycles']} cycle(s)</span><br>")
        H.append(svg_plot([("path", cls, [(r[1], r[2]) for r in sub if r[0] <= 15])], "distance forward, m (first 15 s)", "height change, m", w=440, h=220, title="flight path"))
        H.append(svg_plot([("alpha", cls, [(r[0], r[7]) for r in sub])], "time, s", "angle of attack, deg", w=440, h=220, xlim=(0, 15), ylim=(-10, 60), hlines=[(stall_alpha, "first lift peak")], title="alpha vs time"))
        H.append("</div>")
    H.append("</div>")
    H.append("<h2>5. Linear modes (numerical Jacobian) at the trim of each glide</h2>")
    H.append(html_modes(S, cases))
    H.append("<h2>6. Sensitivity</h2>")
    H.append(html_sens(S, cases, cgs, bends, ex, names))
    H.append("<h2>7. Specimen: Swift A foam 400 as modelled</h2>")
    H.append(table(["quantity", "value", "source"], [(html.escape(a), html.escape(b), tag(c)) for a, b, c in specimen_rows(numbers, mb, spec, plate, S["re"])]))
    H.append("<h2>8. Definitions and files</h2>")
    H.append(html_defs())
    H.append("</body></html>")
    (out / "index.html").write_text("\n".join(H), encoding="utf-8")
    (out / "README.md").write_text(build_readme(S, cases, ex, numbers, af, cgs, bends, names, specimen_rows(numbers, mb, spec, plate, S["re"]), traj), encoding="utf-8")
    log(f"outputs written to {out} ({time.time() - t0:.0f} s)")


def cell_name(cb):
    return f"{cb[0]:g}/{cb[1]:g}"


def runs_of(cells):
    """'73/6, 74/6' compact list; grouped by CG."""
    by = {}
    for cg, b in cells:
        by.setdefault(cg, []).append(b)
    return "; ".join(f"CG {cg:g}: bend {', '.join(f'{b:g}' for b in sorted(bs))}" for cg, bs in sorted(by.items()))


def facts(S, cases, ex, numbers, af):
    """Numbers shared by the HTML summary and the README."""
    f = {}
    f["af"] = af
    m = af.get("modes") or {}
    f["T_lin"] = m["phugoid"]["period"] if "phugoid" in m else None
    f["zeta"] = m["phugoid"]["zeta"] if "phugoid" in m else None
    f["sp_T"] = m["short"]["period"] if "short" in m else None
    f["T_form"] = af.get("T_phugoid_formula_s")
    per = [c["modes"]["phugoid"]["period"] for c in S["asflown"].values() if c.get("modes") and "phugoid" in c["modes"]]
    f["T_lin_range"] = (min(per), max(per)) if per else None
    sim = [c["period_mean_s"] for c in S["asflown"].values() if c["period_mean_s"]]
    f["T_sim_range"] = (min(sim), max(sim)) if sim else None
    f["classes_asflown"] = {v: c["class"] for v, c in S["asflown"].items()}
    f["a_stall_first"] = sum(1 for c in S["asflown"].values() if c.get("stall_in_first_cycle"))
    f["a_stall_after"] = sum(1 for c in S["asflown"].values() if c.get("n_stall_after_first", 0) > 0)
    f["b_pass_vars"] = [v for v, a in S["agree"].items() if a["b"]]
    f["a_pass_vars"] = [v for v, a in S["agree"].items() if a["a"]]
    f["c_pass_vars"] = [v for v, a in S["agree"].items() if a["c"]]
    return f


def conclusions(S, cases, ex, af):
    """Data-driven statements about what survives the sensitivity variants."""
    from collections import Counter
    names = [v for v, _ in VARIANTS]
    n = len(names)
    L = []
    cnt = Counter(c["class"] for c in S["asflown"].values())
    L.append(f"As flown (73 mm, 6 deg): it pulls out and glides (part a) in {sum(1 for v in names if S['agree'][v]['a'])} of {n} variants; the class is " + ", ".join(f"{k} in {v}" for k, v in cnt.most_common()) + ". "
             f"A stall at the top of the first swoop occurs in {sum(1 for c in S['asflown'].values() if c.get('stall_in_first_cycle'))} of {n}; stalls on 2 or more cycles (true porpoise) in {sum(1 for c in S['asflown'].values() if c['n_stall_cycles'] >= PORPOISE_CYCLES)} of {n}.")
    per = [c["modes"]["phugoid"]["period"] for c in S["asflown"].values() if c.get("modes") and "phugoid" in c["modes"]]
    z = [c["modes"]["phugoid"]["zeta"] for c in S["asflown"].values() if c.get("modes") and "phugoid" in c["modes"]]
    L.append(f"The phugoid period at the flown setting is {min(per):.2f}-{max(per):.2f} s and its damping ratio {min(z):.2f}-{max(z):.2f} across the variants: the oscillation is always lightly damped; the as-flown run settles (flight path within a 5 deg band over the last 5 s of 30 s) in {sum(1 for c in S['asflown'].values() if c['settled'])} of {n} variants.")
    bp = [v for v in names if S["agree"][v]["b"]]
    L.append(f"Part (b) (3 and 4 deg dive at 73 mm) passes in {len(bp)} of {n} variants" + (f" ({', '.join(bp)})" if bp else "") + ": the model's dive limit at CG 73 mm is about "
             + ", ".join(f"{next((b for b, a in lst if a['class'] != 'dive'), None):g} deg ({name})" for name, lst in list(ex["low"].items())[:5] if next((b for b, a in lst if a['class'] != 'dive'), None) is not None) + ", whereas Kai reported a nose dive with 'less' bend than 6 deg (how much less was not recorded).")
    combo = [ex["need"][b][(0.5, -0.008)] for b in (3.0, 4.0, 6.0)]
    L.append(f"Only combined changes reproduce the dive at 3 and 4 deg: bend effectiveness 0.5 together with a Cm offset of -0.008 gives {combo[0]} at 3 deg, {combo[1]} at 4 deg and {combo[2]} at 6 deg (73 mm); that combination is inside the stated ranges of each parameter but is a stacked guess, and at 6 deg it removes the large oscillation (classed {combo[2]}), which conflicts with the flight.")
    sm = [find(cases, "base", *cb) for cb in S["smooth_base"]]
    dv = sorted({v for v in names for cb in [(c["cg_nose_mm"], c["bend_deg"]) for c in cases if c["variant"] == v] if find(cases, v, *cb)["class"] == "dive"})
    L.append(f"Smooth region: {len(S['smooth_base'])} cells in the base model, {len(S['smooth_all'])} in all {n} variants, always at forward CG and small bend "
             f"(base smooth cells: static margin {min(c['static_margin_pct'] for c in sm):.1f}-{max(c['static_margin_pct'] for c in sm):.1f} % MAC, trim CL {min(c['trim_cl'] for c in sm):.2f}-{max(c['trim_cl'] for c in sm):.2f}). "
             f"At 6 deg the smooth CGs (base) are {', '.join(f"{cb[0]:g}" for cb in S['smooth_base'] if cb[1] == 6.0) or 'none'} mm; the flown 73 mm is outside it in the base model and inside it in {sum(1 for v in names if find(cases, v, 73.0, 6.0)['class'] == 'smooth')} of {n} variants.")
    L.append("Variants that add any dive inside the grid: " + (", ".join(dv) if dv else "none") + ".")
    porp7 = [cb for cb, k in S["count_porp"].items() if k >= n - 2]
    L.append(f"Porpoise region: {len(S['porp_base'])} cells in the base model; {len(porp7)} cells are porpoise in at least {n - 2} of {n} variants: " + (runs_of(porp7) or "none") + ". It sits at aft CG / large bend (lowest static margin, highest trim lift).")
    return L


def html_summary(S, cases, ex, numbers, af):
    f = facts(S, cases, ex, numbers, af)
    L = []
    L.append("<div class='panel'><ul>")
    zeta = f["zeta"]
    L.append(f"<li><b>As-flown case (73.0 mm, 6 deg)</b> {tag('MODEL')}: pulls out after losing {fmt(af['h_first_level_m'], 3)} m, then swoops with a period of about {fmt(af['period_first_s'], 3)} s "
             f"(first flight-path amplitude {fmt(af['amp_first_deg'], 3)} deg, shrinking by a factor {fmt(af['decay_ratio'], 2)} per cycle). Class: <b>{af['class']}</b>. "
             f"Stall (angle of attack past the lift peak) happens in the first swoop {'only' if af['n_stall_after_first'] == 0 else 'and again on ' + str(af['n_stall_after_first']) + ' later cycle(s)'}. "
             f"Time until the flight path stays within 2 deg of its final value: {fmt(af['t_settle_2deg_s'], 3)} s.</li>")
    L.append(f"<li><b>So the model calls it mostly a phugoid</b> (a long speed-for-height swoop with damping ratio {fmt(zeta, 2)}, {tag('MODEL')}), started by the nose-down release, with one stall at the top of the first swoop. "
             f"In the model it is a transient that settles to within 2 deg after about {fmt(af['t_settle_2deg_s'], 2)} s and there is no sustained limit cycle at the flown setting. If the real oscillation lasted much longer than that or did not decay, the model is missing something (for example unsteady stall, hinge flutter or a larger real CG/bend error).</li>")
    L.append(f"<li><b>Predicted phugoid period at the as-flown case</b>: linear {fmt(f['T_lin'], 3)} s (range {fmt(f['T_lin_range'][0], 3)}-{fmt(f['T_lin_range'][1], 3)} s over the sensitivity cases), in the simulation {fmt(af['period_mean_s'], 3)} s, "
             f"textbook pi x sqrt(2) x V / g = {fmt(f['T_form'], 3)} s at trim V {fmt(af['trim_V'], 3)} m/s. The textbook formula is about {100 * (1 - f['T_form'] / f['T_lin']):.0f}% short of the model because it ignores drag. Short period {fmt(f['sp_T'], 2)} s (too fast to see by eye).</li>")
    L.append(f"<li><b>Agreement with the flight</b>: (a) {'pass' if S['agree']['base']['a'] else 'fail'}, (b) {'pass' if S['agree']['base']['b'] else 'fail'}, (c) {'pass' if S['agree']['base']['c'] else 'fail'}. "
             f"The model does not reproduce 'less bend gives a nose dive' at CG 73 mm: in the base model it keeps gliding down to about {next((b for b, a_ in ex['low']['base'] if a_['class'] != 'dive'), float('nan')):g} deg of bend. Section 6 shows which combination of weaker bend and zero-lift moment offset would be needed for a dive at 3-4 deg.</li>")
    L.append(f"<li><b>Smooth region</b>: {len(S['smooth_base'])} of {len(cases) // len(VARIANTS)} cells, forward CG and small bend (more static margin, less trim lift). {len(S['smooth_all'])} cells are smooth in every one of the {len(VARIANTS)} model variants. "
             f"Stall porpoising sits at the aft-CG, large-bend corner ({len(S['porp_base'])} cells), where the trim is close to the lift peak.</li>")
    L.append("</ul><b>What survives the sensitivity variants</b><ul>" + "".join(f"<li>{html.escape(t)}</li>" for t in conclusions(S, cases, ex, af)) + "</ul></div>")
    return "\n".join(L)


def html_modes(S, cases):
    rows = []
    sel = [AS_FLOWN, (72.0, 6.0), (74.0, 6.0), (73.0, 4.0), (73.0, 3.0), (70.0, 6.0), (70.0, 4.0), (76.0, 10.0)]
    for cb in sel:
        try:
            c = find(cases, "base", *cb)
        except StopIteration:
            continue
        m = fmt_modes(c)
        if not m:
            continue
        rows.append((f"{cb[0]:g} / {cb[1]:g}", c["class"], fmt(c["trim_V"], 3), fmt(c["trim_cl"], 3), fmt(c["trim_gamma"], 3), fmt(m["phugoid"]["period"], 3), fmt(m["phugoid"]["zeta"], 3), fmt(c["T_phugoid_formula_s"], 3),
                     fmt(c["period_mean_s"], 3), fmt(m["short"]["period"], 3), fmt(m["short"]["zeta"], 3)))
    return ("<p class='note'>Eigenvalues (numpy) of a central-difference Jacobian of the same 4-state model (vx, vz, theta, q) at the equilibrium glide, " + tag("MODEL") + ". All cases: modes.csv. "
            "Only glides with the trim below the lift peak are listed.</p>" +
            table(["CG (nose) / bend", "class", "trim V m/s", "trim CL", "trim gamma deg", "phugoid T s (linear)", "phugoid zeta", "pi sqrt2 V/g s", "T in sim s", "short-period T s", "short-period zeta"], rows))


def html_sens(S, cases, cgs, bends, ex, names):
    o = []
    rows = []
    for v in names:
        a = S["agree"][v]
        c = S["asflown"][v]
        m = c.get("modes") or {}
        rows.append((v, pass_word(a["a"]), pass_word(a["b"]), pass_word(a["c"]), c["class"], fmt(c["period_first_s"], 3), fmt(c["decay_ratio"], 2), c["n_stall_cycles"], fmt(c["t_settle_2deg_s"], 3),
                     fmt(m["phugoid"]["period"], 3) if "phugoid" in m else "", fmt(c.get("trim_cl"), 3)))
    o.append("<h3>As-flown case (73 mm, 6 deg) and the three agreement parts in every variant</h3>")
    o.append(table(["variant", "(a)", "(b)", "(c)", "as-flown class", "period s", "decay/cycle", "stall cycles", "settle s (2 deg)", "linear phugoid T s", "trim CL"], rows))
    o.append("<h3>Maps in each variant</h3><div class='small-maps'>")
    for v in names:
        o.append(svg_map(cases, v, cgs, bends, small=True, title=v))
    o.append("</div>" + legend())
    best = S["best"]
    o.append("<h3>The best few smooth cases (shortest time to settle in the base model) across variants</h3>")
    rows = []
    for cb in best:
        rows.append((f"{cb[0]:g} / {cb[1]:g}",) + tuple(CLS_LETTER[find(cases, v, *cb)["class"]] for v in names) + (str(S["count_smooth"][cb]),))
    o.append(table(["CG (nose) / bend"] + names + ["variants smooth"], rows))
    o.append(f"<p class='note'>D dive, P porpoise, h persistent phugoid, S smooth. Cells smooth in all {len(names)} variants: {html.escape(runs_of(S['smooth_all']) or 'none')}.</p>")
    o.append("<h3>Extra: how low can the bend go at CG 73 mm before the model dives?</h3>")
    rows = []
    for name, lst in ex["low"].items():
        rows.append((name,) + tuple(CLS_LETTER[a["class"]] for b, a in lst))
    o.append(table(["variant"] + [f"{b:g} deg" for b in EXTRA_BENDS_LOW], rows))
    o.append("<h3>Extra: what would make 3 and 4 deg dive at CG 73 mm? (bend effectiveness x zero-lift Cm offset)</h3>")
    for bend in (3.0, 4.0, 6.0):
        rows = [(f"{e:g}",) + tuple(CLS_LETTER[ex["need"][bend][(e, off)]] for off in EXTRA_OFF) for e in EXTRA_EFF]
        o.append(f"<p class='note'>bend {bend:g} deg (columns: Cm offset)</p>" + table(["effectiveness"] + [f"{off:g}" for off in EXTRA_OFF], rows))
    return "\n".join(o)


def html_defs():
    return f"""<ul>
<li><b>Pulls out</b>: the flight path becomes shallower than -20 deg after release and the glider does not turn through a full rotation. Height lost = height at that moment.</li>
<li><b>Glide (part a)</b>: pulls out, never returns steeper than -80 deg, mean flight path over the last 5 s is shallower than -45 deg.</li>
<li><b>Settled</b>: the flight path stays within a 5 deg band over the last 5 s of the 30 s run.</li>
<li><b>Cycle</b>: from one flight-path maximum after pull-out to the next (extrema need 0.5 deg of movement). Amplitude of a cycle = half of (its maximum minus its following minimum). Decay ratio per cycle = (amplitude of cycle n / amplitude of cycle 1) to the power 1/(n-1), n = up to 6 cycles.</li>
<li><b>Smooth</b>: pulls out, settles, cycle-3 amplitude below 25 % of cycle 1, and no stall after the first cycle.</li>
<li><b>Stall porpoise</b>: alpha (minus the bend's alpha shift) passes the plate table's first lift peak on 2 or more cycles after pull-out. A cycle counts as stalled if alpha passes the peak anywhere between its flight-path maximum and the next one (this includes the swoop top).</li>
<li><b>Persistent phugoid</b>: a glide that is not smooth and not a porpoise. Note that it can still die out; check the decay ratio and settle time.</li>
<li><b>Dive</b>: anything that fails the glide test.</li>
</ul>
<p class="note">Files in this folder: README.md, index.html, cases.csv (base grid), sensitivity_cases.csv, modes.csv, cases_raw.json, avl/ (AVL decks and logs). Re-run: <code>py -B rigid_glider/swiftA_oscillation_study.py</code>.</p>"""


def build_readme(S, cases, ex, numbers, af, cgs, bends, names, spec_rows, traj):
    f = facts(S, cases, ex, numbers, af)
    ag = S["agree"]["base"]
    sym = lambda v: CLS_LETTER[v]
    L = []
    L.append("# Swift A foam 400 mm: pitch-oscillation study (model only, nothing here is a flight result)\n")
    L.append("## Pre-stated agreement check (written 2026-10-21 11:46, BEFORE any grid result was looked at)\n")
    L.append("The model agrees with Kai's 2026-10-20 flight if all three parts pass:\n")
    L.append(PRESTATED + "\n")
    L.append("Each part is reported pass or fail on its own. Failing a part is a finding about the model, not a reason to change the check.\n")
    L.append("(Everything below this line was filled in after the run. The three statements above were not edited.)\n")
    L.append("## Result of the agreement check (base model)\n")
    L.append(f"- (a) **{'PASS' if ag['a'] else 'FAIL'}**: {ag['detail']['a']}")
    L.append(f"- (b) **{'PASS' if ag['b'] else 'FAIL'}**: {ag['detail']['b']}")
    L.append(f"- (c) **{'PASS' if ag['c'] else 'FAIL'}**: {ag['detail']['c']}")
    L.append(f"- Parts passing per variant: " + "; ".join(f"{v}: a {'P' if a['a'] else 'F'}, b {'P' if a['b'] else 'F'}, c {'P' if a['c'] else 'F'}" for v, a in S["agree"].items()) + "\n")
    L.append("Note on (c): the pre-stated metric is the amplitude of the last full cycle in 30 s, which for a decaying oscillation is a small number (about 1-2 deg) and is very sensitive to the decay rate; the first-cycle amplitudes of 72, 73 and 74 mm are "
             + ", ".join(f"{find(cases, 'base', cg, 6.0)['amp_first_deg']:.0f} deg" for cg in (72.0, 73.0, 74.0)) + ". The verdict above uses the pre-stated metric.\n")
    L.append("## What the model says the oscillation is\n")
    L.append(f"- As flown (CG 73.0 mm, 6 deg) [MODEL]: pulls out after losing {fmt(af['h_first_level_m'], 3)} m; class **{af['class']}**; first-cycle period {fmt(af['period_first_s'], 3)} s (mean of the first cycles {fmt(af['period_mean_s'], 3)} s); first-cycle flight-path amplitude {fmt(af['amp_first_deg'], 3)} deg; "
             f"amplitude ratio {fmt(af['decay_ratio'], 2)} per cycle ({af['decay_word']}); time to stay within 2 deg of the final path {fmt(af['t_settle_2deg_s'], 3)} s; alpha passes the lift peak on {af['n_stall_cycles']} cycle(s) "
             f"({'the first swoop only' if af['n_stall_after_first'] == 0 and af['stall_in_first_cycle'] else 'see cases.csv'}).")
    L.append("- Reading: a lightly damped **phugoid** (linear damping ratio " + fmt(f['zeta'], 2) + ") excited by the nose-down release, with a stall at the top of the first swoop only; it is not a sustained stall porpoise at the flown setting. "
             "'Persistent phugoid' is the stated-threshold label (decay ratio " + fmt(af['decay_ratio'], 2) + " per cycle is worse than the 0.5 that 'smooth' needs); the oscillation still dies out in the model in about " + fmt(af['t_settle_2deg_s'], 2) + " s.")
    L.append(f"- Predicted phugoid period at the as-flown case, for checking against video [MODEL]: linear {fmt(f['T_lin'], 3)} s (sensitivity range {fmt(f['T_lin_range'][0], 3)}-{fmt(f['T_lin_range'][1], 3)} s), simulation {fmt(af['period_mean_s'], 3)} s "
             f"(range {fmt(f['T_sim_range'][0], 3)}-{fmt(f['T_sim_range'][1], 3)} s over the variants), trim speed {fmt(af['trim_V'], 3)} m/s, trim CL {fmt(af['trim_cl'], 3)}, glide angle {fmt(af['trim_gamma'], 3)} deg. "
             f"Textbook T = pi sqrt(2) V / g = {fmt(f['T_form'], 3)} s ({100 * (1 - f['T_form'] / f['T_lin']):.0f}% shorter than the model because it ignores drag). Short-period period {fmt(f['sp_T'], 2)} s.")
    L.append("- A mix cannot be excluded: a stall in the first swoop is predicted, and the aft-CG / high-bend corner of the map is a true porpoise region.\n")
    L.append("## Conclusions that survive the sensitivity variants (data-driven)\n")
    L += [f"- {t}" for t in conclusions(S, cases, ex, af)]
    L.append("")
    L.append("## Map (base model); rows = bend (deg), columns = CG mm behind the nose tip\n")
    L.append("D dive, P stall porpoise, h persistent phugoid, S smooth (decay ratio per cycle in cases.csv and the HTML map)\n")
    L.append("```\nbend\\CG " + " ".join(f"{c:>3g}" for c in cgs))
    for b in reversed(bends):
        L.append(f"{b:>7g}  " + " ".join(f"{sym(find(cases, 'base', c, b)['class']):>3}" for c in cgs))
    L.append("```\n")
    L.append(f"- Smooth cells (base): {runs_of(S['smooth_base'])}.")
    L.append(f"- Smooth in all {len(names)} variants: {runs_of(S['smooth_all']) or 'none'}.")
    L.append(f"- Porpoise cells (base): {runs_of(S['porp_base'])}.")
    L.append(f"- Porpoise in all variants: {runs_of(S['porp_all']) or 'none'}.")
    L.append(f"- Dive in the base model: {runs_of(S['dive_base']) or 'none'}. Dive in at least one variant: {runs_of(S['dive_any']) or 'none'}.\n")
    L.append("## Sensitivity (what survives)\n")
    L.append("| variant | a | b | c | as-flown class | period s | decay/cycle | stall cycles | settle s | linear phugoid T s |\n|---|---|---|---|---|---|---|---|---|---|")
    for v in names:
        a, c = S["agree"][v], S["asflown"][v]
        m = c.get("modes") or {}
        L.append(f"| {v} | {'P' if a['a'] else 'F'} | {'P' if a['b'] else 'F'} | {'P' if a['c'] else 'F'} | {c['class']} | {fmt(c['period_first_s'], 3)} | {fmt(c['decay_ratio'], 2)} | {c['n_stall_cycles']} | {fmt(c['t_settle_2deg_s'], 3)} | {fmt(m['phugoid']['period'], 3) if 'phugoid' in m else ''} |")
    L.append("")
    L.append("Best few smooth cases (shortest settle time, base): " + ", ".join(cell_name(cb) for cb in S["best"]) + ". Class per variant (" + ", ".join(names) + "):")
    for cb in S["best"]:
        L.append(f"- {cell_name(cb)}: " + " ".join(sym(find(cases, v, *cb)["class"]) for v in names))
    L.append("")
    L.append("Bend scan at CG 73.0 mm below 3 deg (class per variant; columns " + ", ".join(f"{b:g}" for b in EXTRA_BENDS_LOW) + " deg):")
    for name, lst in ex["low"].items():
        L.append(f"- {name}: " + " ".join(sym(a["class"]) for b, a in lst))
    L.append("")
    L.append("What would turn 3 and 4 deg at CG 73.0 mm into a dive (rows: bend effectiveness; columns: Cm offset " + ", ".join(f"{o:g}" for o in EXTRA_OFF) + "):")
    for bend in (3.0, 4.0):
        L.append(f"- bend {bend:g} deg:")
        for e in EXTRA_EFF:
            L.append(f"  - eff {e:g}: " + " ".join(sym(ex["need"][bend][(e, o)]) for o in EXTRA_OFF))
    L.append("")
    L.append("## Specimen (as modelled), with source tags\n")
    L.append("| quantity | value | source |\n|---|---|---|")
    for a, b, c in spec_rows:
        L.append(f"| {a} | {b} | {c} |")
    L.append("")
    L.append("## Definitions and thresholds (chosen by the study, stated here)\n")
    L.append("- Pulls out: flight path shallower than -20 deg after release (and no full rotation). Height lost to first level-off = height at that moment.\n"
             "- Settled: flight path stays within a 5 deg band over the last 5 s of the 30 s run.\n"
             "- Cycle: flight-path maximum to the next one (extrema need 0.5 deg); cycle amplitude = half of (maximum minus the following minimum); decay ratio per cycle = (A_n / A_1)^(1/(n-1)) with n up to 6; shrinks below 0.8, grows above 1.1, else persists.\n"
             "- Smooth: pulls out, settles, cycle-3 amplitude < 25 % of cycle 1, no stall after the first cycle.\n"
             "- Stall porpoise: alpha minus the bend's alpha shift passes the plate table's first lift peak on 2 or more cycles after pull-out (a cycle = flight-path maximum to the next; the swoop top lies inside it).\n"
             "- Persistent phugoid: a glide that is neither smooth nor a porpoise (may still decay slowly).\n"
             "- Dive: fails the glide test of part (a).\n"
             "- Linear modes: numpy eigenvalues of a central-difference Jacobian of the 4-state model (vx, vz, theta, q) at the equilibrium glide; phugoid = the lower-frequency complex pair, short period = the higher one.\n")
    L.append("## Assumptions and limits\n")
    L.append("- Moving the CG is modelled as changing the ballast mass at the fixed -12.5 mm hole (rail fixed as built); the all-up mass therefore changes with CG (about 9.6-10.4 g over the grid). Pitch inertia is recomputed with it.\n"
             "- Aerodynamics: Ananda plate table (AR 5 taper 1 run, lowest Re) for the whole Swift outline, AC at the AVL neutral point; the carry-over to the tapered swept foam wing and to the flight Reynolds numbers (" + S["re"] + ") is ASSUMED. The simulation is quasi-steady (no added mass, no unsteady stall, no quad downwash, no wind).\n"
             "- The tail bend is applied as AVL's lift and moment change per degree (linear, from 0 and 5 deg runs); the real foam hinge bend and its springback were not measured (the effectiveness 75 % / 50 % variants stand in for that).\n"
             "- Lateral behaviour (the turn Kai saw) is not modelled.\n"
             "- No flight data beyond Kai's words; no period or amplitude of the real oscillation has been measured.\n")
    L.append("## Files\n")
    L.append("index.html (dark-mode page), cases.csv (base grid), sensitivity_cases.csv, modes.csv, cases_raw.json, avl/. Re-run: `py -B rigid_glider/swiftA_oscillation_study.py`; tests: `py -B rigid_glider/test_swiftA_oscillation_study.py`.\n")
    return "\n".join(L)


# =============================================================================================== main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(OUT_DEFAULT))
    ap.add_argument("--coarse", action="store_true", help="CG steps of 2 mm and bend steps of 2 deg (test only)")
    ap.add_argument("--jobs", type=int, default=max(1, min(8, (os.cpu_count() or 2) - 1)))
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    print("AVL ...")
    avl = build_avl_numbers(out)
    numbers = numbers_from_avl(avl)
    cgs, bends = CG_NOSE, BENDS
    if args.coarse:
        cgs, bends = sorted(set(CG_NOSE[::2] + [72, 73, 74])), sorted(set(BENDS[::2] + [3, 4, 6]))
    print("grid ...")
    cases = run_grid(numbers, cgs, bends, args.jobs)
    (out / "cases_raw.json").write_text(json.dumps(cases, indent=1, default=float))
    print(f"computed in {time.time() - t0:.0f} s")
    write_all(out, numbers, cases, cgs, bends)


if __name__ == "__main__":
    main()
