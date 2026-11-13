"""Straight-and-smooth phase 1: lateral / turn side of the B45 flat wing and the Swift A foam 400 glider.  MODEL study (inviscid AVL 3.52), nothing here is a flight result.
Standard library only.   py -B rigid_glider/straight_smooth_v1.py [--out DIR] [--only NAME[,NAME]] [--quick]
Writes only under results/runs/rigid_glider/straight_smooth_v1/ (README.md results section is written by write_readme_results(), the pre-stated section is never touched).

Imports (read-only): b45_centre_flaps (B45 deck pieces), flat_test, bird_outline(_v2), bird_outline_v4_swift (Swift A geometry), fin_kit_v4 (centre fin, system-matrix parsing, eigenvalues).
Axes: AVL x aft, y starboard, z up.  Cl', Cn' = stability-axis roll / yaw moment coefficients (positive: right wing down / nose right).  All lengths in this file are mm unless a name says _m.
"""
from __future__ import annotations
import sys
sys.dont_write_bytecode = True
import argparse
import csv
import json
import math
import pickle
import re
import subprocess
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import flat_test as ft                    # read-only imports
import b45_centre_flaps as b45
import bird_outline as bo
import bird_outline_v2 as v2
import bird_outline_v4_swift as s4
import fin_kit_v4 as fk
from avl import _field

ROOT = ft.ROOT
OUT = ROOT / "results/runs/rigid_glider/straight_smooth_v1"
EXE = ft.AVL_EXE
RHO = 1.225
GRAV = 9.81
DIHEDRALS = (0.0, 3.0, 6.0, 9.0, 12.0)
ASYM_DEG = 1.0                           # size of every reference asymmetry (ASSUMED)
AMP = 10.0                               # AVL runs apply AMP x the asymmetry and divide the response by AMP: the .ft files print Cl and Cn to 5 decimals, coarse for a 1 deg signal (AVL is linear in these small angles)
PATH_M = 100.0                           # glide path length for the heading-change figure
HEADING_LIMIT_DEG = 90.0
SPIRAL_T2_MIN_S = 15.0
DR_ZETA_MIN = 0.10
CL_TOL = 0.02
SM_TOL_PCT = 1.0
TAB_HINGE = 0.70                         # trim tab hinge at this fraction of the fin chord (ASSUMED)
TAB_PROBE = 6.0                          # deg, central-difference probe of the tab and fin-toe response
SPANS = (400.0, 250.0)

# ---- B45 baseline numbers
B45_CG = 66.0                            # mm aft of root LE, flight_day_cg_page cg_t
B45_FLAP_UP = 10.0
B45_BALLAST_STATION = -37.5              # nose hole used by flat_test for the middle setting (geometry.json ballast_holes_x_mm)
B45_MASS = dict(plate_g=10.666666666666666, fins_g=0.8533333333333333, stick_g=4.055452148148133,
                stick_x0=-47.40740740740741, stick_x1=118.51851851851852, stick_z=-6.0)     # geometry.json B45 thickness_table (ASSUMED 300 g/m2); stick z ASSUMED
# ---- Swift A baseline numbers
SW_SPAN = 400.0
SW_CG = 37.0                             # mm aft of root LE (= 73.0 from nose tip, nose tip 36.0 mm ahead of root LE)
SW_BEND = 6.0
SW_NOSE_AHEAD = 36.0
SW_CENTRE_FIN_PCT = 2.0                  # fin_kit_v4 chosen_sizes.csv, "centre_best" row, foam 400 mm size

SIGNS = {}                               # calibrated at run time (fin toe / tab sign), see calibrate_*


# =============================================================================================== deck pieces
def sec(x, y, z, c, ainc=0.0, nspan=None, controls=()):
    return dict(x=x, y=y, z=z, c=c, ainc=ainc, nspan=nspan, controls=list(controls))


def sec_text(s):
    m = lambda v: v / 1000.0
    t = f"SECTION\n{m(s['x']):.9f} {m(s['y']):.9f} {m(s['z']):.9f} {m(s['c']):.9f} {s['ainc']:.6f}" + ("" if s["nspan"] is None else f" {s['nspan']} 1.0") + "\nNACA\n0000\n"
    for name, gain, xh in s["controls"]:
        t += f"CONTROL\n{name} {gain:.4f} {xh:.9f} 0 0 0 1.0\n"
    return t


def mirror(secs):
    """Left-hand copy of a right-hand section list: y -> -y, order reversed so y still increases (same rule YDUPLICATE uses).  A section's nspan belongs to the interval that
    starts at it, so after the reversal each interval keeps the nspan of the original interval."""
    n = len(secs)
    out = []
    for i, s in enumerate(reversed(secs)):
        d = dict(s)
        d["y"] = -s["y"]
        d["nspan"] = secs[n - 2 - i]["nspan"] if i < n - 1 else secs[n - 1]["nspan"]
        d["controls"] = [(nm[:-1] + "L" if nm.endswith("R") else nm, gn, xh) for nm, gn, xh in s["controls"]]
        out.append(d)
    return out


def surface_text(name, header, secs, comp):
    """comp: AVL COMPONENT index; both halves of one physical surface must share it (otherwise the halves are not shielded as one surface and CLa / Xnp are wrong)."""
    return f"SURFACE\n{name}\n{header}\nCOMPONENT\n{comp}\n" + "".join(sec_text(s) for s in secs)


def shift_ainc(secs, d):
    return [dict(s, ainc=s["ainc"] + d) for s in secs]


def rot(y0, z0, gam_deg):
    """Rigid rotation of a point (y0 along the flat half wing, z0 on it) about the root x axis by the dihedral angle."""
    c, s = math.cos(math.radians(gam_deg)), math.sin(math.radians(gam_deg))
    return y0 * c - z0 * s, y0 * s + z0 * c


# =============================================================================================== B45
def b45_fin_dims(fin_scale):
    return dict(h=b45.FIN_H * fin_scale, c=b45.G["ct"])


def b45_halves(gam, fin_scale=1.0, wing_ainc=0.0, fin_toe=0.0, tab=None):
    """Right-hand wing and fin section lists for the B45 deck (b45_centre_flaps.deck geometry, flaps as controls flapR / tab names ending with R)."""
    G, half = b45.G, b45.G["span"] / 2
    y1, y2 = b45.LOCATIONS["nominal"]
    n1 = max(2, round(y1 / 5))
    n3 = round((half - y2) / 8)
    wing = []
    for y0, nsp, flap in ((0.0, n1, False), (y1, 8, True), (y2, n3, True), (half, 0, False)):
        y, z = rot(y0, 0.0, gam)
        cs = [("flapR", b45.GAIN, 1 - b45.FLAP_CHORD_MM / b45.chord(y0))] if flap else []
        wing.append(sec(b45.le_x(y0), y, z, b45.chord(y0), wing_ainc, nsp, cs))
    fd = b45_fin_dims(fin_scale)
    yt, zt = rot(half, 0.0, gam)
    fin = []
    for dz in (fd["h"] / 2, -fd["h"] / 2):
        cs = [("tabR", 1.0, TAB_HINGE)] if tab else []
        fin.append(sec(G["tipx"], yt, zt + dz, fd["c"], fin_toe, None, cs))
    return wing, fin


def b45_deck(cg, zcg, gam, fin_scale=1.0, wing_inc=(0.0, 0.0), fin_toe=(0.0, 0.0), tab=False, name="b45ss"):
    """Explicit two-half deck (no YDUPLICATE) so the halves can differ.  Controls: flapR (D1), flapL (D2) [, tabR (D3) on the right tip fin]."""
    G = b45.G
    m = lambda v: v / 1000.0
    s = f"{name}\n0.0\n0 0 0.0\n{G['area'] * 1e-6:.9f} {m(G['mac']):.9f} {m(G['span']):.9f}\n{m(cg):.9f} 0.0 {m(zcg):.9f}\n"
    wr, fr = b45_halves(gam, fin_scale, wing_inc[0], fin_toe[0], tab)
    wl, fl = b45_halves(gam, fin_scale, wing_inc[1], fin_toe[1], False)
    wl = mirror(wl)
    fl = mirror(fl)
    s += surface_text("WingR", f"{ft.AVL_NCHORD} 1.0", wr, 1)
    s += surface_text("WingL", f"{ft.AVL_NCHORD} 1.0", wl, 1)
    s += surface_text("FinR", "5 1.0 8 -2.0", fr, 3)
    s += surface_text("FinL", "5 1.0 8 -2.0", fl, 3)
    return s


# =============================================================================================== Swift A
_SW = {}


def sw_geo():
    if "g" not in _SW:
        g = s4.build_geometry4(s4.GLIDE)
        _SW["g"] = g
        _SW["fin"] = fk.fin_geometry(g, "centre", SW_CENTRE_FIN_PCT / 100.0)
    return _SW["g"], _SW["fin"]


def sw_halves(gam, bend, wing_ainc=0.0):
    """Right-hand wing and tail sections (mm) of the Swift A deck (bird_outline_v2.avl_deck2 geometry).  Wing rotated by the dihedral about the root, tail flat."""
    g, _ = sw_geo()
    k = SW_SPAN
    wing = []
    for y, x, z, c in bo.wing_sections(g):
        yy, zz = rot(y * k, z * k, gam)
        wing.append(sec(x * k, yy, zz, c * k, wing_ainc))
    tail = [sec(x * k, y * k, z * k, c * k, -bend, None, [("bendR", -1.0, 0.0)]) for y, x, z, c in v2.tail_sections2(g)]
    return wing, tail


def sw_deck(cg, zcg, gam, centre_fin=False, wing_inc=(0.0, 0.0), bends=(SW_BEND, SW_BEND), fin_toe=0.0, tab=False, name="swss"):
    g, fin = sw_geo()
    k = SW_SPAN
    m = lambda v: v / 1000.0
    s = f"{name}\n0.0\n0 0 0.0\n{g['S'] * k * k * 1e-6:.9f} {m(g['mac'] * k):.9f} {m(k):.9f}\n{m(cg):.9f} 0.0 {m(zcg):.9f}\n"
    wr, tr = sw_halves(gam, bends[0], wing_inc[0])
    wl, tl = sw_halves(gam, bends[1], wing_inc[1])
    wl, tl = mirror(wl), mirror(tl)
    hdr_w = f"{v2.AVL_NCHORD} 1.0 {v2.AVL_NSPAN} {v2.AVL_SPACE}"
    s += surface_text("WingR", hdr_w, wr, 1) + surface_text("WingL", hdr_w, wl, 1)
    s += surface_text("TailR", "6 1.0 12 -2.0", tr, 2) + surface_text("TailL", "6 1.0 12 -2.0", tl, 2)
    if centre_fin:
        secs = []
        for z in (0.0, fin["h"]):
            secs.append(sec(fin["x_le"] * k, 0.0, z * k, fin["c"] * k, fin_toe, None, [("tabR", 1.0, TAB_HINGE)] if tab else []))
        s += surface_text("CentreFin", "5 1.0 8 -2.0", secs, 5)
    return s


# =============================================================================================== mass model (ASSUMED masses, point-sampled plate)
def item(m_g, x, y, z, ixx=0.0, iyy=0.0, izz=0.0):
    return dict(m=m_g, x=x, y=y, z=z, ixx=ixx, iyy=iyy, izz=izz)       # self inertias g mm^2


def assemble(items):
    M = sum(i["m"] for i in items)
    cg = [sum(i["m"] * i[a] for i in items) / M for a in "xyz"]
    ixx = sum(i["m"] * ((i["y"] - cg[1]) ** 2 + (i["z"] - cg[2]) ** 2) + i["ixx"] for i in items)
    iyy = sum(i["m"] * ((i["x"] - cg[0]) ** 2 + (i["z"] - cg[2]) ** 2) + i["iyy"] for i in items)
    izz = sum(i["m"] * ((i["x"] - cg[0]) ** 2 + (i["y"] - cg[1]) ** 2) + i["izz"] for i in items)
    jxz = sum(i["m"] * (i["x"] - cg[0]) * (i["z"] - cg[2]) for i in items)          # product of inertia integral(x z dm), AVL axes x aft z up (same value in x-forward z-down axes)
    return dict(M_g=M, cg=cg, I_kgm2=(ixx * 1e-3 * 1e-6, iyy * 1e-3 * 1e-6, izz * 1e-3 * 1e-6), jxz_kgm2=jxz * 1e-3 * 1e-6)


def mass_file(asm):
    """One AVL mass line at the CG (Ixz = 0, ASSUMED)."""
    M = asm["M_g"] * 1e-3
    x, y, z = (c * 1e-3 for c in asm["cg"])
    ixx, iyy, izz = asm["I_kgm2"]
    return (f"Lunit = 1.0 m\nMunit = 1.0 kg\nTunit = 1.0 s\ng = {GRAV}\nrho = {RHO}\n# mass x y z Ixx Iyy Izz\n"
            f"{M:.9e} {x:.9f} {y:.9f} {z:.9f} {ixx:.6e} {iyy:.6e} {izz:.6e}\n")


def b45_mass(cg_t, gam, fin_scale):
    G, half = b45.G, b45.G["span"] / 2
    B = B45_MASS
    items = []
    ny, nx = 40, 8
    dy = half / ny
    area_half = G["area"] / 2
    for j in range(ny):
        y0 = (j + 0.5) * dy
        c = b45.chord(y0)
        for i in range(nx):
            x = b45.le_x(y0) + (i + 0.5) * c / nx
            me = B["plate_g"] * (dy * c / nx) / area_half / 2
            for s in (1, -1):
                yy, zz = rot(y0, 0.0, gam)
                items.append(item(me, x, s * yy, zz))
    fd = b45_fin_dims(fin_scale)
    yt, zt = rot(half, 0.0, gam)
    mf = B["fins_g"] / 2 * fin_scale
    for s in (1, -1):
        items.append(item(mf, G["tipx"] + fd["c"] / 2, s * yt, zt, mf * fd["h"] ** 2 / 12, mf * (fd["h"] ** 2 + fd["c"] ** 2) / 12, mf * fd["c"] ** 2 / 12))
    L = B["stick_x1"] - B["stick_x0"]
    items.append(item(B["stick_g"], 34.63788855896894, 0.0, B["stick_z"], 0.0, B["stick_g"] * L ** 2 / 12, B["stick_g"] * L ** 2 / 12))
    M0 = sum(i["m"] for i in items)
    cg0 = sum(i["m"] * i["x"] for i in items) / M0
    mb = M0 * (cg_t - cg0) / (B45_BALLAST_STATION - cg_t)
    items.append(item(mb, B45_BALLAST_STATION, 0.0, B["stick_z"]))
    a = assemble(items)
    a.update(M0_g=M0, cg0=cg0, ballast_g=mb)
    return a


def sw_pts():
    if "pts" not in _SW:
        g, _ = sw_geo()
        k = SW_SPAN
        poly = [(x * k, y * k) for x, y in g["full_outline"]]
        xs, ys = [p[0] for p in poly], [p[1] for p in poly]
        n = 120
        ws = [(y * k, x * k, (x + c) * k) for y, x, z, c in bo.wing_sections(g)]
        def inwing(x, y):
            a = abs(y)
            for (y0, l0, t0), (y1, l1, t1) in zip(ws, ws[1:]):
                if y0 <= a <= y1:
                    f = (a - y0) / (y1 - y0) if y1 > y0 else 0.0
                    return l0 + f * (l1 - l0) - 1e-9 <= x <= t0 + f * (t1 - t0) + 1e-9
            return False
        pts = []
        for i in range(n):
            x = min(xs) + (i + 0.5) * (max(xs) - min(xs)) / n
            for j in range(n):
                y = min(ys) + (j + 0.5) * (max(ys) - min(ys)) / n
                if fk._inside(poly, x, y):
                    pts.append((x, y, inwing(x, y)))
        _SW["pts"] = pts
    return _SW["pts"]


def sw_mass(cg_t, gam, centre_fin):
    g, fin = sw_geo()
    n = fk.base_numbers(g, "foam", SW_CG / SW_SPAN, SW_BEND)
    pts = sw_pts()
    me = n["m_sheet"] / len(pts)
    items = []
    for x, y, isw in pts:
        if isw:
            yy, zz = rot(abs(y), 0.0, gam)
            items.append(item(me, x, math.copysign(yy, y), zz))
        else:
            items.append(item(me, x, y, 0.0))
    items.append(item(n["rail"]["mass_g"], n["rail"]["cx"], 0.0, 0.0))
    if centre_fin:
        mf = fk.fin_mass(fin, "foam")
        items.append(item(mf, (fin["x_te"] - fin["c"] / 2) * SW_SPAN, 0.0, fin["h"] * SW_SPAN / 2, mf * (fin["h"] * SW_SPAN) ** 2 / 12, 0.0, mf * (fin["c"] * SW_SPAN) ** 2 / 12))
    M0 = sum(i["m"] for i in items)
    cg0 = sum(i["m"] * i["x"] for i in items) / M0
    mb = M0 * (cg_t - cg0) / (n["station"] - cg_t)
    items.append(item(mb, n["station"], 0.0, 0.0))
    a = assemble(items)
    a.update(M0_g=M0, cg0=cg0, ballast_g=mb, station=n["station"])
    return a


# =============================================================================================== AVL session
def avl_session(workdir: Path, name, deck, stages, mass=None, V=None, sysfile=None, mode_extra=()):
    """stages = list of (oper_cmds, tag): the oper commands are sent, then X, then FT <tag>.ft and ST <tag>.st.  If mass/V given, MODE writes the system matrix of the LAST stage."""
    workdir.mkdir(parents=True, exist_ok=True)
    for f in workdir.glob("*.ft"):
        f.unlink()
    for f in workdir.glob("*.st"):
        f.unlink()
    (workdir / "sys.out").unlink(missing_ok=True)
    (workdir / f"{name}.mass").unlink(missing_ok=True)
    (workdir / f"{name}.avl").write_text(deck, encoding="ascii")
    cmds = ["PLOP", "G", "", "OPER"]
    for oper, tag in stages:
        cmds += list(oper) + ["X", "FT", f"{tag}.ft", "ST", f"{tag}.st"]
    cmds += [""]
    if mass:
        (workdir / f"{name}.mass").write_text(mass, encoding="ascii")
        cmds += ["MODE", "M", f"V {V:.5f}", *mode_extra, "", "S", "sys.out", "", ""]
    cmds += ["QUIT"]
    script = "\n".join(cmds) + "\n"
    (workdir / "commands.in").write_text(script, encoding="ascii")
    r = subprocess.run([str(EXE), f"{name}.avl"], input=script, text=True, cwd=workdir, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=300)
    (workdir / "avl.log").write_text(r.stdout, encoding="utf-8", errors="replace")
    out = {}
    for _, tag in stages:
        ftp, stp = workdir / f"{tag}.ft", workdir / f"{tag}.st"
        if not (ftp.is_file() and stp.is_file()):
            raise RuntimeError(f"AVL gave no output for {tag}; see {workdir / 'avl.log'}")
        out[tag] = parse_outputs(ftp.read_text(errors="replace"), stp.read_text(errors="replace"))
    return out


FT_KEYS = {"CL": "CLtot", "CD": "CDtot", "CY": "CYtot", "Cm": "Cmtot", "Cl": "Cl'tot", "Cn": "Cn'tot", "alpha": "Alpha"}
ST_KEYS = ["CLa", "Cma", "Xnp", "CYb", "Clb", "Cnb", "CYp", "Clp", "Cnp", "CYr", "Clr", "Cnr", "Cmq"]


def parse_outputs(ft_t, st_t):
    d = {k: _field(ft_t, v) for k, v in FT_KEYS.items()}
    for k in ST_KEYS:
        d[k] = _field(st_t, k)
    for c in (1, 2, 3):
        for q in ("CL", "CY", "Cl", "Cm", "Cn"):
            try:
                d[f"{q}d{c}"] = _field(st_t, f"{q}d{c:02d}")
            except ValueError:
                pass
    return d


# =============================================================================================== linear algebra, modes, closed form
def parse_sysmat_full(path):
    """12 x (12 + n_controls) numbers of AVL's system matrix file (states u w q theta v p r phi x y z psi; control columns per DEGREE, checked in the tests)."""
    rows = []
    for line in Path(path).read_text().splitlines():
        q = line.replace("|", " ").split()
        try:
            v = [float(x) for x in q]
        except ValueError:
            continue
        if len(v) >= 12:
            rows.append(v)
    if len(rows) < 12:
        raise RuntimeError("system matrix not parsed")
    return rows[:12]


def lat_blocks(rows):
    idx = (4, 5, 6, 7)
    A = [[rows[i][j] for j in idx] for i in idx]
    B = [[rows[i][12 + c] for c in range(len(rows[0]) - 12)] for i in idx]
    return A, B


def solve(A, b):
    n = len(A)
    M = [list(A[i]) + [b[i]] for i in range(n)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(M[r][c]))
        M[c], M[p] = M[p], M[c]
        for r in range(n):
            if r != c:
                f = M[r][c] / M[c][c]
                M[r] = [a - f * bb for a, bb in zip(M[r], M[c])]
    return [M[i][n] / M[i][i] for i in range(n)]


def rhat_closed(clb, cnb, clr, cnr, dcl, dcn):
    """Steady turn rate r b / 2V for a constant extra roll / yaw moment coefficient (p = 0): README, from Clb*beta + Clr*rhat + dCl = 0 and Cnb*beta + Cnr*rhat + dCn = 0."""
    den = clb * cnr - clr * cnb
    return (cnb * dcl - clb * dcn) / den, den


def beta_steady(clb, clr, dcl, rh):
    return -(dcl + clr * rh) / clb


def rhat_body(clb, cnb, clr, cnr, clp, cnp, dcl, dcn, alpha_deg):
    """Same balance but with the roll rate of the BODY-axis steady turn: AVL's system matrix is in body axes with the pitch attitude 0, so p_body = 0 in the steady turn,
    which in stability axes is p' = r' tan(alpha).  Returns (r_hat stability-axis, r_hat heading rate psi_dot b/2V, D_body, beta)."""
    ta = math.tan(math.radians(alpha_deg))
    clr_e, cnr_e = clr + ta * clp, cnr + ta * cnp
    rs, den = rhat_closed(clb, cnb, clr_e, cnr_e, dcl, dcn)
    beta = -(dcl + clr_e * rs) / clb
    return rs, rs / math.cos(math.radians(alpha_deg)), den, beta


def integrate_lateral(A, B, u, T, dt=0.01):
    """RK4 of x' = A x + B u (constant u, degrees), x = (v, p, r, phi), from rest.  Returns the trajectory (every second) and the final state."""
    n = 4
    bu = [sum(B[i][c] * u[c] for c in range(len(u))) for i in range(n)]
    f = lambda x: [sum(A[i][j] * x[j] for j in range(n)) + bu[i] for i in range(n)]
    x = [0.0] * n
    traj = []
    every = int(round(1.0 / dt))
    for k in range(int(T / dt) + 1):
        if k % every == 0:
            traj.append((k * dt, list(x)))
        k1 = f(x)
        k2 = f([x[i] + 0.5 * dt * k1[i] for i in range(n)])
        k3 = f([x[i] + 0.5 * dt * k2[i] for i in range(n)])
        k4 = f([x[i] + dt * k3[i] for i in range(n)])
        x = [x[i] + dt / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) for i in range(n)]
    return traj, x


def heading_deg(rhat, span_mm, path_m=PATH_M):
    return math.degrees(2 * rhat * path_m / (span_mm / 1000.0))


def radius_m(rhat, span_mm):
    return float("inf") if rhat == 0 else (span_mm / 1000.0) / (2 * abs(rhat))


# =============================================================================================== one configuration
def make_cfg(kind, gam, fin):
    if kind == "b45":
        nm = f"B45_fin{fin:g}_d{gam:g}"
    else:
        nm = f"SwiftA_{'centrefin' if fin else 'nofin'}_d{gam:g}"
    return dict(kind=kind, gam=gam, fin=fin, name=nm)


def cfg_list():
    L = [make_cfg("b45", g, fs) for g in DIHEDRALS for fs in (1.0, 1.5)]
    L += [make_cfg("sw", g, cf) for g in DIHEDRALS for cf in (False, True)]
    return L


def kind_consts(kind):
    if kind == "b45":
        return dict(mac=b45.G["mac"], S_m2=b45.G["area"] * 1e-6, b_mm=b45.G["span"], cg_nom=B45_CG)
    g, _ = sw_geo()
    return dict(mac=g["mac"] * SW_SPAN, S_m2=g["S"] * (SW_SPAN * 1e-3) ** 2, b_mm=SW_SPAN, cg_nom=SW_CG)


def deck_for(cfg, cg, zcg, **kw):
    if cfg["kind"] == "b45":
        return b45_deck(cg, zcg, cfg["gam"], fin_scale=cfg["fin"], **kw)
    return sw_deck(cg, zcg, cfg["gam"], centre_fin=cfg["fin"], **kw)


def mass_for(cfg, cg):
    return b45_mass(cg, cfg["gam"], cfg["fin"]) if cfg["kind"] == "b45" else sw_mass(cg, cfg["gam"], cfg["fin"])


def oper_nominal(cfg):
    """Nominal control settings, always sent explicitly (AVL keeps control values between runs of one session): B45 flaps 10 deg UP; Swift A extra tail-half bend 0."""
    return [f"D1 D1 {B45_FLAP_UP}", f"D2 D2 {B45_FLAP_UP}"] if cfg["kind"] == "b45" else ["D1 D1 0", "D2 D2 0"]


def asym_entry(a, b0, sign=1.0):
    """Response per ASYM_DEG, from a run at AMP x ASYM_DEG."""
    return dict(dCl=sign * (a["Cl"] - b0["Cl"]) / AMP, dCn=sign * (a["Cn"] - b0["Cn"]) / AMP, dCY=sign * (a["CY"] - b0["CY"]) / AMP, dCL=(a["CL"] - b0["CL"]) / AMP, dCm=(a["Cm"] - b0["Cm"]) / AMP)


def evaluate(cfg, base, work: Path):
    kc = kind_consts(cfg["kind"])
    mac = kc["mac"]
    wd = work / cfg["name"]
    cg_ref = kc["cg_nom"]
    alphas = (0.0, 4.0)
    ref = avl_session(wd / "ref", "ss", deck_for(cfg, cg_ref, 0.0), [(oper_nominal(cfg) + [f"A A {a}"], f"ref{i}") for i, a in enumerate(alphas)])
    xnp = ref["ref1"]["Xnp"] * 1000.0
    cg = cg_ref if base is None else xnp - base["sm_pct"] / 100.0 * mac
    cl0, cla = b45.linfit(alphas, [ref[f"ref{i}"]["CL"] for i in range(2)])
    cm0r, cmar = b45.linfit(alphas, [ref[f"ref{i}"]["Cm"] for i in range(2)])
    cm0 = cm0r + cl0 * (cg - cg_ref) / mac                # Cm about the new CG (x aft): Cm_ref + CL (cg - cg_ref)/MAC
    cma = cmar + cla * (cg - cg_ref) / mac
    alpha = -cm0 / cma
    asm = mass_for(cfg, cg)
    zcg = asm["cg"][2]
    deck = deck_for(cfg, cg, zcg)
    for _ in range(3):
        out = avl_session(wd / "trim", "ss", deck, [(oper_nominal(cfg) + [f"A A {alpha:.6f}"], "t0")])["t0"]
        if abs(out["Cm"]) < 3e-4:
            break
        alpha -= out["Cm"] / (out["Cma"] * math.pi / 180.0)
    cl_trim = out["CL"]
    V = math.sqrt(2 * asm["M_g"] * 1e-3 * GRAV / (RHO * kc["S_m2"] * cl_trim))
    oper = oper_nominal(cfg) + [f"A A {alpha:.6f}"]
    base_ctl = B45_FLAP_UP if cfg["kind"] == "b45" else 0.0
    stages = [([f"D1 D1 {base_ctl + AMP * ASYM_DEG / 2}", f"D2 D2 {base_ctl - AMP * ASYM_DEG / 2}", f"A A {alpha:.6f}"], "a_ctl")]
    stages.append((oper, "base"))
    main = avl_session(wd / "main", "ss", deck, stages, mass=mass_file(asm), V=V)
    rows = parse_sysmat_full(wd / "main" / "sys.out")
    modes = fk.lateral_modes([r[:12] for r in rows])
    A4, B4 = lat_blocks(rows)
    b0 = main["base"]
    res = dict(cfg=cfg, cg=cg, zcg=zcg, xnp=xnp, mac=mac, sm_pct=100 * (xnp - cg) / mac, alpha=alpha, cl_trim=cl_trim, cm_trim=b0["Cm"], V=V, mass_g=asm["M_g"], ballast_g=asm["ballast_g"],
               inertia=asm["I_kgm2"], deriv={k: v for k, v in b0.items()}, modes={k: v for k, v in modes.items() if k != "eigs"}, eigs=modes["eigs"], A4=A4, B4=B4, wd=str(wd))
    asym = {}
    res["ctl"] = asym_entry(main["a_ctl"], b0)                   # differential through the AVL control variables D1 / D2 (flap R / L, or extra tail-half bend R / L)
    if cfg["kind"] == "b45":
        a = main["a_ctl"]
    else:
        a = avl_session(wd / "a", "ss", deck_for(cfg, cg, zcg, bends=(SW_BEND + AMP * ASYM_DEG / 2, SW_BEND - AMP * ASYM_DEG / 2)), [(oper, "x")])["x"]
    asym["a"] = asym_entry(a, b0)
    b = avl_session(wd / "b", "ss", deck_for(cfg, cg, zcg, wing_inc=(AMP * ASYM_DEG / 2, -AMP * ASYM_DEG / 2)), [(oper, "x")])["x"]
    asym["b"] = asym_entry(b, b0)
    if cfg["kind"] == "b45" or cfg["fin"]:
        A = AMP * ASYM_DEG / 2                                         # one fin toed +A and -A: the odd part (central difference) is the linear response
        fin_kw = lambda x: dict(fin_toe=(x, 0.0)) if cfg["kind"] == "b45" else dict(fin_toe=x)
        cp = avl_session(wd / "c_plus", "ss", deck_for(cfg, cg, zcg, **fin_kw(+A)), [(oper, "x")])["x"]
        cm = avl_session(wd / "c_minus", "ss", deck_for(cfg, cg, zcg, **fin_kw(-A)), [(oper, "x")])["x"]
        odd = {k_: (cp[k_] - cm[k_]) / 2.0 for k_ in ("Cl", "Cn", "CY", "CL", "Cm")}
        zero = {k_: 0.0 for k_ in odd}
        sg = 1.0 if odd["CY"] < 0 else -1.0                              # positive toe = trailing edge toward +y <=> side force toward -y
        asym["c"] = asym_entry(odd, zero, sg * AMP / A)
        asym["c"]["dCL"], asym["c"]["dCm"] = odd["CL"] / A, odd["Cm"] / A
        asym["c"]["ainc_sign"] = sg
    d = res["deriv"]
    for k, v in asym.items():
        rh, den = rhat_closed(d["Clb"], d["Cnb"], d["Clr"], d["Cnr"], v["dCl"], v["dCn"])
        v.update(rhat=rh, den=den, beta=beta_steady(d["Clb"], d["Clr"], v["dCl"], rh), R400=radius_m(rh, 400.0), R250=radius_m(rh, 250.0),
                 hdg400=heading_deg(rh, 400.0), hdg250=heading_deg(rh, 250.0), bank_deg=math.degrees(math.atan(2 * rh * V * V / (kc["b_mm"] * 1e-3 * GRAV))))
        rs, rb, denb, betab = rhat_body(d["Clb"], d["Cnb"], d["Clr"], d["Cnr"], d["Clp"], d["Cnp"], v["dCl"], v["dCn"], alpha)
        v.update(rhat_s_body=rs, rhat_body=rb, den_body=denb, beta_body=betab, R400_body=radius_m(rb, 400.0), R250_body=radius_m(rb, 250.0),
                 hdg400_body=heading_deg(rb, 400.0), hdg250_body=heading_deg(rb, 250.0), bank_deg_body=math.degrees(math.atan(2 * rb * V * V / (kc["b_mm"] * 1e-3 * GRAV))))
    res["asym"] = asym
    return res


def judge(res, base):
    """Pre-stated rule parts 1-4 (README)."""
    m = res["modes"]
    sp = m["spiral"]
    t2 = (math.log(2) / sp) if (sp is not None and sp > 0) else None
    r1 = sp is not None and (sp < 0 or t2 > SPIRAL_T2_MIN_S)
    r2 = m.get("dr_zeta") is not None and m["dr_zeta"] >= DR_ZETA_MIN and m["roll"] is not None and m["roll"] < 0
    hd = {k: abs(v["hdg400"]) for k, v in res["asym"].items()}
    r3 = all(h < HEADING_LIMIT_DEG for h in hd.values())
    r4 = abs(res["sm_pct"] - base["sm_pct"]) <= SM_TOL_PCT and abs(res["cl_trim"] - base["cl_trim"]) <= CL_TOL
    hdb = {k: abs(v["hdg400_body"]) for k, v in res["asym"].items()}
    r3b = all(h < HEADING_LIMIT_DEG for h in hdb.values())
    return dict(r1=r1, r2=r2, r3=r3, r4=r4, passed=bool(r1 and r2 and r3 and r4), t2=t2, hdg=hd, r3_body=r3b, passed_body=bool(r1 and r2 and r3b and r4), hdg_body=hdb)


# =============================================================================================== cross-check, trim tab
def u_vec(ncols, a=ASYM_DEG):
    u = [0.0] * ncols
    u[0], u[1] = a / 2, -a / 2
    return u


def crosscheck(res):
    """Closed forms (from the control derivatives of the AVL stability file) against (i) the algebraic equilibrium of the AVL system matrix with its control columns and (ii) RK4 time integration
    (stable spiral only).  Two closed forms: the pre-stated one (stability-axis p' = 0) and the body-axis one (p = 0, i.e. p' = r' tan alpha) that matches the AVL matrix frame.
    Compared quantity: heading-rate r_hat = psi_dot b / 2V (psi_dot = the matrix state r); the pre-stated form is used as is (its r_hat) for 'pre'."""
    d, A4, B4 = res["deriv"], res["A4"], res["B4"]
    b = kind_consts(res["cfg"]["kind"])["b_mm"] / 1000.0
    V = res["V"]
    dcl, dcn = res["ctl"]["dCl"], res["ctl"]["dCn"]
    rh_pre, _ = rhat_closed(d["Clb"], d["Cnb"], d["Clr"], d["Cnr"], dcl, dcn)
    _, rh_body, den_body, _ = rhat_body(d["Clb"], d["Cnb"], d["Clr"], d["Cnr"], d["Clp"], d["Cnp"], dcl, dcn, res["alpha"])
    u = u_vec(len(B4[0]))
    bu = [sum(B4[i][c] * u[c] for c in range(len(u))) for i in range(4)]
    xs = solve(A4, [-v for v in bu])
    out = dict(name=res["cfg"]["name"], dCl_ctl=dcl, dCn_ctl=dcn, rhat_pre=rh_pre, rhat_body=rh_body, rhat_equilibrium=xs[2] * b / (2 * V), spiral=res["modes"]["spiral"], integ=False, rhat_integrated=None, T=None)
    pc = lambda a, ref: (100 * (a - ref) / ref) if ref else None
    out["err_eq_pre_pct"], out["err_eq_body_pct"] = pc(out["rhat_equilibrium"], rh_pre), pc(out["rhat_equilibrium"], rh_body)
    num = d["Cnb"] * dcl - d["Clb"] * dcn
    unc = (abs(d["Cnb"]) + abs(d["Clb"])) * 1e-5 / AMP                  # the .ft files print Cl, Cn to 5 decimals: uncertainty of the numerator from the input run
    out["numerator"], out["numerator_uncertainty"] = num, unc
    out["num_over_unc"] = abs(num) / unc if unc else None
    out["well_conditioned"] = bool(abs(num) > 5 * unc)
    maxre = max(e[0] for e in res["eigs"])
    out["max_real_eig"] = maxre
    if maxre < 0:                                    # every mode stable: a steady state exists and the integration approaches it
        T = max(30.0, min(3000.0, 8.0 / abs(maxre)))
        traj, x = integrate_lateral(A4, B4, u, T)
        out.update(integ=True, T=T, rhat_integrated=x[2] * b / (2 * V), bank_deg_integrated=math.degrees(x[3]), p_end=x[1])
        out["err_int_pre_pct"], out["err_int_body_pct"] = pc(out["rhat_integrated"], rh_pre), pc(out["rhat_integrated"], rh_body)
        out["bank_deg_body"] = math.degrees(math.atan(2 * rh_body * V * V / (b * GRAV)))
    return out


def tab_analysis(cfg, res, work: Path):
    """Trim tab (hinge at 70 % fin chord) on the right tip fin (B45) or the centre fin (Swift A): deflection (deg, positive = trailing edge toward +y) that makes r_hat = 0 for each asymmetry
    (r_hat = 0 is the numerator Cnb dCl - Clb dCn = 0, the same for both closed forms).  AVL rotates control surfaces exactly, so the tab response has an even part: the sensitivity is the central
    difference at +-TAB_PROBE deg, and the check runs apply asymmetry and tab together at +s and -s (odd part)."""
    cg, zcg, alpha = res["cg"], res["zcg"], res["alpha"]
    wd = work / (cfg["name"] + "_tab")
    oper = oper_nominal(cfg) + [f"A A {alpha:.6f}"]
    d = res["deriv"]
    deck = deck_for(cfg, cg, zcg, tab=True)
    P = TAB_PROBE
    t0 = avl_session(wd, "ss", deck, [(oper + [f"D3 D3 {P}"], "tp"), (oper + [f"D3 D3 {-P}"], "tm")])
    sl = lambda q: (t0["tp"][q] - t0["tm"][q]) / (2 * P)
    sg = 1.0 if sl("CY") < 0 else -1.0                                # raw D3 > 0 is TE toward +y if the tab force points to -y
    dcl_t, dcn_t, dcy_t = sg * sl("Cl"), sg * sl("Cn"), sg * sl("CY")
    out = dict(name=cfg["name"], tab_sign=sg, dCl_per_deg=dcl_t, dCn_per_deg=dcn_t, dCY_per_deg=dcy_t, per_asym={})
    for k, v in res["asym"].items():
        num = d["Cnb"] * v["dCl"] - d["Clb"] * v["dCn"]
        delta = -num / (d["Cnb"] * dcl_t - d["Clb"] * dcn_t)
        M = min(AMP / 2, 12.0 / max(abs(delta), 1e-9))
        runs = {}
        for s in (1.0, -1.0):
            raw = f"D3 D3 {sg * delta * M * s:.6f}"
            if k == "a" and cfg["kind"] == "b45":
                dk, st = deck, [([f"D1 D1 {B45_FLAP_UP + s * M * ASYM_DEG / 2}", f"D2 D2 {B45_FLAP_UP - s * M * ASYM_DEG / 2}", raw, f"A A {alpha:.6f}"], "v")]
            elif k == "a":
                dk, st = deck_for(cfg, cg, zcg, bends=(SW_BEND + s * M * ASYM_DEG / 2, SW_BEND - s * M * ASYM_DEG / 2), tab=True), [(oper + [raw], "v")]
            elif k == "b":
                dk, st = deck_for(cfg, cg, zcg, wing_inc=(s * M * ASYM_DEG / 2, -s * M * ASYM_DEG / 2), tab=True), [(oper + [raw], "v")]
            else:
                x = s * v["ainc_sign"] * M * ASYM_DEG                     # raw section angle that is a physical +s*M toe
                dk, st = deck_for(cfg, cg, zcg, tab=True, **(dict(fin_toe=(x, 0.0)) if cfg["kind"] == "b45" else dict(fin_toe=x))), [(oper + [raw], "v")]
            runs[s] = avl_session(wd / f"{k}{'p' if s > 0 else 'm'}", "ss", dk, st)["v"]
        dcl_v = (runs[1.0]["Cl"] - runs[-1.0]["Cl"]) / (2 * M)
        dcn_v = (runs[1.0]["Cn"] - runs[-1.0]["Cn"]) / (2 * M)
        rh_after, _ = rhat_closed(d["Clb"], d["Cnb"], d["Clr"], d["Cnr"], dcl_v, dcn_v)
        rb_after = rhat_body(d["Clb"], d["Cnb"], d["Clr"], d["Cnr"], d["Clp"], d["Cnp"], dcl_v, dcn_v, alpha)[1]
        out["per_asym"][k] = dict(delta_deg=delta, rhat_before=v["rhat"], rhat_after_avl=rh_after, rhat_body_before=v["rhat_body"], rhat_body_after=rb_after, hdg_before=v["hdg400"], hdg_after=heading_deg(rh_after, 400.0),
                                  hdg_body_before=v["hdg400_body"], hdg_body_after=heading_deg(rb_after, 400.0), beta_after=beta_steady(d["Clb"], d["Clr"], dcl_v, rh_after), check_amplitude_deg=M)
    return out


# =============================================================================================== grid
def run_grid(work: Path, only=None, log=print):
    cfgs = cfg_list()
    if only:
        cfgs = [c for c in cfgs if c["name"] in only]
    base = {}
    res = {}
    order = sorted(cfgs, key=lambda c: (c["kind"], c["gam"] != 0.0 or c["fin"] not in (1.0, False)))
    for kind in ("b45", "sw"):
        b = [c for c in cfg_list() if c["kind"] == kind and c["gam"] == 0.0 and c["fin"] in (1.0, False)][0]
        t = time.time()
        r = evaluate(b, None, work)
        base[kind] = dict(sm_pct=r["sm_pct"], cl_trim=r["cl_trim"], cg=r["cg"], name=b["name"])
        res[b["name"]] = r
        log(f"{b['name']}: CG {r['cg']:.2f} Xnp {r['xnp']:.2f} SM {r['sm_pct']:.2f}% CL {r['cl_trim']:.3f}  ({time.time() - t:.1f} s)")
    for c in cfgs:
        if c["name"] in res:
            continue
        t = time.time()
        res[c["name"]] = evaluate(c, base[c["kind"]], work)
        r = res[c["name"]]
        log(f"{c['name']}: CG {r['cg']:.2f} SM {r['sm_pct']:.2f}% CL {r['cl_trim']:.3f} spiral {r['modes']['spiral']:+.4f}  ({time.time() - t:.1f} s)")
    for name, r in res.items():
        r["rule"] = judge(r, base[r["cfg"]["kind"]])
    return res, base


# =============================================================================================== post-hoc comparison with Kai's reported turns (NOT part of the pre-stated rule)
POSTHOC = dict(release_m=(25.0, 30.0), pullout_loss_m=6.93, glide_deg=6.61, turns=(1.0, 2.0))      # Kai: 1-2 full turns per drop from 25-30 m; pull-out loss: swiftA_oscillation README (6.93 m, MODEL, Swift A; B45 value ASSUMED the same); glide angle: same README (-6.61 deg, MODEL; B45 ASSUMED the same)


def posthoc(res, kind_base_name):
    r = res[kind_base_name]
    P = POSTHOC
    tg = math.tan(math.radians(P["glide_deg"]))
    H = [h - P["pullout_loss_m"] for h in P["release_m"]]
    path = [h / tg for h in H]
    Rs = [path[0] / (2 * math.pi * P["turns"][1]), path[1] / (2 * math.pi * P["turns"][0])]
    b = kind_consts(r["cfg"]["kind"])["b_mm"] / 1000.0
    out = dict(name=kind_base_name, H=H, path=path, R_min=Rs[0], R_max=Rs[1], T_flight=[p / r["V"] for p in path], per_asym={})
    for k, v in r["asym"].items():
        d = {}
        for lab, key in (("pre", "rhat"), ("body", "rhat_body")):
            rh1 = abs(v[key])
            d[f"size_deg_for_Rmin_{lab}"] = ASYM_DEG * (b / (2 * Rs[0])) / rh1
            d[f"size_deg_for_Rmax_{lab}"] = ASYM_DEG * (b / (2 * Rs[1])) / rh1
            d[f"R_1deg_{lab}"] = radius_m(v[key], 400.0)
        out["per_asym"][k] = d
    # time-domain (linear, from rest) for asymmetry (a) through AVL's control columns: heading turns after the model flight time
    A4, B4 = r["A4"], r["B4"]
    V = r["V"]
    u = u_vec(len(B4[0]))
    td = []
    for T in out["T_flight"]:
        traj, x = integrate_lateral(A4, B4, u, T, dt=0.01)
        psi = sum(0.5 * (traj[i][1][2] + traj[i + 1][1][2]) * (traj[i + 1][0] - traj[i][0]) for i in range(len(traj) - 1))
        td.append(dict(T=T, turns_1deg=psi / (2 * math.pi), bank_deg=math.degrees(x[3])))
    out["time_domain_a"] = td
    for e in td:
        e["size_for_1turn"] = ASYM_DEG / abs(e["turns_1deg"]) if e["turns_1deg"] else None
        e["size_for_2turns"] = 2 * ASYM_DEG / abs(e["turns_1deg"]) if e["turns_1deg"] else None
    return out


# =============================================================================================== outputs
FIN_LABEL = {"b45": lambda f: "tip fins as flown" if f == 1.0 else "tip fins area x1.5", "sw": lambda f: "centre fin 2.0% S (fin_kit_v4)" if f else "no fin (as flown)"}


def t2_text(lam):
    if lam is None:
        return "n/a"
    return f"stable (time to half {math.log(2) / -lam:.1f} s)" if lam < 0 else f"doubles in {math.log(2) / lam:.1f} s"


def dr_text(m):
    sig = m["dr_re"]
    return f"{'half' if sig < 0 else 'double'} {math.log(2) / abs(sig):.1f} s" if sig else "n/a"


def cfg_rows(res):
    rows = []
    for n, r in res.items():
        c, d, m = r["cfg"], r["deriv"], r["modes"]
        wd = m["dr_wn"] * math.sqrt(max(1e-12, 1 - m["dr_zeta"] ** 2))
        rows.append(dict(config=n, glider="B45" if c["kind"] == "b45" else "Swift A foam 400", dihedral_deg=c["gam"], fin=FIN_LABEL[c["kind"]](c["fin"]), cg_mm_aft_root_LE=round(r["cg"], 2),
                         cg_mm_from_nose_tip=round(r["cg"] + (SW_NOSE_AHEAD if c["kind"] == "sw" else 0.0), 2), xnp_mm=round(r["xnp"], 2), static_margin_pct_mac=round(r["sm_pct"], 3),
                         trim_alpha_deg=round(r["alpha"], 3), trim_CL=round(r["cl_trim"], 4), trim_V_m_s=round(r["V"], 3), mass_g=round(r["mass_g"], 3), ballast_g=round(r["ballast_g"], 3),
                         CYb=round(d["CYb"], 5), Clb=round(d["Clb"], 5), Cnb=round(d["Cnb"], 5), Clp=round(d["Clp"], 5), Cnp=round(d["Cnp"], 5), Clr=round(d["Clr"], 5), Cnr=round(d["Cnr"], 5),
                         roll_eig_per_s=round(m["roll"], 3), roll_time_to_half_s=round(math.log(2) / abs(m["roll"]), 4), spiral_eig_per_s=round(m["spiral"], 5), spiral=t2_text(m["spiral"]),
                         dutch_roll_wn=round(m["dr_wn"], 3), dutch_roll_zeta=round(m["dr_zeta"], 4), dutch_roll_period_s=round(2 * math.pi / wd, 3), dutch_roll=dr_text(m),
                         D_prestated=r["asym"]["a"]["den"], D_body=r["asym"]["a"]["den_body"],
                         rule1_spiral=int(r["rule"]["r1"]), rule2_dutch_roll_and_roll=int(r["rule"]["r2"]), rule3_heading_prestated_rhat=int(r["rule"]["r3"]), rule3_heading_body_rhat=int(r["rule"]["r3_body"]),
                         rule4_sm_and_CL=int(r["rule"]["r4"]), STRAIGHT_CAPABLE_prestated=int(r["rule"]["passed"]), STRAIGHT_CAPABLE_body=int(r["rule"]["passed_body"])))
    return rows


def turn_rows(res):
    rows = []
    for n, r in res.items():
        for k, v in r["asym"].items():
            rows.append(dict(config=n, asymmetry={"a": "a differential pitch-trim surface 1 deg", "b": "b wing-half incidence 1 deg", "c": "c one-fin toe 1 deg"}[k], dCl=round(v["dCl"], 6), dCn=round(v["dCn"], 6), dCY=round(v["dCY"], 6),
                             rhat_prestated=round(v["rhat"], 6), R400_m_prestated=round(v["R400"], 2), heading_deg_per_100m_400mm_prestated=round(v["hdg400"], 1), R250_m_prestated=round(v["R250"], 2), heading_deg_per_100m_250mm_prestated=round(v["hdg250"], 1),
                             rhat_body=round(v["rhat_body"], 6), R400_m_body=round(v["R400_body"], 2), heading_deg_per_100m_400mm_body=round(v["hdg400_body"], 1), R250_m_body=round(v["R250_body"], 2), heading_deg_per_100m_250mm_body=round(v["hdg250_body"], 1),
                             sideslip_deg_body=round(math.degrees(v["beta_body"]), 3), bank_deg_body_400mm=round(v["bank_deg_body"], 2)))
    return rows


def write_csv(path, rows):
    names = list(dict.fromkeys(k for r in rows for k in r))
    with Path(path).open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=names)
        w.writeheader()
        w.writerows(rows)


def pick_tab_candidates(res):
    fin_cfgs = [n for n, r in res.items() if (r["cfg"]["kind"] == "b45" or r["cfg"]["fin"])]
    rank_pre = sorted(fin_cfgs, key=lambda n: max(abs(v["hdg400"]) for v in res[n]["asym"].values()))
    rank_body = sorted(fin_cfgs, key=lambda n: max(abs(v["hdg400_body"]) for v in res[n]["asym"].values()))
    return rank_pre, rank_body


def _json_safe(o):
    if isinstance(o, dict):
        return {str(k): _json_safe(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_json_safe(v) for v in o]
    if isinstance(o, float) and (math.isinf(o) or math.isnan(o)):
        return str(o)
    return o


DARK_CSS = """:root{color-scheme:dark;--bg:#14181d;--panel:#1c2229;--line:#333c46;--ink:#dde3ea;--muted:#9aa7b4;--link:#7fb4ff;--accent:#e89a55;--thead:#262e37}
body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.5 "Segoe UI",system-ui,sans-serif}main{max-width:1500px;margin:0 auto;padding:26px 20px 60px;display:flex;flex-direction:column;gap:16px}
h1{font-size:26px;margin:0}h2{font-size:19px;margin:0;border-bottom:1px solid var(--line);padding-bottom:5px}p,ul{margin:0;max-width:120ch}ul{padding-left:22px}
table{border-collapse:collapse;background:var(--panel);font-size:12.5px}th,td{border:1px solid var(--line);padding:3px 7px;text-align:left;vertical-align:top}th{background:var(--thead)}td.n{text-align:right;white-space:nowrap}
.note{border-left:3px solid var(--accent);padding:2px 0 2px 12px;color:var(--muted);font-size:14px}.scroll{overflow-x:auto}.good{background:#1d3b2a}.bad{background:#3b1d1d}a{color:var(--link)}"""


def html_table(rows, cols=None, cls=None):
    cols = cols or list(rows[0].keys())
    h = "<div class='scroll'><table><tr>" + "".join(f"<th>{c}</th>" for c in cols) + "</tr>"
    for r in rows:
        h += "<tr>" + "".join(f"<td class='n {(cls(c, r[c]) if cls else '')}'>{r[c]}</td>" for c in cols) + "</tr>"
    return h + "</table></div>"


def write_html(out: Path, cfgs, turns, ccs, tabs, ph):
    good_bad = lambda c, v: ("good" if v == 1 else "bad") if c.startswith(("rule", "STRAIGHT")) else ""
    P = [f"<!doctype html><html lang='en'><head><meta charset='utf-8'><title>Straight and smooth, phase 1</title><style>{DARK_CSS}</style></head><body><main>",
         "<h1>Straight and smooth, phase 1: lateral / turn side</h1>",
         "<p class='note'>MODEL = inviscid AVL 3.52 (flat plate, thickness ignored). ASSUMED = masses, inertias, asymmetry sizes (1 deg each), fin trim-tab hinge (70 % chord). Nothing here is a flight result. The pre-stated rule, method and limits are in <a href='README.md'>README.md</a>. 'prestated' r_hat = the formula as written before the run (stability-axis roll rate zero); 'body' r_hat = the same balance with the body-axis steady turn (roll rate zero in the AVL system-matrix frame), which is the one that matches AVL's time integration (see cross-check).</p>",
         "<h2>Configurations: trim, derivatives, modes, rule parts</h2>", html_table(cfgs, cls=good_bad),
         "<h2>Turn sensitivity for 1 deg of each reference asymmetry</h2>", html_table(turns),
         "<h2>Cross-check: closed form against the AVL system matrix</h2>", html_table(ccs),
         "<h2>Trim tab (hinge at 70 % fin chord, ASSUMED)</h2>", html_table(tabs) if tabs else "<p>not run</p>",
         "<h2>Post-hoc comparison with Kai's reported 1-2 turns per drop (not part of the pre-stated rule)</h2>", html_table(ph) if ph else "<p>not run</p>",
         "</main></body></html>"]
    (out / "index.html").write_text("\n".join(P), encoding="utf-8")

def md_table(head, rows):
    nl = chr(10)
    return "| " + " | ".join(head) + " |" + nl + "|" + "---|" * len(head) + nl + nl.join("| " + " | ".join(str(c) for c in r) + " |" for r in rows) + nl


# =============================================================================================== REVISION (reviewer REVISE, 2026-10-21): base-state elevation, Ixz, mode shapes, roll trim, trim-CL diagnosis
GLIDE_REF = -6.61                        # deg flight-path angle: swiftA_oscillation README (MODEL, Swift A); the same value is ASSUMED for B45
GAMMAS = (("g6.61", GLIDE_REF), ("g5", -5.0), ("g8", -8.0))
LINEAR_BANK_DEG = 30.0                   # per-1-deg radii whose steady bank angle exceeds this are outside the linear range and are not quoted to Kai
OSC_BETA_PHI_MIN = 0.5                   # post-hoc label threshold: |beta|/|phi| below this (or Cnb < 0) is not called a Dutch roll


def rhat_gamma(clb, cnb, clr, cnr, clp, cnp, dcl, dcn, gamma_deg):
    """Steady turn about the VERTICAL axis during a glide at flight-path angle gamma (<0): the rotation vector has a component along the velocity, p' = r' tan(-gamma), r' = Omega cos(gamma).
    Returns (r_hat stability axis, r_hat heading rate Omega b/2V, D_eff, beta)."""
    ta = math.tan(math.radians(-gamma_deg))
    clr_e, cnr_e = clr + ta * clp, cnr + ta * cnp
    rs, den = rhat_closed(clb, cnb, clr_e, cnr_e, dcl, dcn)
    return rs, rs / math.cos(math.radians(gamma_deg)), den, -(dcl + clr_e * rs) / clb


def _csolve(M, b):
    n = len(M)
    A = [list(map(complex, M[i])) + [complex(b[i])] for i in range(n)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(A[r][c]))
        A[c], A[p] = A[p], A[c]
        for r in range(n):
            if r != c:
                f = A[r][c] / A[c][c]
                A[r] = [x - f * y for x, y in zip(A[r], A[c])]
    return [A[i][n] / A[i][i] for i in range(n)]


def mode_shape(A4, V, lam):
    """Eigenvector of the 4 x 4 lateral block (v p r phi) for eigenvalue lam, scaled to phi = 1: returns |beta|/|phi|, |p b/2V|... as plain ratios (beta = v/V)."""
    M = [[A4[i][j] - (lam if i == j else 0) for j in range(4)] for i in range(4)]
    x = _csolve([row[:3] for row in M[:3]], [-M[i][3] for i in range(3)]) + [1.0 + 0j]
    return dict(beta_over_phi=abs(x[0]) / V, p_over_phi=abs(x[1]), r_over_phi=abs(x[2]))


def mode_run(cfg, res, gamma_deg, use_ixz, work: Path, tag):
    """AVL system matrix with base-state elevation theta0 = alpha_trim + gamma (MODE parameter E) and, optionally, the product of inertia from the point-mass model (MODE parameter IZX).
    AVL's IZX has the opposite sign to the product integral(x z dm) (found by test: +IZX makes N_v-dot more positive, which Etkin's Ixz of the opposite sign does), so IZX = -Jxz."""
    cg, zcg, alpha = res["cg"], res["zcg"], res["alpha"]
    asm = mass_for(cfg, cg)
    deck = deck_for(cfg, cg, zcg)
    extra = [f"E {alpha + gamma_deg:.5f}"] + ([f"IZX {-asm['jxz_kgm2']:.6e}"] if use_ixz else [])
    oper = oper_nominal(cfg) + [f"A A {alpha:.6f}"]
    avl_session(work / res["cfg"]["name"] / f"mode_{tag}", "ss", deck, [(oper, "base")], mass=mass_file(asm), V=res["V"], mode_extra=extra)
    rows = parse_sysmat_full(work / res["cfg"]["name"] / f"mode_{tag}" / "sys.out")
    modes = fk.lateral_modes([r[:12] for r in rows])
    A4, B4 = lat_blocks(rows)
    psi = [rows[11][j] for j in (4, 5, 6, 7)]
    ms = None
    cplx = [e for e in modes["eigs"] if e[1] > 1e-9]
    if cplx:
        ms = mode_shape(A4, res["V"], complex(*cplx[0]))
    return dict(modes={k: v for k, v in modes.items() if k != "eigs"}, eigs=modes["eigs"], A4=A4, B4=B4, psi=psi, shape=ms, jxz=asm["jxz_kgm2"], gamma=gamma_deg, ixz_used=use_ixz)


def kc_b(res):
    return kind_consts(res["cfg"]["kind"])["b_mm"] / 1000.0


def post_analysis(cfg, res, work: Path):
    """For gamma = -6.61 (no Ixz, and with Ixz), -5, -8 (with Ixz): modes, mode shape, corrected closed-form r_hat (heading rate), algebraic equilibrium and RK4 of the (a) differential through AVL's control columns."""
    d, V = res["deriv"], res["V"]
    b = kc_b(res)
    out = {}
    plan = [("g6.61_noIxz", GLIDE_REF, False)] + [(tg, g, True) for tg, g in GAMMAS]
    for tag, g, ixz in plan:
        m = mode_run(cfg, res, g, ixz, work, tag)
        e = dict(modes=m["modes"], shape=m["shape"], jxz=m["jxz"], gamma=g, ixz_used=ixz, asym={}, max_real_eig=max(z[0] for z in m["eigs"]), A4=m["A4"], B4=m["B4"], psi=m["psi"])
        osc = m["modes"]
        sh = m["shape"]
        e["osc_label"] = "Dutch roll" if (d["Cnb"] > 0 and sh and sh["beta_over_phi"] >= OSC_BETA_PHI_MIN) else "lateral oscillation (not a Dutch roll)"
        for k, v in res["asym"].items():
            rs, rh, den, beta = rhat_gamma(d["Clb"], d["Cnb"], d["Clr"], d["Cnr"], d["Clp"], d["Cnp"], v["dCl"], v["dCn"], g)
            bank = math.degrees(math.atan(2 * rh * V * V / (b * GRAV)))
            e["asym"][k] = dict(rhat_h=rh, rhat_s=rs, D=den, beta=beta, R400=radius_m(rh, 400.0), R250=radius_m(rh, 250.0), hdg400=heading_deg(rh, 400.0), hdg250=heading_deg(rh, 250.0), bank_deg=bank,
                                out_of_linear_range=bool(abs(bank) > LINEAR_BANK_DEG), size_for_R12=ASYM_DEG * (b / (2 * 12.409)) / abs(rh) if rh else None, size_for_R32=ASYM_DEG * (b / (2 * 31.685)) / abs(rh) if rh else None)
        # cross-check of the (a) differential through the control columns of this matrix
        dcl, dcn = res["ctl"]["dCl"], res["ctl"]["dCn"]
        _, rh_cf, den_cf, _ = rhat_gamma(d["Clb"], d["Cnb"], d["Clr"], d["Cnr"], d["Clp"], d["Cnp"], dcl, dcn, g)
        u = u_vec(len(m["B4"][0]))
        bu = [sum(m["B4"][i][c] * u[c] for c in range(len(u))) for i in range(4)]
        xs = solve(m["A4"], [-v_ for v_ in bu])
        om = sum(c * x for c, x in zip(m["psi"], xs))
        cc = dict(rhat_closed=rh_cf, rhat_equilibrium=om * b / (2 * V), rhat_rk4=None)
        cc["err_eq_pct"] = 100 * (cc["rhat_equilibrium"] - rh_cf) / rh_cf if rh_cf else None
        num = d["Cnb"] * dcl - d["Clb"] * dcn
        cc["num_over_unc"] = abs(num) / ((abs(d["Cnb"]) + abs(d["Clb"])) * 1e-5 / AMP)
        if e["max_real_eig"] < 0:
            T = max(30.0, min(3000.0, 8.0 / abs(e["max_real_eig"])))
            traj, x = integrate_lateral(m["A4"], m["B4"], u, T)
            cc["rhat_rk4"] = sum(c * xv for c, xv in zip(m["psi"], x)) * b / (2 * V)
            cc["err_rk4_pct"] = 100 * (cc["rhat_rk4"] - rh_cf) / rh_cf if rh_cf else None
            cc["T"] = T
        e["crosscheck"] = cc
        out[tag] = e
    return out


def roll_trims(res, tabs):
    """Differential flap / tail-half angle (degrees of D1 - D2, i.e. +-half on each side) that zeroes the r_hat numerator Cnb dCl - Clb dCn for each asymmetry, from the control response measured in the
    main run (res['ctl'], per degree of differential), and the residual steady sideslip at r_hat = 0.  B45: centre flaps.  Swift A: tail halves (the model's only differential surface).  Wing-tip TE
    surfaces do not exist in the model, so no wing-tip number can be computed from the existing data."""
    d, c = res["deriv"], res["ctl"]
    out = {}
    for k, v in res["asym"].items():
        num = d["Cnb"] * v["dCl"] - d["Clb"] * v["dCn"]
        den = d["Cnb"] * c["dCl"] - d["Clb"] * c["dCn"]
        dlt = -num / den
        beta = -(v["dCl"] + dlt * c["dCl"]) / d["Clb"]
        out[k] = dict(diff_deg=dlt, per_side_deg=dlt / 2, residual_sideslip_deg=math.degrees(beta), dCl_per_deg=c["dCl"], dCn_per_deg=c["dCn"], beyond_10deg=bool(abs(dlt) > 10.0))
    return out


def trim_at(cfg, cg, zcg, alpha0, work):
    alpha = alpha0
    for _ in range(5):
        o = avl_session(work, "ss", deck_for(cfg, cg, zcg), [(oper_nominal(cfg) + [f"A A {alpha:.6f}"], "t")])["t"]
        if abs(o["Cm"]) < 3e-4:
            break
        alpha -= o["Cm"] / (o["Cma"] * math.pi / 180.0)
    return alpha, o["CL"]


def diagnose_trim(res, work: Path):
    """Why does the trim CL rise with dihedral at constant static margin?  CL_trim = Cm_NP0 / SM (moment about the NP is alpha-independent).  Rows: Xnp, CLa, Cm_NP0 (about the NP, z-reference 0),
    CL predicted by Cm_NP0 / SM, and the AVL trim CL with the CG height as modelled, with the CG height forced to 0 and with the baseline's CG height."""
    rows = []
    for kind, fin, nm in (("b45", 1.0, "B45_fin1"), ("sw", False, "SwiftA_nofin")):
        kc = kind_consts(kind)
        z_base = res[f"{nm}_d0"]["zcg"]
        for gam in (0.0, 6.0, 12.0):
            r = res[f"{nm}_d{gam:g}"]
            cfg = r["cfg"]
            deck = deck_for(cfg, kc["cg_nom"], 0.0)
            o = avl_session(work / f"trimdiag_{r['cfg']['name']}", "ss", deck, [(oper_nominal(cfg) + ["A A 0"], "a0"), (oper_nominal(cfg) + ["A A 4"], "a4")])
            xnp = o["a4"]["Xnp"] * 1000
            cm_np0 = o["a0"]["Cm"] + o["a0"]["CL"] * (xnp - kc["cg_nom"]) / kc["mac"]
            cla = (o["a4"]["CL"] - o["a0"]["CL"]) / 4 * 180 / math.pi
            sm = r["sm_pct"] / 100
            row = dict(config=r["cfg"]["name"], xnp_mm=xnp, CLa_per_rad=cla, CL0=o["a0"]["CL"], Cm_about_NP_alpha0=cm_np0, SM_pct=r["sm_pct"], CL_trim_pred_CmNP_over_SM=cm_np0 / sm,
                       zcg_mm_modelled=r["zcg"], alpha_trim_deg=r["alpha"], CL_trim_AVL_zcg_modelled=r["cl_trim"])
            wd = work / f"trimdiag_{r['cfg']['name']}"
            for lab, z in (("zcg0", 0.0), ("zcg_baseline", z_base)):
                a_, cl_ = trim_at(cfg, r["cg"], z, r["alpha"], wd / lab)
                row[f"CL_trim_AVL_{lab}"] = cl_
            rows.append(row)
    return rows

# =============================================================================================== README results (revised) and page
def td_heading(e, res, T_list):
    """Linear time response from rest to the 1 deg differential on the control columns of the corrected matrix (heading rate = AVL's psi row): turns after each flight time."""
    u = u_vec(len(e["B4"][0]))
    out = []
    for T in T_list:
        traj, x = integrate_lateral(e["A4"], e["B4"], u, T, dt=0.01)
        om = [sum(c * xv for c, xv in zip(e["psi"], s[1])) for s in traj]
        psi = sum(0.5 * (om[i] + om[i + 1]) for i in range(len(om) - 1)) * (traj[1][0] - traj[0][0])
        out.append(dict(T=T, turns_per_deg=psi / (2 * math.pi), bank_deg=math.degrees(x[3])))
    for o in out:
        o["deg_for_1_turn"] = ASYM_DEG / abs(o["turns_per_deg"]) if o["turns_per_deg"] else None
        o["deg_for_2_turns"] = 2 * o["deg_for_1_turn"] if o["deg_for_1_turn"] else None
    return out


def rng(lo, hi):
    return f"{lo:.2f}-{hi:.2f}"


def write_readme_results(out: Path, res, base, ccs, tabs, ph, post, rolls, tdiag, wall, ranks):
    rank_pre, rank_body = ranks
    names = list(res)
    N = len(names)
    L = []
    A = L.append
    b45n, swn = base["b45"]["name"], base["sw"]["name"]
    P = lambda n, tag="g6.61": post[n][tag]
    A(f"Filled in {time.strftime('%Y-%m-%d %H:%M')} (local) by `rigid_glider/straight_smooth_v1.py`, REVISED pass after the reviewer's REVISE (8 directives); wall time of the full run {wall:.0f} s. MODEL = inviscid AVL 3.52; ASSUMED = masses, inertias, asymmetry sizes, tab hinge, B45 glide angle and pull-out loss; KAI = Kai's statement.\n")
    A("### What changed vs the first pass\n")
    A("1. AVL modes, system matrices and RK4 now use the base-state elevation theta0 = alpha_trim + gamma (MODE parameter E; the first pass left it at 0, i.e. an implied glide angle of -alpha, 4-9 deg). gamma = -6.61 deg (MODEL for Swift A, ASSUMED for B45), sensitivity gamma = -5 and -8 deg.")
    A("2. Closed form for the steady turn now uses the roll rate of a turn about the VERTICAL axis during a glide, p' = r' tan(-gamma). The pre-stated p' = 0 form is physically wrong for a glide (it is the stability-axis roll rate of a turn about the velocity vector, which a descending turn is not). The first pass's 'body' form (tan alpha) was only right for theta0 = 0 and is superseded. RK4 vs closed form on the SAME AVL matrix checks algebra (the closed form is the steady state of that linear system), not physics (that the base state and the matrix are right).")
    A("3. The product of inertia Jxz = integral(x z dm) from the point-mass model (dihedral tips are up and aft) is now in the mode calculation; Delta zeta per configuration is reported, and the oscillatory mode is labelled by its shape (|beta|/|phi|, sign of Cnb).")
    A("4. Kai-facing text leads with the degrees of each asymmetry needed for R = 12-32 m; per-1-deg radii beyond bank 30 deg are not quoted (kept in the CSV, flagged out of linear range).")
    A("5. Roll trims (differential centre flap for B45; differential tail halves for Swift A) are computed next to the fin tab. Options, none chosen.")
    A("6. The trim-CL rise with dihedral at fixed static margin is diagnosed (below) before any re-trim statement.")
    A("7. Tests re-run (summary at the end, including the hash check of the pre-stated section above, which is unchanged).")
    A("8. Dihedral is framed honestly (summary below).\n")
    # ------------------------------------------------------------------ summary for Kai
    A("### Summary for Kai (read this first)\n")
    A(f"**Degrees of each asymmetry needed to give a circle of R = 12.4 to 31.7 m** (Kai's rough 1-2 turns per drop from 25-30 m; R from release height minus the {POSTHOC['pullout_loss_m']:g} m pull-out loss, glide angle {POSTHOC['glide_deg']:g} deg, MODEL for Swift, ASSUMED for B45). Corrected steady-turn model (gamma = {GLIDE_REF:g} deg, Ixz included), linear scaling from 1 deg:\n")
    head = ["glider as flown", "(a) differential trim surface (B45 flaps / Swift tail halves)", "(b) wing-half incidence difference", "(c) one-fin toe"]
    rows = []
    for n, lab in ((b45n, "B45 (tip fins, flaps 10 deg up)"), (swn, "Swift A foam 400 (no fin, tail bent 6 deg)")):
        e = P(n)["asym"]
        rows.append([lab, rng(e["a"]["size_for_R32"], e["a"]["size_for_R12"]) + " deg", rng(e["b"]["size_for_R32"], e["b"]["size_for_R12"]) + " deg", (rng(e["c"]["size_for_R32"], e["c"]["size_for_R12"]) + " deg") if "c" in e else "no fin"])
    A(md_table(head, rows))
    td = {n: td_heading(P(n), res[n], ph[n]["T_flight"]) for n in (b45n, swn)}
    A("**How far to trust these numbers.** AVL's Clb (hence the denominator D and r_hat) is known to be off for thin, low-aspect-ratio plates at Re of 10-30k: measured roll due to sideslip departs from vortex-lattice predictions because tip-vortex roll-up is destabilising in Clb (Linehan & Mohseni, cited in `research/lateral_robustness/reports/Glider robustness to build asymmetry.md`, section 'How far to trust AVL at Re 10-30k'), and CLa is uncertain by about +-30 %. These numbers therefore rank configurations and give an order of magnitude; they are not predicted radii or tolerances.\n")
    A("**Option for Kai (not a decision):** calibration drops with a B45 given a deliberate, measured wing-half twist of 0.5 deg and of 1.0 deg (one half only, gauged), with the circle radius measured on repeated drops. That would give the real turn per degree of twist to replace the model's order of magnitude.\n")
    A(f"Time-domain check for (a) from rest (linear, same corrected matrix, glide time {ph[b45n]['T_flight'][0]:.0f}-{ph[b45n]['T_flight'][1]:.0f} s): B45 needs {td[b45n][1]['deg_for_1_turn']:.2f}-{td[b45n][0]['deg_for_2_turns']:.2f} deg for 1-2 turns (its spiral is unstable, which amplifies a small input); Swift A needs {td[swn][1]['deg_for_1_turn']:.1f}-{td[swn][0]['deg_for_2_turns']:.1f} deg of tail-half difference.\n")
    A("What this says: asymmetries of a fraction of a degree to a couple of degrees (wing-half twist 0.1-0.3 deg, one-fin toe, a flap hinge that differs by 0.1-0.5 deg) are enough in the model to produce Kai's circles. For the Swift A foam, a tail-half bend difference alone would need a few degrees, so a small wing-half incidence difference (warp) is the likelier model route. For the Swift A the quasi-steady figure (0.9-2.2 deg for (a)) is smaller than the time-domain one because its spiral is stable with a half-life of about 13 s (at gamma = -6.61): the circle has not fully developed within a 28-36 s glide. This is a consistency statement, not proof.\n")
    A(f"**Dihedral, honestly:** its real benefit in this model is curing the B45 spiral (B45 baseline spiral doubles in {math.log(2) / P(b45n)['modes']['spiral']:.0f} s; with the tip fins as flown it is stable from 3 deg of dihedral; with fins x1.5 it is still weakly unstable at 3 deg, doubling in {math.log(2) / P('B45_fin1.5_d3')['modes']['spiral']:.1f} s, which passes part 1 only on the 15 s criterion, and stable from 6 deg). The Swift A foam's spiral is already stable at gamma = {GLIDE_REF:g} (half-life {math.log(2) / -P(swn)['modes']['spiral']:.0f} s; it only looked neutral in the first pass because AVL's base state was theta0 = 0). The apparent help of dihedral on asymmetries (a) and (c) is cancellation as Cnb falls toward zero (Swift A Cnb goes {res['SwiftA_nofin_d0']['deriv']['Cnb']:+.4f} -> {res['SwiftA_nofin_d6']['deriv']['Cnb']:+.4f} by 6 deg; B45 {res['B45_fin1_d0']['deriv']['Cnb']:+.4f} -> {res['B45_fin1_d12']['deriv']['Cnb']:+.4f} by 12 deg), which is a liability: the Swift A loses directional stability, the oscillation damping falls to about zero or below zero from 3-6 deg (more with Ixz), and asymmetry (b) (wing-half incidence) is not cured by any dihedral in the grid. No configuration in the grid is straight-capable.\n")
    A("**Glide angle caveat:** the glide angle is unmeasured. The sensitivity band used here (-5 and -8 deg) does not contain the roughly -11 deg (L/D about 6) given by `swoop_model_v1`. At -11 deg the B45 baseline spiral is stable, so the statements that the B45 spiral is unstable and that dihedral cures it depend on the assumed glide angle and are not findings until the glide angle is measured (see the post-hoc glide-angle section below and `results/runs/rigid_glider/shape_sweep_v1/gamma11_check.csv`).\n")
    A("**Stated plainly:** inviscid AVL (no viscous effects, flat plate, no thickness; fin and tab effectiveness are upper estimates); inertias ASSUMED (300 g/m2 foam, stick z ASSUMED); asymmetry sizes ASSUMED (1 deg each); no measured turn exists (Kai's 1-2 turns per drop is a rough count, no direction, no video from the side); the Swift A foam 400 as flown had no vertical element at all (KAI, 2026-10-21; it matches the build files, `flight_day_cg_page.py`, `swiftA_oscillation_study.py`).\n")
    # ------------------------------------------------------------------ verdict first pass
    cnt = lambda key: sum(1 for n in names if res[n]["rule"][key])
    A("### Verdict against the pre-stated rule (FIRST PASS: theta0 = 0, p' = 0 closed form, no Ixz)\n")
    A(f"**No configuration of the {N} is STRAIGHT-CAPABLE.** Parts passed: part 1 spiral {cnt('r1')}/{N}; part 2 Dutch roll and roll {cnt('r2')}/{N}; part 3 heading change {cnt('r3')}/{N}; part 4 static margin and trim CL {cnt('r4')}/{N}. Per-configuration pass/fail with the corrected frame is in the next section; the first-pass numbers are in `configs.csv`.\n")
    # ------------------------------------------------------------------ post-hoc corrected verdict
    A("### Post-hoc corrected verdict (gamma = -6.61 deg, Ixz included; labelled post-hoc, the pre-stated rule above is unchanged)\n")
    head = ["config", "trim CL", "spiral", "oscillation zeta, no Ixz -> with Ixz (delta)", "oscillation label (beta/phi ratio)", "Jxz kg m2", "1 spiral", "2 DR+roll", "3 heading (<90 deg all asym)", "4 SM+CL", "pass"]
    rows = []
    pc = {}
    for n in names:
        e, e0 = P(n), P(n, "g6.61_noIxz")
        m = e["modes"]
        ru = res[n]["rule"]
        sp = m["spiral"]
        r1 = sp < 0 or math.log(2) / sp > SPIRAL_T2_MIN_S
        r2 = m["dr_zeta"] >= DR_ZETA_MIN and m["roll"] < 0 and e["osc_label"] == "Dutch roll"
        r3 = all(abs(v["hdg400"]) < HEADING_LIMIT_DEG for v in e["asym"].values())
        pc[n] = dict(r1=r1, r2=r2, r3=r3, r4=ru["r4"], passed=bool(r1 and r2 and r3 and ru["r4"]))
        rows.append([n, f"{res[n]['cl_trim']:.3f}", t2_text(sp), f"{e0['modes']['dr_zeta']:+.3f} -> {m['dr_zeta']:+.3f} ({m['dr_zeta'] - e0['modes']['dr_zeta']:+.3f})", f"{e['osc_label']} ({e['shape']['beta_over_phi']:.2f})" if e["shape"] else "n/a",
                     f"{e['jxz']:+.1e}", *["pass" if pc[n][k] else "FAIL" for k in ("r1", "r2", "r3", "r4")], "PASS" if pc[n]["passed"] else "no"])
    A(md_table(head, rows))
    nlab = [n for n in names if P(n)["osc_label"] != "Dutch roll"]
    A(f"Mode shapes: {len(nlab)} of {N} configurations have an oscillatory lateral mode that fails the post-hoc Dutch-roll label (Cnb < 0 or |beta|/|phi| < {OSC_BETA_PHI_MIN}): {', '.join(nlab) or 'none'}. These are listed as 'lateral oscillation (not a Dutch roll)' and are not credited with a Dutch roll damping ratio in part 2. Ixz changes zeta by {min(P(n)['modes']['dr_zeta'] - P(n, 'g6.61_noIxz')['modes']['dr_zeta'] for n in names):+.3f} to {max(P(n)['modes']['dr_zeta'] - P(n, 'g6.61_noIxz')['modes']['dr_zeta'] for n in names):+.3f} (per configuration above). Moving from theta0 = 0 (first pass) to theta0 = alpha + gamma changes the spiral eigenvalue of the B45 baseline from {res[b45n]['modes']['spiral']:+.4f} to {P(b45n, 'g6.61_noIxz')['modes']['spiral']:+.4f} 1/s and of the Swift baseline from {res[swn]['modes']['spiral']:+.4f} to {P(swn, 'g6.61_noIxz')['modes']['spiral']:+.4f} 1/s.\n")
    A("Glide-angle sensitivity (spiral eigenvalue, oscillation zeta, and the steady heading-rate r_hat for asymmetries (a) and (b), Ixz included; selected configurations, all 20 in the CSV; r_hat jumps by large factors where the denominator is near zero, e.g. a spiral eigenvalue near 0):\n")
    head = ["config", "gamma -5: spiral, zeta", "gamma -6.61: spiral, zeta", "gamma -8: spiral, zeta", "r_hat (a) at -5 / -6.61 / -8", "r_hat (b) at -5 / -6.61 / -8"]
    rows = []
    for n in (b45n, "B45_fin1.5_d0", "B45_fin1_d3", "B45_fin1.5_d12", swn, "SwiftA_centrefin_d0", "SwiftA_nofin_d3"):
        if n not in post:
            continue
        cells = [n]
        for tg in ("g5", "g6.61", "g8"):
            m = P(n, tg)["modes"]
            cells.append(f"{m['spiral']:+.4f} 1/s, {m['dr_zeta']:+.3f}")
        for k in ("a", "b"):
            cells.append(" / ".join(f"{P(n, tg)['asym'][k]['rhat_h']:+.5f}" for tg in ("g5", "g6.61", "g8")))
        rows.append(cells)
    A(md_table(head, rows))
    A("All 20 configurations at the three glide angles are in `post_modes.csv` and `post_turns.csv`.\n")
    # ------------------------------------------------------------------ cross-check
    A("### Cross-check of the corrected closed form against the AVL system matrix (algebra, not physics)\n")
    A("Pre-stated requirement: closed form vs time integration within 5 % for at least 3 configurations. The pre-stated p' = 0 form was off by 40-2983 % in the first pass (theta0 = 0 matrix) and is physically wrong for a glide. With the corrected closed form (p' = r' tan(-gamma)) and the matrix at theta0 = alpha + gamma (Ixz included), the steady heading rate of the linear system (RK4 from rest where all modes are stable; algebraic equilibrium of A and its control columns otherwise) is compared with the closed form for the (a) differential. This verifies the algebra only; it cannot say whether AVL's base state or the 1 deg inputs describe the real glider.\n")
    head = ["config", "gamma", "all modes stable", "r_hat closed (corrected)", "r_hat equilibrium", "error %", "r_hat RK4", "error %", "num/unc"]
    rows = []
    cnt_ok = {}
    for n in names:
        for tg in ("g6.61", "g5", "g8"):
            cc = P(n, tg)["crosscheck"]
            if cc["rhat_rk4"] is not None and abs(cc["err_rk4_pct"]) < 5.0:
                cnt_ok[tg] = cnt_ok.get(tg, 0) + 1
            if tg == "g6.61" or cc["rhat_rk4"] is not None:
                rows.append([n, tg, "yes" if cc["rhat_rk4"] is not None else "no", f"{cc['rhat_closed']:+.6f}", f"{cc['rhat_equilibrium']:+.6f}", f"{cc['err_eq_pct']:+.1f}" if cc["err_eq_pct"] is not None else "n/a",
                             f"{cc['rhat_rk4']:+.6f}" if cc["rhat_rk4"] is not None else "-", f"{cc['err_rk4_pct']:+.2f}" if cc["rhat_rk4"] is not None else "-", f"{cc['num_over_unc']:.1f}"])
    A(md_table(head, rows))
    stable6 = [(n, P(n)["crosscheck"]) for n in names if P(n)["crosscheck"]["rhat_rk4"] is not None]
    okn = [n for n, c in stable6 if abs(c["err_rk4_pct"]) < 5.0]
    A(f"At gamma = {GLIDE_REF:g}: {len(stable6)} configurations have all modes stable; {len(okn)} of them agree with RK4 within 5 % ({', '.join(okn)}). Within 5 % at gamma = -5: {cnt_ok.get('g5', 0)}, at -8: {cnt_ok.get('g8', 0)} (counts over all stable configurations). Where num/unc is below about 5, the numerator Cnb dCl - Clb dCn is at the print resolution of AVL's input run and the comparison is not informative.\n")
    # ------------------------------------------------------------------ turn table (corrected)
    A("### Turn sensitivity, corrected frame (1 deg of each asymmetry; gamma = -6.61, Ixz included)\n")
    A(f"R and heading change are quoted only where the steady bank stays below {LINEAR_BANK_DEG:g} deg (linear range); otherwise 'out of range' (raw numbers in `post_turns.csv`). Radius R = b/(2 r_hat) at 400 mm; 250 mm gives R x 0.625 and heading change x 1.6 at the same angles.\n")
    head = ["config", "asym", "dCl", "dCn", "r_hat (heading rate)", "R400 m", "heading deg/100 m (400)", "sideslip deg", "bank deg", "deg for R=31.7 m", "deg for R=12.4 m"]
    rows = []
    for n in names:
        for k, v in P(n)["asym"].items():
            ra = res[n]["asym"][k]
            oor = v["out_of_linear_range"]
            rows.append([n, k, f"{ra['dCl']:+.5f}", f"{ra['dCn']:+.5f}", f"{v['rhat_h']:+.5f}", "out of range" if oor else f"{v['R400']:.1f}", "out of range" if oor else f"{v['hdg400']:+.0f}",
                         "-" if oor else f"{math.degrees(v['beta']):+.2f}", "-" if oor else f"{v['bank_deg']:+.1f}", f"{v['size_for_R32']:.3f}", f"{v['size_for_R12']:.3f}"])
    A(md_table(head, rows))
    # ------------------------------------------------------------------ trim options
    A("### Trim options for the turn (options only, none chosen)\n")
    A("**Option 1: fin trim tab** (fin or rudder trim is speed-sensitive, which is known free-flight practice and matters for a nose-down drop whose speed changes a lot; hinge at 70 % fin chord, ASSUMED; right tip fin on B45, centre fin on Swift A; r_hat = 0 for each 1 deg asymmetry separately, leaving a steady sideslip). Tab angles from the first pass (the tab numerator condition does not depend on the closed-form frame):\n")
    head = ["config", "role", "asym", "tab deflection deg (TE toward +y)", "residual sideslip deg", "tab dCn per deg"]
    rows = [[n, tb["role"], k, f"{v['delta_deg']:+.2f}", f"{math.degrees(v['beta_after']):+.2f}", f"{tb['dCn_per_deg']:.1e}"] for n, tb in tabs.items() for k, v in tb["per_asym"].items()]
    A(md_table(head, rows))
    A("**Option 2: differential centre flap (B45) / differential tail halves (Swift A), a roll trim.** Differential angle (D1 - D2, so half on each side with opposite sign) that zeroes the r_hat numerator Cnb dCl - Clb dCn, from the control response already measured in the main run (flap R/L on B45, tail-half R/L on Swift A), and the residual steady sideslip. Flagged '>10' where the angle is beyond the range the existing runs support (+-5 deg each side). Low-Re flap and tail effectiveness is below AVL's, so these angles (for example 9.2 deg for B45 asymmetry (b)) are lower bounds. Wing-tip trailing-edge surfaces do not exist in the model, so a wing-tip number cannot be computed from the existing data.\n")
    head = ["config", "asym", "differential deg", "per side deg", "residual sideslip deg", "beyond 10 deg"]
    rows = []
    show = [b45n, swn] + [n for n in rank_pre[:2]] + [n for n in rank_body[:2] if n not in rank_pre[:2]]
    for n in dict.fromkeys(show):
        for k, v in rolls[n].items():
            rows.append([n, k, f"{v['diff_deg']:+.2f}", f"{v['per_side_deg']:+.2f}", f"{v['residual_sideslip_deg']:+.2f}", ">10" if v["beyond_10deg"] else "no"])
    A(md_table(head, rows))
    A("All configurations and asymmetries: `roll_trim.csv`. Both options null the numerator, not the sideslip; the glider then flies straight but slipping. Which to build is Kai's decision.\n")
    # ------------------------------------------------------------------ trim CL diagnosis
    A("### Why does the trim CL rise with dihedral at constant static margin? (diagnosis)\n")
    head = ["config", "Xnp mm", "CLa /rad", "Cm about NP (z-ref 0)", "SM % MAC", "CL = CmNP/SM", "CG height modelled mm", "CL_trim (CG height modelled)", "CL_trim (CG height 0)", "CL_trim (baseline CG height)"]
    rows = [[r["config"], f"{r['xnp_mm']:.2f}", f"{r['CLa_per_rad']:.3f}", f"{r['Cm_about_NP_alpha0']:+.5f}", f"{r['SM_pct']:.2f}", f"{r['CL_trim_pred_CmNP_over_SM']:.3f}", f"{r['zcg_mm_modelled']:+.2f}", f"{r['CL_trim_AVL_zcg_modelled']:.3f}", f"{r['CL_trim_AVL_zcg0']:.3f}", f"{r['CL_trim_AVL_zcg_baseline']:.3f}"] for r in tdiag]
    A(md_table(head, rows))
    A(tdiag_text(tdiag))
    # ------------------------------------------------------------------ Kai post-hoc details
    A("### Post-hoc comparison with Kai's reported turns, details\n")
    A(f"Glide height {ph[b45n]['H'][0]:.1f}-{ph[b45n]['H'][1]:.1f} m, path {ph[b45n]['path'][0]:.0f}-{ph[b45n]['path'][1]:.0f} m, R band {ph[b45n]['R_min']:.1f}-{ph[b45n]['R_max']:.1f} m (glide angle {POSTHOC['glide_deg']:g} deg). Time-domain turns per 1 deg of (a), corrected matrix:\n")
    head = ["glider", "release", "glide time s", "turns per 1 deg", "bank at end per 1 deg", "deg for 1 turn", "deg for 2 turns"]
    rows = [[n, lab, f"{o['T']:.1f}", f"{o['turns_per_deg']:.3f}", f"{o['bank_deg']:.1f}", f"{o['deg_for_1_turn']:.3f}", f"{o['deg_for_2_turns']:.3f}"] for n in (b45n, swn) for lab, o in zip(("25 m", "30 m"), td[n])]
    A(md_table(head, rows))
    A("Plausible for hand-built foamboard (fractions of a degree to a couple of degrees = plausible): wing-half incidence 0.1-0.3 deg yes; one-fin toe and B45 flap-hinge differences yes; Swift A tail-half bend difference of several degrees: borderline to unlikely for 'slightly more on one side'. The model has not been checked against any measured turn.\n")
    A("### Glide-angle check at -11 deg (post-hoc)\n")
    A("The glide angle of the real gliders has not been measured. The sensitivity band above (-5 and -8 deg) was chosen around the -6.61 deg of the oscillation model; the swoop model (`swoop_model_v1`) gives about -11 deg (L/D about 6), which lies outside that band. The same pipeline was rerun at -11 deg in `results/runs/rigid_glider/shape_sweep_v1/gamma11_check.csv` (values copied below):\n")
    A(md_table(["quantity", "gamma -6.61 deg", "gamma -11 deg"], [
        ["B45 as flown, spiral eigenvalue 1/s", "+0.0648 (unstable)", "-0.0482 (stable)"],
        ["Swift A as flown, spiral eigenvalue 1/s", "-0.0518", "-0.1556"],
        ["B45 as flown, oscillation zeta", "+0.061", "+0.049"],
        ["Swift A as flown, oscillation zeta", "+0.039", "+0.027"],
        ["B45 degrees for R = 12-32 m scale, (a) / (b) / (c), tw30 columns of the check file", "0.319 / 0.035 / 0.646", "0.235 / 0.026 / 0.477"],
        ["Swift A same scale, (a) / (b)", "1.505 / 0.047", "4.500 / 0.139"]]))
    A("Consequences: (1) whether the B45 spiral is unstable, and therefore whether dihedral is needed to cure it, depends on the glide angle; it is a conditional statement, not a finding, until the glide angle is measured. (2) The degrees of asymmetry needed for a given circle change by a factor of about 1.4 (B45) to 3 (Swift A) between -6.61 and -11 deg, so the order-of-magnitude reading above stands but the individual numbers do not. (3) The sign of Cnb (loss of directional stability of the Swift A from 6 deg of dihedral) does not depend on the glide angle; the oscillation damping of the dihedral variants was not rerun at -11 deg.\n")
    A("### Limits\n")
    A("- AVL is inviscid, flat plate, no thickness; real fins and tabs at Re of a few thousand to 30,000 give less than AVL. Inertias and masses ASSUMED; asymmetry sizes ASSUMED; the results are linear in the asymmetries only up to bank about 30 deg.\n- AVL's base state: constant speed, thrust balancing drag implicitly, no nonlinear effects (stall); the glide angle gamma is an input (MODEL for Swift, ASSUMED for B45), sensitivity -5 and -8 deg above.\n- The product of inertia is from point masses, other products are 0; the stick height (z = -6 mm, ASSUMED) sets the CG height, which drives the trim-CL behaviour diagnosed above.\n- Where the denominator D is near zero (an unstable or slow spiral: B45 baseline, B45 with fins x1.5 at 3 deg, centre-fin Swift at gamma = -5) the steady-turn r_hat is hypersensitive; the time-domain numbers are the better guide there.\n- Phase 1 only: lateral side. No CAD, no STL, no pitch simulation.\n")
    A("### Files\n")
    A("`configs.csv`, `turn_sensitivity.csv` and `crosscheck.csv` (FIRST PASS: theta0 = 0, pre-stated and tan-alpha forms), `post_modes.csv`, `post_turns.csv`, `post_crosscheck.csv`, `roll_trim.csv`, `trim_tab.csv`, `trim_cl_diagnosis.csv`, `posthoc_turns.csv`, `runs.json`, `index.html`, `avl_work/`, `prestated_section.sha256`. Code: `rigid_glider/straight_smooth_v1.py`; tests `rigid_glider/test_straight_smooth_v1.py`.\n")
    return "\n".join(L)


def tdiag_text(rows):
    by = {r["config"]: r for r in rows}
    out = ["Decomposition of the change in trim CL from 0 to 12 deg (AVL, from the table above):\n"]
    for nm, lab in (("B45_fin1", "B45"), ("SwiftA_nofin", "Swift A")):
        a, b = by[f"{nm}_d0"], by[f"{nm}_d12"]
        tot = b["CL_trim_AVL_zcg_modelled"] - a["CL_trim_AVL_zcg_modelled"]
        np_term = b["CL_trim_pred_CmNP_over_SM"] - a["CL_trim_pred_CmNP_over_SM"]
        height = (b["CL_trim_AVL_zcg_modelled"] - b["CL_trim_AVL_zcg0"]) - (a["CL_trim_AVL_zcg_modelled"] - a["CL_trim_AVL_zcg0"])
        rest = b["CL_trim_AVL_zcg0"] - a["CL_trim_AVL_zcg0"] - np_term
        out.append(f"- {lab}: total {tot:+.3f} = moment-about-the-NP term {np_term:+.3f} (Cm about the NP falls {100 * (b['Cm_about_NP_alpha0'] / a['Cm_about_NP_alpha0'] - 1):+.0f} %, SM held) + change of the modelled CG height above the wing plane {height:+.3f} (CG height {a['zcg_mm_modelled']:+.1f} -> {b['zcg_mm_modelled']:+.1f} mm) + remainder with the CG on the reference plane {rest:+.3f}.")
    out.append("")
    out.append("So the rise is NOT a lift-tilt or static-margin effect (SM is held and the NP moment falls). For B45 the largest part is the CG height (for Swift A the CG height and the remainder are comparable): the dihedral lifts the wing-half masses, the point-mass CG moves above the wing plane, and at alpha of 5-9 deg the axial force acting at that height adds a nose-up moment the trim has to cancel. That part depends on the ASSUMED stick position (z = -6 mm) and the point-mass model, so its size is model-dependent. "
               "The remainder (non-zero even with the CG forced onto the reference plane, larger for the Swift A) is not isolated here; the likely cause is the same axial-force/height coupling acting at the raised wing halves, but that is an inference, not a test. A 're-trim needed' statement is therefore conditional: the real CG height and the real flap/tail effectiveness decide it, and the model alone does not settle it.\n")
    return "\n".join(out)


def finish_readme(out, res, base, ccs, tabs, ph, post, rolls, tdiag, wall, ranks):
    txt = write_readme_results(out, res, base, ccs, tabs, ph, post, rolls, tdiag, wall, ranks)
    rp = out / "README.md"
    raw = rp.read_bytes()
    marker = b"\nRESULTS\n"
    head = raw.split(marker)[0]
    rp.write_bytes(head + marker + txt.encode("utf-8") + b"\n")

# =============================================================================================== page (README results rendered dark) and main
def md_to_html(md):
    import html as _h
    inline = lambda s: re.sub(r"`([^`]+)`", r"<code>\1</code>", re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", _h.escape(s)))
    out, lines, i = [], md.split("\n"), 0
    while i < len(lines):
        ln = lines[i]
        if ln.startswith("|"):
            rows = []
            while i < len(lines) and lines[i].startswith("|"):
                rows.append([c.strip() for c in lines[i].strip().strip("|").split("|")])
                i += 1
            out.append("<div class='scroll'><table><tr>" + "".join(f"<th>{inline(c)}</th>" for c in rows[0]) + "</tr>" + "".join("<tr>" + "".join(f"<td>{inline(c)}</td>" for c in r) + "</tr>" for r in rows[2:]) + "</table></div>")
            continue
        if ln.startswith("- ") or re.match(r"^\d+\. ", ln):
            tag = "ul" if ln.startswith("- ") else "ol"
            items = []
            while i < len(lines) and (lines[i].startswith("- ") or re.match(r"^\d+\. ", lines[i])):
                items.append(re.sub(r"^(- |\d+\. )", "", lines[i]))
                i += 1
            out.append(f"<{tag}>" + "".join(f"<li>{inline(x)}</li>" for x in items) + f"</{tag}>")
            continue
        if ln.startswith("### "):
            out.append(f"<h2>{inline(ln[4:])}</h2>")
        elif ln.strip():
            out.append(f"<p>{inline(ln)}</p>")
        i += 1
    return "\n".join(out)


def write_page(out: Path, results_md):
    secs = re.split(r"(?m)^(?=### )", results_md)
    intro, body = secs[0], secs[1:]
    summ = [s for s in body if s.startswith("### Summary for Kai")]
    rest = [s for s in body if not s.startswith("### Summary for Kai")]
    note = ("<p class='note'>Straight and smooth, phase 1 (lateral / turn side). MODEL = inviscid AVL 3.52, flat plate. Nothing here is a flight result. The pre-stated question, method and rule are in "
            "<a href='README.md'>README.md</a> (hash-checked, unchanged). Raw per-configuration numbers: the CSV files in this folder.</p>")
    html_doc = (f"<!doctype html><html lang='en'><head><meta charset='utf-8'><title>Straight and smooth, phase 1</title><style>{DARK_CSS}</style></head><body><main><h1>Straight and smooth, phase 1: lateral / turn side</h1>{note}"
                f"{md_to_html(''.join(summ))}{md_to_html(intro)}{md_to_html(''.join(rest))}</main></body></html>")
    (out / "index.html").write_text(html_doc, encoding="utf-8")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(OUT))
    ap.add_argument("--only", default=None)
    ap.add_argument("--no-tabs", action="store_true")
    ap.add_argument("--render", action="store_true", help="re-render README results / CSV / page from avl_work/state.pkl without running AVL")
    ap.add_argument("--append-tests", default=None, help="text file with the test summary to append to the README results and the page")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    if a.append_tests:
        rp = out / "README.md"
        raw = rp.read_bytes()
        marker = b"\nRESULTS\n"
        head, res_part = raw.split(marker)[0], raw.split(marker)[1].decode("utf-8")
        res_part = res_part.split("### Tests (")[0].rstrip("\n") + "\n\n" + Path(a.append_tests).read_text(encoding="utf-8")
        rp.write_bytes(head + marker + res_part.encode("utf-8"))
        write_page(out, res_part)
        print("appended tests")
        return
    t0 = time.time()
    if a.render:
        res, base, ccs, tabs, ph, post, rolls, tdiag, rank_pre, rank_body, wall = pickle.load(open(out / "avl_work" / "state.pkl", "rb"))
    else:
        res, base = run_grid(out / "avl_work", only=a.only.split(",") if a.only else None)
        ccs = {n: crosscheck(r) for n, r in res.items()}
        tabs = {}
        rank_pre, rank_body = pick_tab_candidates(res)
        chosen = list(dict.fromkeys(rank_pre[:2] + rank_body[:2]))
        extra = [base["b45"]["name"], "SwiftA_centrefin_d0"]
        if not a.no_tabs:
            for n in chosen + [e for e in extra if e not in chosen and e in res]:
                tabs[n] = tab_analysis(res[n]["cfg"], res[n], out / "avl_work")
                tabs[n]["role"] = "best (pre-stated ranking)" if n in rank_pre[:2] else ("best (body-rhat ranking)" if n in rank_body[:2] else "EXTRA (baseline with fin, not in the pre-stated list)")
        ph = {n: posthoc(res, n) for n in (base["b45"]["name"], base["sw"]["name"])}
        post = {n: post_analysis(r["cfg"], r, out / "avl_work") for n, r in res.items()}
        rolls = {n: roll_trims(r, tabs) for n, r in res.items()}
        tdiag = diagnose_trim(res, out / "avl_work")
        wall = time.time() - t0
        pickle.dump((res, base, ccs, tabs, ph, post, rolls, tdiag, rank_pre, rank_body, wall), open(out / "avl_work" / "state.pkl", "wb"))
    cfgs, turns = cfg_rows(res), turn_rows(res)
    for tr in turns:
        pass
    write_csv(out / "configs.csv", cfgs)
    write_csv(out / "turn_sensitivity.csv", turns)
    cc_rows = [{k: (round(v, 6) if isinstance(v, float) else v) for k, v in c.items()} for c in ccs.values()]
    write_csv(out / "crosscheck.csv", cc_rows)
    tab_rows = []
    for n, tb in tabs.items():
        for k, v in tb["per_asym"].items():
            tab_rows.append(dict(config=n, role=tb["role"], asymmetry=k, tab_sign_raw=tb["tab_sign"], dCl_per_deg=round(tb["dCl_per_deg"], 7), dCn_per_deg=round(tb["dCn_per_deg"], 7),
                                 tab_deflection_deg_TE_toward_plus_y=round(v["delta_deg"], 2), rhat_before=round(v["rhat_before"], 6), rhat_after_avl=round(v["rhat_after_avl"], 8),
                                 heading_deg_per_100m_before=round(v["hdg_before"], 1), heading_deg_per_100m_after=round(v["hdg_after"], 3), residual_sideslip_deg=round(math.degrees(v["beta_after"]), 3)))
    if tab_rows:
        write_csv(out / "trim_tab.csv", tab_rows)
    ph_rows = []
    for n, p in ph.items():
        for k, v in p["per_asym"].items():
            ph_rows.append(dict(config=n, asymmetry=k, R_band_m=f"{p['R_min']:.1f}-{p['R_max']:.1f}", R_for_1deg_prestated=round(v["R_1deg_pre"], 1), R_for_1deg_firstpass_tanalpha=round(v["R_1deg_body"], 2),
                                deg_for_Rmin_prestated=round(v["size_deg_for_Rmin_pre"], 2), deg_for_Rmax_prestated=round(v["size_deg_for_Rmax_pre"], 2), note="FIRST PASS (theta0=0); per-1-deg radii out of linear range are raw, see post_turns.csv"))
    write_csv(out / "posthoc_turns.csv", ph_rows)
    # new post-hoc CSVs
    pm, pt, pcx = [], [], []
    for n, d in post.items():
        for tag, e in d.items():
            m, sh = e["modes"], e["shape"] or {}
            pm.append(dict(config=n, case=tag, gamma_deg=e["gamma"], ixz_used=int(e["ixz_used"]), Jxz_kgm2=e["jxz"], spiral_eig=round(m["spiral"], 5), spiral=t2_text(m["spiral"]), roll_eig=round(m["roll"], 3),
                           osc_wn=round(m["dr_wn"], 3), osc_zeta=round(m["dr_zeta"], 4), osc_label=e["osc_label"], beta_over_phi=round(sh.get("beta_over_phi", float("nan")), 3), p_over_phi=round(sh.get("p_over_phi", float("nan")), 3),
                           max_real_eig=round(e["max_real_eig"], 4)))
            for k, v in e["asym"].items():
                pt.append(dict(config=n, case=tag, gamma_deg=e["gamma"], asym=k, rhat_heading=v["rhat_h"], rhat_stability_axis=v["rhat_s"], D_eff=v["D"], sideslip_deg=math.degrees(v["beta"]), bank_deg=v["bank_deg"],
                               out_of_linear_range=int(v["out_of_linear_range"]), R400_m=v["R400"], R250_m=v["R250"], heading_deg_per_100m_400=v["hdg400"], heading_deg_per_100m_250=v["hdg250"],
                               deg_needed_for_R31_7m=v["size_for_R32"], deg_needed_for_R12_4m=v["size_for_R12"]))
            cc = e["crosscheck"]
            pcx.append(dict(config=n, case=tag, **{k: v for k, v in cc.items()}))
    write_csv(out / "post_modes.csv", pm)
    write_csv(out / "post_turns.csv", pt)
    write_csv(out / "post_crosscheck.csv", pcx)
    write_csv(out / "roll_trim.csv", [dict(config=n, asym=k, **v) for n, d in rolls.items() for k, v in d.items()])
    write_csv(out / "trim_cl_diagnosis.csv", tdiag)
    slim = {n: {k: v for k, v in r.items() if k not in ("A4", "B4")} for n, r in res.items()}
    post_slim = {n: {tag: {k: v for k, v in e.items() if k not in ("A4", "B4", "psi")} for tag, e in d.items()} for n, d in post.items()}
    (out / "runs.json").write_text(json.dumps(_json_safe(dict(configs=slim, base=base, crosscheck=ccs, tabs=tabs, posthoc=ph, post=post_slim, roll_trims=rolls, trim_cl_diagnosis=tdiag, wall_s=wall,
                                                              tab_rank_pre=rank_pre[:4], tab_rank_body=rank_body[:4])), indent=1), encoding="utf-8")
    finish_readme(out, res, base, ccs, tabs, ph, post, rolls, tdiag, wall, (rank_pre, rank_body))
    rp = (out / "README.md").read_bytes().split(b"\nRESULTS\n")[1].decode("utf-8")
    write_page(out, rp)
    print(f"done in {wall:.0f} s; wrote {out}")


if __name__ == "__main__":
    main()
