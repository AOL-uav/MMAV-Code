"""Folding concept drawings on the swift v4 outline A (400 mm foam build): two concepts from research/birds/folding_mechanisms.md.

   py -B fold_concepts_v4.py

  (1) geared single-pivot swing wing, band-opened (shortlist C1)
  (2) reversed string: two pieces per side (arm + hand), spring-open, held folded by the carrier (shortlist B2 cut to two pieces)

Reads: the v4 geometry (imported from bird_outline_v4_swift, which is NOT re-run), v4's existing AVL strip-force file for A (CL 0.30), the v4 outlines.csv.
No AVL runs.  Writes results/runs/rigid_glider/fold_concepts_v4/: index.html (dark), drawings.pdf (white, Letter), numbers.csv, numbers.json, the SVG drawings.
Everything is concept geometry and arithmetic: nothing was built or flown.  Standard library only.
"""
from __future__ import annotations
import sys
sys.dont_write_bytecode = True
import csv
import html
import json
import math
import subprocess
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import bird_outline_v2 as v2
import bird_outline_v4_swift as v4
import flat_test as ft

ROOT = ft.ROOT
OUT = ROOT / "results/runs/rigid_glider/fold_concepts_v4"
K = 400.0                                  # span in mm: the foam 400 build
RHO, GRAV = 1.225, 9.81
PLA = ft.PLA_DENSITY_G_CM3                 # ASSUMED, as v4
STEEL = 7.85                               # g/cm3 (ASSUMED handbook value)
FOAM_AREAL = ft.FOAM_AREAL_MASS_G_M2       # ASSUMED, as v4
T_FOAM = v2.SIZES["foam"]["thickness_mm"]
T_PLA = v2.SIZES["printed"]["thickness_mm"]
T_PRINT = 1.0                              # printed hardware thickness, mm (JUDGEMENT)
PIN_D, PIN_L = 1.0, 12.0                   # steel wire pin, mm (ASSUMED)
X_P, Y_P = 8.0, 12.0                       # shoulder pin position, mm (x aft of the root leading edge, y from the centreline) (JUDGEMENT)
WRIST_DX = 8.0                             # wrist pin: this far aft of the wing leading edge at the wrist (JUDGEMENT)
LAP = 12.0                                 # hand-over-arm lap at the wrist, mm (research file 3.4: at least 10 to 15)
SF = 2.0                                   # hold-open safety factor (JUDGEMENT)
N_PULL = 4.0                               # pull-out load factor (the research file's guess, ASSUMED)
CD0 = 0.06                                 # flat-plate profile drag coefficient at Re 30,000 (ASSUMED; AVL has only induced drag)
BAND_K = 0.2                               # N/mm, small rubber band (ASSUMED)
BAND_MAX_STRETCH = 2.5
GRID = 1.0                                 # mm, point-sampling grid for areas


# =============================================================================================== geometry (mm)
def geometry():
    g = v4.build_geometry4(v4.GLIDE)
    ys = [y * K for y in g["y_nodes"]]
    le = [x * K for x in g["le_nodes"]]
    te = [x * K for x in g["te_nodes"]]
    yb, yw, s = g["y_body"] * K, g["y_wrist"] * K, g["half_span"] * K
    yB = yb * 2.0                                                     # root blend ends here (blend = 1.0)

    def at(y, arr):
        for i in range(len(ys) - 1):
            if ys[i] - 1e-9 <= y <= ys[i + 1] + 1e-9:
                t = (y - ys[i]) / (ys[i + 1] - ys[i])
                return arr[i] + t * (arr[i + 1] - arr[i])
        return arr[-1]

    def piece(y0, y1):
        yy = [y0] + [y for y in ys if y0 + 1e-9 < y < y1 - 1e-9] + [y1]
        return [(at(y, le), y) for y in yy] + [(at(y, te), y) for y in reversed(yy)]
    R = [(x * K, y * K) for x, y in g["half_outline"]]
    nose = []
    for p in R:
        if p[1] > yB + 1e-6:
            break
        nose.append(p)
    i_te = max(i for i, p in enumerate(R) if abs(p[1] - g["wt0"] / 2 * K) < 1e-6 and abs(p[0] - g["te_neck"] * K) < 1e-6)
    tail = R[i_te:]
    lead = [(at(y, le), y) for y in reversed(ys) if yb - 1e-9 <= y < yB - 1e-6]
    half = nose + lead + [(at(yb, te), yb)] + tail
    fixed = half + [(x, -y) for (x, y) in reversed(half[1:-1])]
    wingR = piece(yb, s)
    G = dict(g=g, fixed=fixed, wingR=wingR, armR=piece(yb, yw), handR=piece(yw, s), yb=yb, yw=yw, s=s, le=lambda y: at(y, le), te=lambda y: at(y, te),
             x_h=g["x_h"] * K, tail_end=g["tail_end_x"] * K, nose_x=g["nose_x"] * K, St=g["St"] * K, mac=g["mac"] * K, S=g["S"] * K * K, wt0=g["wt0"] * K)
    return G


def mirror(poly):
    return [(x, -y) for x, y in poly]


def rot(poly, P, phi_deg):
    """Swing a polygon back (tip toward +x, aft) by phi about the vertical axis at P = (x, y)."""
    c, s_ = math.cos(math.radians(phi_deg)), math.sin(math.radians(phi_deg))
    return [(P[0] + (x - P[0]) * c + (y - P[1]) * s_, P[1] - (x - P[0]) * s_ + (y - P[1]) * c) for x, y in poly]


def rot_pt(pt, P, phi):
    return rot([pt], P, phi)[0]


def area(poly):
    return 0.5 * abs(sum(poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1] for i in range(len(poly))))


def centroid(poly):
    a = 0.0
    cx = cy = 0.0
    for i in range(len(poly)):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % len(poly)]
        w = x0 * y1 - x1 * y0
        a += w
        cx += (x0 + x1) * w
        cy += (y0 + y1) * w
    return cx / (3 * a), cy / (3 * a)


def inside(poly, x, y):
    c = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            c = not c
        j = i
    return c


def overlap_runs(a, b, step=GRID):
    """Overlap of two polygons by point sampling: (area mm2, [(x0, y0, w, h) row runs])."""
    x0 = max(min(p[0] for p in a), min(p[0] for p in b))
    x1 = min(max(p[0] for p in a), max(p[0] for p in b))
    y0 = max(min(p[1] for p in a), min(p[1] for p in b))
    y1 = min(max(p[1] for p in a), max(p[1] for p in b))
    if x1 <= x0 or y1 <= y0:
        return 0.0, []
    runs, tot = [], 0
    nx, ny = int((x1 - x0) / step), int((y1 - y0) / step)
    for i in range(nx):
        x = x0 + (i + 0.5) * step
        start = None
        for j in range(ny + 1):
            y = y0 + (j + 0.5) * step
            hit = j < ny and inside(a, x, y) and inside(b, x, y)
            if hit and start is None:
                start = y0 + j * step
            if not hit and start is not None:
                runs.append((x - step / 2, start, step, y0 + j * step - start))
                tot += (y0 + j * step - start) * step
                start = None
    return tot, runs


def extent_y(poly):
    ys_ = [p[1] for p in poly]
    return max(ys_) - min(ys_)


def best_angle(poly, P, lo=20.0, hi=130.0):
    """Swing angle that gives the narrowest lateral extent (the piece lies along the body axis)."""
    best = None
    a = lo
    while a <= hi:
        e = extent_y(rot(poly, P, a))
        if best is None or e < best[0] - 1e-9:
            best = (e, a)
        a += 0.25
    return best[1]


# =============================================================================================== aerodynamic load on one half wing (MODEL: v4's AVL strip file)
FS = ROOT / "results/runs/rigid_glider/bird_outline_v4_swift/avl/A_as_measured/b5/cl03/point_00.fs"


def strip_loads(W_n, S_mm2):
    """Right-half wing strips from AVL (A, 250 mm, bend 5, CL 0.301).  Returns lists in mm at the 400 mm build: y, dL (N), dD_induced (N); q from the weight at CL 0.3."""
    rows, cur, inside_ = [], None, False
    for line in FS.read_text(errors="replace").splitlines():
        s = line.strip()
        if s.startswith("Surface #"):
            cur = " ".join(s.split()[3:])
            inside_ = False
        elif "Strip Forces referred to Strip Area, Chord" in line:
            inside_ = True
        elif inside_ and cur == "Wing":
            q = line.split()
            if len(q) >= 15 and q[0].isdigit():
                rows.append((float(q[2]), float(q[4]), float(q[8]), float(q[10])))       # y, chord, cl, cd (induced)
    rows.sort()
    qd = W_n / (S_mm2 * 1e-6 * 0.3)                                      # dynamic pressure from L = W at CL 0.3 (Pa)
    out = []
    n = len(rows)
    for i, (y, c, cl, cd) in enumerate(rows):
        lo = (rows[i - 1][0] + y) / 2 if i > 0 else y - (rows[1][0] - y) / 2
        hi = (rows[i + 1][0] + y) / 2 if i < n - 1 else y + (y - rows[i - 1][0]) / 2
        a = (c * 4 * 1.6) * 0 + c * (hi - lo)                           # strip area at 250 mm, m2
        a *= 1.6 ** 2                                                    # to the 400 mm build
        out.append((y * 1.6 * 1000.0, qd * cl * a, qd * cd * a, a))
    return out, qd


# =============================================================================================== concept 1 and 2
def solve_concepts(G):
    R = {}
    yb, yw, s = G["yb"], G["yw"], G["s"]
    P1 = (X_P, Y_P)
    wing_mass = lambda poly: area(poly) * 1e-6 * FOAM_AREAL
    # all-up mass and CG of the v4 A foam 400 build
    with (ROOT / "results/runs/rigid_glider/bird_outline_v4_swift/outlines.csv").open(encoding="utf-8") as f:
        rows = {r["outline"][0]: r for r in csv.DictReader(f)}
    M0 = float(rows["A"]["all-up g foam (10% margin)"])
    cg0 = float(rows["A"]["trim CG mm at 10% margin (250)"]) * K / 250.0
    R["base"] = dict(M0=M0, cg0=cg0, wing_g_each=wing_mass(G["wingR"]), wing_area_each=area(G["wingR"]), span=K, length=G["tail_end"] - G["nose_x"], tail_end=G["tail_end"], x_h=G["x_h"], nose_x=G["nose_x"])
    # ---- loads (AVL strips, MODEL)
    W = M0 * 1e-3 * GRAV
    strips, qd = strip_loads(W, G["S"])
    L_half = sum(r[1] for r in strips)
    D_ind = sum(r[2] for r in strips)
    S_half = sum(r[3] for r in strips) * 1e6                                         # mm2 (wing strips only, body strip included at y < yb)
    D_prof = qd * CD0 * S_half * 1e-6
    D_half = D_ind + D_prof

    def moment_arm(y_from, y_to=1e9):
        sel = [r for r in strips if y_from <= r[0] <= y_to]
        Lw = sum(r[1] for r in sel)
        Dw = sum(r[2] + qd * CD0 * r[3] for r in sel)
        arm_L = sum(r[1] * (r[0] - y_from) for r in sel) / Lw if Lw else 0.0
        arm_D = sum((r[2] + qd * CD0 * r[3]) * (r[0] - y_from) for r in sel) / Dw if Dw else 0.0
        return dict(L=Lw, D=Dw, arm_L=arm_L, arm_D=arm_D, M_lift_mNm=Lw * arm_L, M_drag_mNm=Dw * arm_D)       # N x mm = mN.m
    R["loads"] = dict(W=W, q=qd, V=math.sqrt(2 * W / (RHO * G["S"] * 1e-6 * 0.3)), L_half=L_half, D_ind=D_ind, D_prof=D_prof, D_half=D_half,
                      shoulder=moment_arm(Y_P), shoulder_outer=moment_arm(yb), wrist=moment_arm(yw), n_pull=N_PULL)
    # ---- concept 1: one rigid wing per side, one pin at P1, sync gears, one band
    phi1 = best_angle(G["wingR"], P1)
    wf = rot(G["wingR"], P1, phi1)
    tip_le = (G["le"](s), s)
    R["c1"] = dict(phi=phi1, P=P1)
    # ---- concept 2: arm + hand
    Pw = (G["le"](yw) + WRIST_DX, yw)
    phi_a = 0.0
    lo, hi_ = 0.0, 140.0
    for _ in range(60):                                                              # arm angle: the wrist pin lands on y = Y_P (arm axis along the body)
        mid = (lo + hi_) / 2
        if rot_pt(Pw, P1, mid)[1] > Y_P:
            lo = mid
        else:
            hi_ = mid
    phi_a = (lo + hi_) / 2
    Pw_f = rot_pt(Pw, P1, phi_a)
    arm_f = rot(G["armR"], P1, phi_a)
    # hand angle (absolute) = the narrowest extent about the moved wrist pin; the hand polygon is first carried with the arm, then turned about the wrist pin
    hand_c = rot(G["handR"], P1, phi_a)
    best = None
    a = -60.0
    while a <= 70.0:
        e = extent_y(rot(hand_c, Pw_f, a))
        if best is None or e < best[0] - 1e-9:
            best = (e, a)
        a += 0.25
    phi_w = best[1]                                                                  # extra turn about the wrist pin (relative joint angle)
    R["c2"] = dict(phi_sh=phi_a, phi_wr=phi_w, P=P1, Pw=Pw)
    R["geom"] = dict(tip_le=tip_le, wingR_fold=wf)
    R["pose"] = {}
    for name, fn in (("c1", pose_c1), ("c2", pose_c2)):
        R["pose"][name] = {t: fn(G, R, t) for t in (0.0, 0.5, 1.0)}
    return R


def pose_c1(G, R, t):
    P = R["c1"]["P"]
    phi = R["c1"]["phi"] * t
    return dict(t=t, wingR=rot(G["wingR"], P, phi), wingL=mirror(rot(G["wingR"], P, phi)), angles=dict(shoulder=phi), tip=rot_pt((G["le"](G["s"]), G["s"]), P, phi))


def pose_c2(G, R, t):
    P, Pw = R["c2"]["P"], R["c2"]["Pw"]
    a1 = R["c2"]["phi_sh"] * t
    a2 = R["c2"]["phi_wr"] * t
    Pw_t = rot_pt(Pw, P, a1)
    arm = rot(G["armR"], P, a1)
    hand = rot(rot(G["handR"], P, a1), Pw_t, a2)
    tip = rot_pt(rot_pt((G["le"](G["s"]), G["s"]), P, a1), Pw_t, a2)
    return dict(t=t, armR=arm, handR=hand, armL=mirror(arm), handL=mirror(hand), Pw=Pw_t, angles=dict(shoulder=a1, wrist=a2), tip=tip)


def path_of_tip(G, R, concept, n=40, tmax=1.0):
    fn = pose_c1 if concept == "c1" else pose_c2
    return [fn(G, R, tmax * i / n)["tip"] for i in range(n + 1)]


# =============================================================================================== numbers
def envelope(polys):
    xs = [p[0] for pl in polys for p in pl]
    ys = [p[1] for pl in polys for p in pl]
    return min(xs), max(xs), min(ys), max(ys)


def folded_numbers(G, R, concept):
    pose = R["pose"][concept][1.0]
    opened = R["pose"][concept][0.0]
    parts = [("wingR", "wingL")] if concept == "c1" else [("armR", "armL"), ("handR", "handL")]
    moving = [pose[k] for pair in parts for k in pair]
    env_w = envelope(moving)
    env_all = envelope(moving + [G["fixed"]])
    n = dict(stowed_span=env_all[3] - env_all[2], wings_span=env_w[3] - env_w[2], stowed_length=env_all[1] - env_all[0], wings_x_max=env_w[1], tail_end=G["tail_end"],
             tips_past_tail=env_w[1] - G["tail_end"], open_span=K, open_length=G["tail_end"] - G["nose_x"])
    ov = {}
    if concept == "c1":
        a, runsLR = overlap_runs(pose["wingR"], pose["wingL"])
        ov["wingR_wingL"] = (a, runsLR)
        for nm in ("wingR", "wingL"):
            ov[nm + "_body"] = overlap_runs(pose[nm], G["fixed"])
    else:
        for nm in ("R", "L"):
            ov["arm" + nm + "_hand" + nm] = overlap_runs(pose["arm" + nm], pose["hand" + nm])
        ov["handR_handL"] = overlap_runs(pose["handR"], pose["handL"])
        ov["handR_armL"] = overlap_runs(pose["handR"], pose["armL"])
        for nm in ("armR", "handR"):
            ov[nm + "_body"] = overlap_runs(pose[nm], G["fixed"])
    n["overlaps"] = {k: v[0] for k, v in ov.items()}
    n["overlap_runs"] = {k: v[1] for k, v in ov.items()}
    return n


def cg_numbers(G, R, concept, m_hw, hw_pos_open, hw_pos_fold):
    """CG of the glider open and folded, with the hardware included (no re-ballast).  Wing plates: foam areal mass (ASSUMED); the v4 build supplies the rest."""
    b = R["base"]
    pose0, pose1 = R["pose"][concept][0.0], R["pose"][concept][1.0]
    keys = ("wingR", "wingL") if concept == "c1" else ("armR", "armL", "handR", "handL")
    wm = lambda poly: area(poly) * 1e-6 * FOAM_AREAL
    m_w = sum(wm(pose0[k]) for k in keys)
    xo = sum(wm(pose0[k]) * centroid(pose0[k])[0] for k in keys)
    xf = sum(wm(pose1[k]) * centroid(pose1[k])[0] for k in keys)
    M_rest = b["M0"] - m_w
    cg_rest = (b["M0"] * b["cg0"] - xo) / M_rest
    mhw = sum(m for m, _ in hw_pos_open)
    mom_o = sum(m * x for m, x in hw_pos_open)
    mom_f = sum(m * x for m, x in hw_pos_fold)
    Mt = b["M0"] + mhw
    cg_open_plain = b["cg0"]
    cg_open = (b["M0"] * b["cg0"] + mom_o) / Mt
    cg_fold = (M_rest * cg_rest + xf + mom_f) / Mt
    return dict(m_wings=m_w, wings_cg_open=xo / m_w, wings_cg_fold=xf / m_w, cg_open_v4=cg_open_plain, cg_open_hw=cg_open, cg_fold_hw=cg_fold, shift_fold=cg_fold - cg_open,
                shift_hw=cg_open - cg_open_plain, M_total=Mt, m_hw=mhw)


def vol_g(mm3, rho=PLA):
    return mm3 * 1e-3 * rho


def pin_g(d=PIN_D, l=PIN_L):
    return vol_g(math.pi * d * d / 4 * l, STEEL)


def sector_area(r, ang_deg):
    return 0.5 * r * r * math.radians(ang_deg)


def parts_c1(G, R):
    """Part list for concept 1: (name, printed?, count, mass each g, basis)."""
    phi = R["c1"]["phi"]
    gear = vol_g(sector_area(Y_P, phi + 16.0) * T_PRINT)
    lug = vol_g(14.0 * 16.0 * T_PRINT)
    bracket = vol_g(16.0 * 16.0 * T_PRINT) + vol_g(math.pi * 3.0 ** 2 * 2.0)                 # one boss plate 16 x 16 + a pin boss dia 6 x 2
    stops = vol_g(math.pi * 1.5 ** 2 * 4.0)
    eye = vol_g(math.pi * (4.0 ** 2 - 1.5 ** 2) * T_PRINT)
    return [("wing lug + gear sector (printed)", True, 2, gear + lug, f"sector r {Y_P:g} mm x {phi + 16:.0f} deg + 14 x 16 mm lug, {T_PRINT:g} mm PLA"),
            ("pin boss plate on the body strip (printed)", True, 2, bracket, "16 x 16 mm plate + boss dia 6 x 2 mm"),
            ("pivot pin (steel wire)", False, 2, pin_g(), f"dia {PIN_D:g} x {PIN_L:g} mm steel"),
            ("open stop peg (printed)", True, 2, stops, "dia 3 x 4 mm"),
            ("hold eyelet ring (printed)", True, 2, eye, "dia 8 mm ring on each wing"),
            ("rubber band", False, 1, 0.15, "ASSUMED, small band"),
            ("glue / tape for the parts", False, 1, 0.2, "ASSUMED")]


def parts_c2(G, R):
    gear = vol_g(sector_area(Y_P, R["c2"]["phi_sh"] + 16.0) * T_PRINT)
    lug = vol_g(14.0 * 16.0 * T_PRINT)
    bracket = vol_g(16.0 * 16.0 * T_PRINT) + vol_g(math.pi * 3.0 ** 2 * 2.0)
    pad = vol_g(math.pi * 7.5 ** 2 * T_PRINT)
    stops = vol_g(math.pi * 1.5 ** 2 * 4.0)
    guide = vol_g(math.pi * (3.0 ** 2 - 0.6 ** 2) * 2.0)
    return [("arm lug + gear sector (printed)", True, 2, gear + lug, f"as concept 1"),
            ("pin boss plate on the body strip (printed)", True, 2, bracket, "as concept 1"),
            ("wrist pad, one per plate (printed)", True, 4, pad, f"dia 15 x {T_PRINT:g} mm, two plates per wrist"),
            ("pins, shoulder + wrist (steel wire)", False, 4, pin_g(), f"dia {PIN_D:g} x {PIN_L:g} mm steel"),
            ("stop pegs, shoulder + wrist (printed)", True, 4, stops, "dia 3 x 4 mm"),
            ("string guides (printed)", True, 6, guide, "dia 6 x 2 mm with 1.2 mm hole"),
            ("fold string, 0.3 m", False, 1, 0.03, "0.1 mm braid, ASSUMED"),
            ("ring for the carrier pin (printed)", True, 1, vol_g(math.pi * (4.0 ** 2 - 1.5 ** 2) * T_PRINT), "dia 8 ring"),
            ("elastic bands: 1 shoulder + 2 wrist", False, 3, 0.1, "ASSUMED, 0.1 g each"),
            ("hold eyelet ring (printed)", True, 2, vol_g(math.pi * (4.0 ** 2 - 1.5 ** 2) * T_PRINT), "dia 8 ring on each hand plate"),
            ("glue / tape for the parts", False, 1, 0.3, "ASSUMED")]


def band_search(G, R):
    """Concept 1 band: one anchor post on the right wing lug (local frame of the wing), one on the body bracket.  Maximise the worst-case opening lever dL/dphi over 0..phi_f."""
    P = R["c1"]["P"]
    phi_f = R["c1"]["phi"]
    best = None
    phis = [phi_f * i / 24 for i in range(25)]
    for dxa in range(-4, 17, 2):
        for dya in range(-8, 17, 2):
            ax0, ay0 = P[0] + dxa, P[1] + dya
            if math.hypot(dxa, dya) < 5.0:
                continue
            for xb in range(0, 31, 2):
                for yb_ in range(-16, 17, 2):
                    Ls = []
                    for ph in phis:
                        a = rot_pt((ax0, ay0), P, ph)
                        Ls.append(math.hypot(a[0] - xb, a[1] - yb_))
                    if min(Ls) < 4.0 or max(Ls) / min(Ls) > BAND_MAX_STRETCH:
                        continue
                    r_eff = [(Ls[i + 1] - Ls[i]) / math.radians(phis[i + 1] - phis[i]) for i in range(len(phis) - 1)]
                    score = min(r_eff)
                    if best is None or score > best[0]:
                        best = (score, dict(local=(dxa, dya), body=(xb, yb_), L=Ls, r_eff=r_eff, phis=phis))
    return best[1] if best else None


def hold_hole(G, R, concept, x_hook):
    """Where the hold-folded pin hole sits in each wing: the point of the wing that lies over the hook point (x_hook, 0) when folded, and whether it is inside the wing plate."""
    P = R["c1"]["P"]
    out = {}
    if concept == "c1":
        phi = R["c1"]["phi"]
        pts = {"wing": rot_pt((x_hook, 0.0), P, -phi)}
        out["wing"] = (pts["wing"], inside(G["wingR"], *pts["wing"]) and margin_ok(G["wingR"], pts["wing"]))
    else:
        a1, a2 = R["c2"]["phi_sh"], R["c2"]["phi_wr"]
        Pw_f = rot_pt(R["c2"]["Pw"], P, a1)
        p_hand = rot_pt((x_hook, 0.0), Pw_f, -a2)
        p_hand = rot_pt(p_hand, P, -a1)
        p_arm = rot_pt((x_hook, 0.0), P, -a1)
        out["hand"] = (p_hand, inside(G["handR"], *p_hand) and margin_ok(G["handR"], p_hand))
        out["arm"] = (p_arm, inside(G["armR"], *p_arm) and margin_ok(G["armR"], p_arm))
    return out


def margin_ok(poly, pt, m=4.0):
    for ang in range(0, 360, 30):
        if not inside(poly, pt[0] + m * math.cos(math.radians(ang)), pt[1] + m * math.sin(math.radians(ang))):
            return False
    return True


def string_route(G, R):
    """Concept 2 fold string: tip leading edge -> wrist guide -> shoulder guide -> ring on the body centreline.  Path length open and folded."""
    P, Pw = R["c2"]["P"], R["c2"]["Pw"]
    ring = (G["x_h"] - 3.0, 0.0)                                      # the ring sits on the hook point: one carrier pin through both
    tip = (G["le"](G["s"]) + 1.0, G["s"] - 1.0)
    gw = (G["le"](G["yw"]) + 1.5, G["yw"])
    gs = (24.0, Y_P)                                                  # printed guide on the body strip, aft of the shoulder pin (JUDGEMENT)
    lens = {}
    for t in (0.0, 0.5, 1.0):
        a1 = R["c2"]["phi_sh"] * t
        a2 = R["c2"]["phi_wr"] * t
        gw_t = rot_pt(gw, P, a1)
        Pw_t = rot_pt(Pw, P, a1)
        tip_t = rot_pt(rot_pt(tip, P, a1), Pw_t, a2)
        gw_t2 = rot_pt(gw_t, Pw_t, 0.0)
        gs_t = rot_pt(gs, P, 0.0)
        pts = [tip_t, gw_t2, gs_t, ring]
        lens[t] = (sum(math.dist(pts[i], pts[i + 1]) for i in range(3)), pts)
    return dict(ring=ring, tip=tip, gw=gw, gs=gs, lens=lens, travel=lens[0.0][0] - lens[1.0][0])


# =============================================================================================== drawing primitives (one description, two themes)
THEMES = {
    "dark": dict(bg="#14181d", body="#27313c", body_s="#8fa0b0", wingR="#3b78b4", wingL="#3f9a6c", arm="#3b78b4", hand="#6fa5d8", armL="#3f9a6c", handL="#78c79c", text="#dde3ea", muted="#9aa7b4", pivot="#ff9f6b",
                 path="#e89a55", over="#ff5da2", hold="#6cc08b", printed="#e89a55", shelf="#7fb4ff", dim="#c9d3dc", grid="#333c46", fold="#c39bff", wing_op=0.78),
    "white": dict(bg="#ffffff", body="#e6eaee", body_s="#333333", wingR="#8fb3d9", wingL="#9bcdb0", arm="#8fb3d9", hand="#b5cde8", armL="#9bcdb0", handL="#bfe0cc", text="#000000", muted="#444444", pivot="#c2621a",
                  path="#c2621a", over="#c2185b", hold="#1b7f3f", printed="#f0b27a", shelf="#1f5fbf", dim="#222222", grid="#bbbbbb", fold="#7a3fb0", wing_op=0.85),
}


class Canvas:
    def __init__(self, theme):
        self.th = THEMES[theme]
        self.dark = theme == "dark"
        self.items = []

    def c(self, key):
        return self.th.get(key, key)

    def poly(self, pts, fill="body", stroke="body_s", w=0.4, op=1.0, dash=None, close=True):
        self.items.append(("poly", list(pts), self.c(fill) if fill else None, self.c(stroke) if stroke else None, w, op, dash, close))

    def line(self, a, b, stroke="body_s", w=0.4, dash=None):
        self.items.append(("line", a, b, self.c(stroke), w, dash))

    def text(self, p, s, size=3.2, fill="text", anchor="start", bold=False):
        self.items.append(("text", p, s, size, self.c(fill), anchor, bold))

    def circle(self, c, r, fill=None, stroke="body_s", w=0.4, op=1.0):
        n = 28
        self.poly([(c[0] + r * math.cos(2 * math.pi * i / n), c[1] + r * math.sin(2 * math.pi * i / n)) for i in range(n)], fill=fill, stroke=stroke, w=w, op=op)

    def arrow(self, a, b, stroke="dim", w=0.35, head=1.6):
        self.line(a, b, stroke, w)
        ang = math.atan2(b[1] - a[1], b[0] - a[0])
        for sgn in (1, -1):
            self.line(b, (b[0] - head * math.cos(ang + sgn * 0.45), b[1] - head * math.sin(ang + sgn * 0.45)), stroke, w)

    def dim(self, a, b, label, dx=0.0, dy=-1.2, size=3.0, anchor="middle"):
        self.arrow(a, b)
        self.arrow(b, a)
        self.text(((a[0] + b[0]) / 2 + dx, (a[1] + b[1]) / 2 + dy), label, size, "dim", anchor)

    def svg_dark(self, vb):
        o = [f"<svg xmlns='http://www.w3.org/2000/svg' viewBox='{vb[0]:.1f} {vb[1]:.1f} {vb[2]:.1f} {vb[3]:.1f}' width='100%' style='background:{self.th['bg']};border:1px solid #333c46;border-radius:6px'>"]
        for it in self.items:
            k = it[0]
            if k == "poly":
                _, pts, fill, stroke, w, op, dash, close = it
                tag = "polygon" if close else "polyline"
                a = f"fill='{fill or 'none'}'" + (f" fill-opacity='{op}'" if fill and op < 1 else "") + (f" stroke='{stroke}' stroke-width='{w}'" if stroke else "") + (f" stroke-dasharray='{dash}'" if dash else "")
                o.append(f"<{tag} points='{' '.join(f'{x:.2f},{y:.2f}' for x, y in pts)}' {a}/>")
            elif k == "line":
                _, p, q, stroke, w, dash = it
                o.append(f"<line x1='{p[0]:.2f}' y1='{p[1]:.2f}' x2='{q[0]:.2f}' y2='{q[1]:.2f}' stroke='{stroke}' stroke-width='{w}'" + (f" stroke-dasharray='{dash}'" if dash else "") + "/>")
            else:
                _, p, s, size, fill, anchor, bold = it
                o.append(f"<text x='{p[0]:.2f}' y='{p[1]:.2f}' font-family='Segoe UI,Arial,sans-serif' font-size='{size}' fill='{fill}' text-anchor='{anchor}'" + (" font-weight='bold'" if bold else "") + f">{html.escape(s)}</text>")
        return "".join(o) + "</svg>"

    def pdf_elements(self, dx=0.0, dy=0.0):
        """Elements restricted to line / polygon / text for flat_templates_pdf (open polylines become line segments)."""
        o = []
        for it in self.items:
            k = it[0]
            if k == "poly":
                _, pts, fill, stroke, w, op, dash, close = it
                if not close:
                    for a, b in zip(pts, pts[1:]):
                        o.append(f'<line x1="{a[0] + dx:.3f}" y1="{a[1] + dy:.3f}" x2="{b[0] + dx:.3f}" y2="{b[1] + dy:.3f}" stroke="{stroke or "#000000"}" stroke-width="{w}"' + (f' stroke-dasharray="{dash}"' if dash else "") + "/>")
                    continue
                a = f'fill="{fill or "none"}"' + (f' fill-opacity="{op}"' if fill and op < 1 else "") + (f' stroke="{stroke}" stroke-width="{w}"' if stroke else "") + (f' stroke-dasharray="{dash}"' if dash else "")
                o.append(f'<polygon points="{" ".join(f"{x + dx:.3f},{y + dy:.3f}" for x, y in pts)}" {a}/>')
            elif k == "line":
                _, p, q, stroke, w, dash = it
                o.append(f'<line x1="{p[0] + dx:.3f}" y1="{p[1] + dy:.3f}" x2="{q[0] + dx:.3f}" y2="{q[1] + dy:.3f}" stroke="{stroke}" stroke-width="{w}"' + (f' stroke-dasharray="{dash}"' if dash else "") + "/>")
            else:
                _, p, s, size, fill, anchor, bold = it
                o.append(f'<text x="{p[0] + dx:.3f}" y="{p[1] + dy:.3f}" font-family="Helvetica" font-size="{size}" fill="{fill}"' + (' text-anchor="end"' if anchor == "end" else "") + (' font-weight="bold"' if bold else "") + f">{html.escape(s)}</text>")
        return o


class View:
    """Maps glider millimetres (x aft, y right) to drawing units: nose up, scale s per mm, origin (ox, oy) = the point (x0, 0)."""
    def __init__(self, ox, oy, s, x0=0.0):
        self.ox, self.oy, self.s, self.x0 = ox, oy, s, x0

    def __call__(self, p):
        return (self.ox + p[1] * self.s, self.oy + (p[0] - self.x0) * self.s)

    def poly(self, pts):
        return [self(p) for p in pts]


def runs_poly(runs):
    return [[(x, y), (x + w, y), (x + w, y + h), (x, y + h)] for x, y, w, h in runs]


def draw_state(cv, V, G, R, concept, t, size=3.0, show_path=True, clip=None):
    th = cv.th
    pose = R["pose"][concept][t]
    cv.poly(V.poly(G["fixed"]), "body", "body_s", 0.5)
    order = [("wingL", "wingL"), ("wingR", "wingR")] if concept == "c1" else [("handL", "handL"), ("armL", "armL"), ("handR", "hand"), ("armR", "arm")]
    for key, col in order:
        cv.poly(V.poly(pose[key]), col, "body_s", 0.45, op=th["wing_op"])
    if t > 0:
        a, runs = overlap_runs(pose["wingR" if concept == "c1" else "handR"], pose["wingL" if concept == "c1" else "handL"], 1.5)
        for rp in runs_poly(runs):
            cv.poly(V.poly(rp), "over", None, 0, op=0.55)
    if t == 0:
        for yy in (G["yb"], G["yw"]):
            if concept == "c1" and yy == G["yw"]:
                continue
            for sg in (1, -1):
                cv.line(V((G["le"](yy), sg * yy)), V((G["te"](yy), sg * yy)), "fold", 0.35, "1.5 1.2")
    if show_path and t > 0:
        pth = path_of_tip(G, R, concept, tmax=t)
        ok = (lambda x, y: True) if clip is None else (lambda x, y: clip[0] <= x <= clip[1] and clip[2] <= y <= clip[3])
        for sg in (1, -1):
            run = []
            for x, y in pth:
                if ok(x, sg * y):
                    run.append((x, sg * y))
                else:
                    if len(run) > 1:
                        cv.poly(V.poly(run), None, "path", 0.5, dash="2 1.4", close=False)
                    run = []
            if len(run) > 1:
                cv.poly(V.poly(run), None, "path", 0.5, dash="2 1.4", close=False)
        for sg in (1, -1):
            if ok(pose["tip"][0], sg * pose["tip"][1]):
                cv.circle(V((pose["tip"][0], sg * pose["tip"][1])), 1.1 * size / 3, "path", "path")
    P = R["c1"]["P"]
    for sg in (1, -1):
        cv.circle(V((P[0], sg * P[1])), 1.5 * size / 3, "pivot", "pivot")
        if concept == "c2":
            cv.circle(V((pose["Pw"][0], sg * pose["Pw"][1])), 1.5 * size / 3, "pivot", "pivot")
    if t == 1.0:
        x_hook = G["x_h"] - 3.0
        cv.circle(V((x_hook, 0)), 1.3 * size / 3, "hold", "hold")


def state_extent(G, R, concept, t):
    pose = R["pose"][concept][t]
    keys = ("wingR", "wingL") if concept == "c1" else ("armR", "armL", "handR", "handL")
    return envelope([pose[k] for k in keys] + [G["fixed"]])


# =============================================================================================== analysis: all numbers in one place
def band_search_wrist(G, R):
    """Concept 2 wrist band: post on the arm plate and post on the hand plate (both given in the arm frame at the open pose); the hand turns about the wrist pin by phi_wr (negative = forward)."""
    Pw = R["c2"]["Pw"]
    a2 = R["c2"]["phi_wr"]
    best = None
    steps = [a2 * i / 12 for i in range(13)]
    for ax in range(-16, 17, 4):
        for ay in range(-18, 1, 3):                    # arm post: inboard of the wrist pin
            for hx in range(-16, 17, 4):
                for hy in range(2, 19, 3):             # hand post: outboard of the wrist pin
                    A = (Pw[0] + ax, Pw[1] + ay)
                    H0 = (Pw[0] + hx, Pw[1] + hy)
                    Ls = [math.dist(A, rot_pt(H0, Pw, a)) for a in steps]
                    if min(Ls) < 4 or max(Ls) / min(Ls) > BAND_MAX_STRETCH:
                        continue
                    rr = [(Ls[i + 1] - Ls[i]) / (math.radians(steps[i + 1] - steps[i])) for i in range(12)]
                    # folding means the angle goes from 0 to a2 (negative): the band must lengthen, so dL/da must be negative here
                    score = -max(rr)
                    if best is None or score > best[0]:
                        best = (score, dict(arm=A, hand=H0, L=Ls, r_eff=[-r for r in rr], steps=steps))
    return best[1] if best and best[0] > 0 else (best[1] if best else None)


def analyse(G, R):
    N = {"hook": G["x_h"] - 3.0, "ld": R["loads"]}
    L = R["loads"]
    sh = L["shoulder"]
    wr = L["wrist"]
    m_sh = SF * N_PULL * sh["M_drag_mNm"]                          # per wing, mN.m
    m_wr = SF * N_PULL * wr["M_drag_mNm"]
    band1 = band_search(G, R)
    r_eff0 = band1["r_eff"][0]
    T1 = 2 * m_sh / r_eff0                                          # one band drives both wings through the gears
    k = BAND_K
    L0, Lf = band1["L"][0], band1["L"][-1]
    L_free = L0 - T1 / k
    T_fold = k * (Lf - L_free)
    band_info = dict(local=band1["local"], body=band1["body"], r_eff_open=r_eff0, r_eff_min=min(band1["r_eff"]), L_open=L0, L_fold=Lf, stretch=Lf / L_free if L_free > 0 else float("nan"), T_open=T1, T_fold=T_fold,
                     L_free=L_free, M_req_per_wing=m_sh, M_total=2 * m_sh)
    wband = band_search_wrist(G, R)
    x_hook = N["hook"]
    sr = string_route(G, R)
    for c in ("c1", "c2"):
        n = folded_numbers(G, R, c)
        parts = parts_c1(G, R) if c == "c1" else parts_c2(G, R)
        holes = hold_hole(G, R, c, x_hook)
        P = R["c1"]["P"]
        mt = sum(cnt * m for _, _, cnt, m, _ in parts)
        mp = sum(cnt * m for _, pr, cnt, m, _ in parts if pr)
        # hardware positions (open, folded) for the CG shift
        hw_o, hw_f = [], []
        phi = R["c1"]["phi"] if c == "c1" else R["c2"]["phi_sh"]
        lug_o = (P[0] + 4.0, P[1] + 8.0)
        lug_f = rot_pt(lug_o, P, phi)
        for name, pr, cnt, m, why in parts:
            tot = cnt * m
            if "lug" in name:
                hw_o.append((tot, lug_o[0]))
                hw_f.append((tot, lug_f[0]))
            elif "eyelet" in name:
                key = "wing" if c == "c1" else "hand"
                hw_o.append((tot, holes[key][0][0]))
                hw_f.append((tot, x_hook))
            elif "wrist pad" in name or "wrist" in name and "band" in name:
                pw_o, pw_f = R["c2"]["Pw"], R["pose"]["c2"][1.0]["Pw"]
                hw_o.append((tot, pw_o[0]))
                hw_f.append((tot, pw_f[0]))
            else:
                hw_o.append((tot, P[0] + 6.0))
                hw_f.append((tot, P[0] + 6.0))
        cgn = cg_numbers(G, R, c, mt, hw_o, hw_f)
        N[c] = dict(n=n, parts=parts, m_hw=mt, m_printed=mp, holes=holes, cg=cgn, count=sum(cnt for _, _, cnt, _, _ in parts))
        N[c]["hang"] = dict(x_hook=x_hook, cg_open=cgn["cg_open_hw"], cg_fold=cgn["cg_fold_hw"], margin_open=x_hook - cgn["cg_open_hw"], margin_fold=x_hook - cgn["cg_fold_hw"])
    N["c1"]["band"] = band_info
    N["c1"]["band_raw"] = band1
    N["c2"]["wrist_band"] = wband
    N["c2"]["m_sh_req"] = m_sh
    N["c2"]["m_wr_req"] = m_wr
    N["c2"]["string"] = sr
    N["c2"]["F_sh"] = m_sh / 8.0                                      # N at an 8 mm lever (JUDGEMENT)
    N["c2"]["F_wr"] = m_wr / 8.0
    return N


# =============================================================================================== mechanism sketches
def rect_pts(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def sector_pts(P, r, a0, a1, mirror_=False, n=14):
    out = [P]
    for i in range(n + 1):
        a = math.radians(a0 + (a1 - a0) * i / n)
        out.append((P[0] + r * math.sin(a), P[1] + (1 if mirror_ else -1) * r * math.cos(a)))
    return out


def callout(cv, lines, target, x_du, y_du, size, color="text", anchor="start"):
    """Label (list of lines) at drawing position (x_du, y_du) with a leader to the target (drawing units)."""
    for i, ln in enumerate(lines):
        cv.text((x_du, y_du + i * size * 1.15), ln, size, color, anchor)
    w = max(len(l) for l in lines) * size * 0.5
    ex = x_du - 1.0 if anchor == "start" else x_du + 1.0
    cv.line(target, (ex, y_du - size * 0.3), "muted", 0.35)


def mech_c1(theme, s, size, N, G, R):
    cv = Canvas(theme)
    P = R["c1"]["P"]
    phi_f = R["c1"]["phi"]
    yh, x0, x1 = 80.0, -16.0, 66.0
    pad = 3 * size
    top = 4.5 * size
    V = View(yh * s + pad, top + pad, s, x0)
    W_, H_ = 2 * yh * s + 2 * pad, (x1 - x0) * s + top + 2 * pad + 5 * size
    cv.text((pad, size * 1.8), "Concept 1: geared swing wing, band-opened: root of the glider, open pose, nose up", size * 1.1, "text", bold=True)
    for sg in (1, -1):
        ys_ = [G["yb"] + i for i in range(0, 15, 2)]
        pts = [(G["le"](y), sg * y) for y in ys_] + [(G["te"](y), sg * y) for y in reversed(ys_)]
        cv.poly(V.poly(pts), "wingR" if sg > 0 else "wingL", "body_s", 0.4, op=0.4)
    cv.poly(V.poly([(x0, -G["yb"]), (G["x_h"] + 4, -G["yb"])]), None, "body_s", 0.5, dash="3 2", close=False)
    cv.poly(V.poly([(x0, G["yb"]), (G["x_h"] + 4, G["yb"])]), None, "body_s", 0.5, dash="3 2", close=False)
    cv.line(V((x0, 0)), V((x1, 0)), "grid", 0.4, "1 2")
    for sg_ in (1, -1):
        cv.poly(V.poly([(P[0] + dx, sg_ * P[1] + dy) for dx, dy in rect_pts(-8.0, -8.0, 8.0, 8.0)]), "printed", "body_s", 0.4, op=0.3)
    for sg in (1, -1):
        Pp = (P[0], sg * P[1])
        cv.poly(V.poly(sector_pts(Pp, P[1], -8.0, phi_f + 8.0, mirror_=(sg < 0))), "printed", "body_s", 0.4, op=0.9)
        for i in range(-1, int((phi_f + 8) / 6) + 1):
            a = math.radians(6.0 * i)
            sgn = 1 if sg < 0 else -1
            cv.line(V((Pp[0] + P[1] * math.sin(a), Pp[1] + sgn * P[1] * math.cos(a))), V((Pp[0] + (P[1] + 1.4) * math.sin(a), Pp[1] + sgn * (P[1] + 1.4) * math.cos(a))), "body_s", 0.5)
        cv.poly(V.poly([(Pp[0] + dx, Pp[1] + sg * dy) for dx, dy in rect_pts(-4.0, 0.0, 10.0, 16.0)]), "printed", "body_s", 0.4, op=0.9)
        cv.circle(V(Pp), 3.0 * s, "printed", "body_s", 0.5)
        cv.circle(V(Pp), 0.5 * PIN_D * s + 0.3, "shelf", "shelf", 0.4)
        cv.circle(V((P[0] - 5.5, sg * (P[1] + 8.0))), 1.5 * s, "printed", "body_s", 0.5)
    b = N["c1"]["band_raw"]
    A0 = (P[0] + b["local"][0], P[1] + b["local"][1])
    B = (float(b["body"][0]), float(b["body"][1]))
    Af = rot_pt(A0, P, phi_f)
    cv.line(V(A0), V(B), "shelf", 1.3)
    cv.line(V(Af), V(B), "shelf", 0.9, "3 2")
    for q, r_ in ((A0, 1.1), (B, 1.1)):
        cv.circle(V(q), r_ * s, "shelf", "shelf")
    cv.circle(V(Af), 0.9 * s, None, "shelf", 0.5)
    hook = (N["hook"], 0.0)
    cv.circle(V(hook), 1.6 * s, "hold", "hold")
    xl = W_ / 2 - 38 * s
    xr = W_ / 2 + 38 * s
    sz = size * 0.95
    rows = lambda x_mm: V((x_mm, 0))[1]
    callout(cv, ["pivot pin, steel wire", "dia %.1f x %.0f mm (off the shelf)" % (PIN_D, PIN_L)], V((P[0], -P[1])), xl, rows(-10), sz, "shelf", "end")
    callout(cv, ["gear sectors on the lugs (printed),", "pitch radius = pin offset", "%.0f mm, mesh on the centreline" % P[1]], V((P[0], 0.0)), xl, rows(6), sz, "text", "end")
    callout(cv, ["pin boss plates on the body", "strip (printed, faint orange)"], V((P[0] + 18, -18.0)), xl, rows(24), sz, "text", "end")
    callout(cv, ["hook: string loop at the end of", "the centre rail; the carrier pin", "passes here and through an", "eyelet in each folded wing"], V(hook), xl, rows(46), sz, "hold", "end")
    callout(cv, ["wing lug on the wing plate", "(printed)"], V((P[0] + 4, P[1] + 8)), xr, rows(-10), sz, "text")
    callout(cv, ["open-stop peg (printed)"], V((P[0] - 5.5, P[1] + 8.0)), xr, rows(4), sz, "text")
    callout(cv, ["band anchor on the lug"], V(A0), xr, rows(14), sz, "shelf")
    callout(cv, ["band anchor on the body;", "rubber band (off the shelf):", "solid = open, dashed = folded"], V(B), xr, rows(26), sz, "shelf")
    callout(cv, ["wing plate: the v4 foam", "sheet, uncut except for", "a 1 mm slit at the body edge"], V((P[0] + 34, P[1] + 12)), xr, rows(44), sz, "text")
    cv.text((pad, H_ - 1.6 * size), "orange = printed   blue = off the shelf (pin, band)   green = hook / hold point", size, "muted")
    return cv, (0, 0, W_, H_)


def mech_c2(theme, s, size, N, G, R):
    """Zoom on the root and wrist of the right half (the outer hand plate continues to the tip); labels in a column on the right."""
    cv = Canvas(theme)
    P = R["c1"]["P"]
    Pw = R["c2"]["Pw"]
    y0, y1, x0, x1 = -14.0, 72.0, -14.0, 74.0
    pad = 3 * size
    top = 4.5 * size
    V = View(-y0 * s + pad, top + pad, s, x0)
    wcol = 22 * size
    W_, H_ = (y1 - y0) * s + 2 * pad + wcol, max((x1 - x0) * s + top + 2 * pad + 4 * size, top + 40 * size)
    cv.text((pad, size * 1.8), "Concept 2: reversed string, two pieces per side: root and wrist of the right half, open pose, nose up", size * 1.1, "text", bold=True)
    cut = lambda pl: [(x, min(y, y1)) for x, y in pl]
    cv.poly(V.poly([(min(max(p[0], x0), x1), p[1]) for p in G["fixed"] if p[1] >= -1]), "body", "body_s", 0.4, op=0.8)
    cv.poly(V.poly(G["armR"]), "arm", "body_s", 0.5, op=0.6)
    hand_c = [(x, y) for x, y in G["handR"] if y <= y1 + 0.5]
    ys_ = [G["yw"], y1]
    hand_view = [(G["le"](G["yw"]), G["yw"]), (G["le"](y1), y1), (G["te"](y1), y1), (G["te"](G["yw"]), G["yw"])]
    cv.poly(V.poly(hand_view), "hand", "body_s", 0.5, op=0.6)
    cv.line(V((x0, 0)), V((x1, 0)), "grid", 0.4, "1 2")
    lap = [(G["le"](G["yw"] - LAP), G["yw"] - LAP), (G["le"](G["yw"]), G["yw"]), (G["te"](G["yw"]), G["yw"]), (G["te"](G["yw"] - LAP), G["yw"] - LAP)]
    cv.poly(V.poly(lap), None, "hold", 0.7, dash="2 1.5")
    phi_a = R["c2"]["phi_sh"]
    cv.poly(V.poly(sector_pts(P, P[1], -8.0, phi_a + 8.0)), "printed", "body_s", 0.4, op=0.9)
    cv.poly(V.poly([(P[0] + dx, P[1] + dy) for dx, dy in rect_pts(-4.0, 0.0, 10.0, 16.0)]), "printed", "body_s", 0.4, op=0.9)
    cv.circle(V(P), 3.0 * s, "printed", "body_s", 0.5)
    cv.circle(V(P), 0.5 * PIN_D * s + 0.3, "shelf", "shelf", 0.4)
    cv.circle(V((P[0] - 5.5, P[1] + 8.0)), 1.5 * s, "printed", "body_s", 0.5)
    cv.circle(V(Pw), 7.5 * s, "printed", "body_s", 0.5, op=0.75)
    cv.circle(V(Pw), 0.5 * PIN_D * s + 0.3, "shelf", "shelf", 0.4)
    cv.circle(V((Pw[0] - 6.0, Pw[1] + 7.0)), 1.5 * s, "printed", "body_s", 0.5)
    wb = N["c2"]["wrist_band"]
    if wb:
        cv.line(V(wb["arm"]), V(wb["hand"]), "shelf", 1.4)
        for q in (wb["arm"], wb["hand"]):
            cv.circle(V(q), 1.0 * s, "shelf", "shelf")
    b = N["c1"]["band_raw"]
    A0 = (P[0] + b["local"][0], P[1] + b["local"][1])
    Bp = (float(b["body"][0]), float(b["body"][1]))
    cv.line(V(A0), V(Bp), "shelf", 1.4)
    for q in (A0, Bp):
        cv.circle(V(q), 1.0 * s, "shelf", "shelf")
    sr = N["c2"]["string"]
    pts = sr["lens"][0.0][1]
    # string: ring -> shoulder guide -> wrist guide -> leaves toward the tip (arrow at the edge of the view)
    gw, gs = pts[1], pts[2]
    dirx, diry = pts[0][0] - gw[0], pts[0][1] - gw[1]
    nrm = math.hypot(dirx, diry)
    edge = (gw[0] + dirx / nrm * (y1 - gw[1]) / (diry / nrm), y1)
    cv.poly(V.poly([sr["ring"], gs, gw, edge]), None, "over", 1.2, close=False)
    for q in (gs, gw):
        cv.circle(V(q), 1.0 * s, "over", "over")
    cv.circle(V(sr["ring"]), 1.6 * s, None, "over", 1.0)
    cv.text(V((edge[0] + 2, edge[1] - 2)), "to the tip", size * 0.9, "over", "end")
    hook = (N["hook"], 0.0)
    cv.circle(V(hook), 1.2 * s, "hold", "hold")
    sz = size * 0.95
    xl = V((0, y1))[0] + 2 * size
    items = [
        (P[0] + 2, ["shoulder: steel pin, printed lug,", "gear sector, boss plate, stop peg"], V(P), "text"),
        ((A0[0] + Bp[0]) / 2, ["shoulder band (rubber, off the shelf)"], V(((A0[0] + Bp[0]) / 2, (A0[1] + Bp[1]) / 2)), "shelf"),
        (Pw[0], ["wrist: printed pad dia 15 on each", "plate, steel pin, stop peg"], V(Pw), "text"),
        (Pw[0] + 2, ["wrist band (rubber, off the shelf)"], V(((wb["arm"][0] + wb["hand"][0]) / 2, (wb["arm"][1] + wb["hand"][1]) / 2)) if wb else V(Pw), "shelf"),
        (Pw[0] + 4, ["lap %.0f mm: hand plate over arm plate" % LAP], V((G["te"](G["yw"] - LAP / 2) - 2, G["yw"] - LAP / 2)), "hold"),
        (gw[0] + 2, ["fold string, 0.1 mm braid, tied at", "the tip, through printed guides"], V(gw), "over"),
        (hook[0], ["hook loop + string ring (printed) share", "the carrier pin; pulling it frees both"], V(hook), "hold"),
    ]
    items.sort(key=lambda it: it[0])
    y_du = top + pad
    for _, lines, tgt, col in items:
        callout(cv, lines, tgt, xl, y_du + size, sz, col)
        y_du += (len(lines) + 0.9) * sz * 1.15 + sz * 0.6
    cv.text((pad, H_ - 1.6 * size), "orange = printed   blue = off the shelf   magenta = fold string   green = hook   (band anchor positions: schematic)", size, "muted")
    return cv, (0, 0, W_, H_)


def section_sketch(theme, size, concept, s=None):
    """Side view across the body at the pivots: the plate stagger needed to fold wings over the body."""
    cv = Canvas(theme)
    t = T_FOAM
    s = s or 3.0 * size / 2.7
    W_, H_ = 76 * s, 22 * s
    cx, y0 = W_ / 2, H_ / 2 + 2 * s
    cv.text((4, size * 1.5), "Side view at the pivots (plates %g mm, 5 mm foam): the stagger folding over the body needs" % t, size, "text", bold=True)
    cv.poly([(cx - 18 * s, y0 - t * s / 2), (cx + 18 * s, y0 - t * s / 2), (cx + 18 * s, y0 + t * s / 2), (cx - 18 * s, y0 + t * s / 2)], "body", "body_s", 0.5)
    cv.text((cx - 17 * s, y0 + 0.5 * s), "body strip and tail plate", size * 0.85, "text")
    cv.poly([(cx + 18 * s, y0 - 1.5 * t * s), (cx + 36 * s, y0 - 1.5 * t * s), (cx + 36 * s, y0 - 0.5 * t * s), (cx + 18 * s, y0 - 0.5 * t * s)], "wingR", "body_s", 0.5, op=0.85)
    cv.poly([(cx - 36 * s, y0 + 0.5 * t * s), (cx - 18 * s, y0 + 0.5 * t * s), (cx - 18 * s, y0 + 1.5 * t * s), (cx - 36 * s, y0 + 1.5 * t * s)], "wingL", "body_s", 0.5, op=0.85)
    cv.text((cx + 19 * s, y0 - 2.1 * t * s), "right wing, one plate above", size * 0.85, "text")
    cv.text((cx - 36 * s, y0 + 2.9 * t * s), "left wing, one plate below", size * 0.85, "text")
    cv.line((cx + 18 * s, y0 - 2.4 * t * s), (cx + 18 * s, y0 + 2.4 * t * s), "shelf", 0.9)
    cv.text((cx + 1.0 * s, y0 + 2.4 * t * s + size * 1.2), "pin (off the shelf)", size * 0.85, "shelf")
    return cv, (0, 0, W_, H_)


# =============================================================================================== state drawings
def state_title(concept, t):
    ang = lambda R_, c, tt: R_["pose"][c][tt]["angles"]
    return {0.0: "open (glide pose)", 0.5: "part-folded (half way)", 1.0: "fully folded (stowed)"}[t]


def draw_one(theme, G, R, concept, t, s, size, N, margin=8.0, with_dims=True, include_path=True):
    """One state on its own canvas.  Returns (canvas, viewBox, view)."""
    cv = Canvas(theme)
    env = state_extent(G, R, concept, t)
    pth = path_of_tip(G, R, concept, tmax=t)
    if include_path:
        ymin = min(env[2], min(p[1] for p in pth), -max(p[1] for p in pth))
    else:
        ymin = min(env[2], -env[3]) - 6.0
    ymax = -ymin
    top = 9 * size
    V = View(-ymin * s + margin, top - env[0] * s + margin * 0.0, s, 0.0)
    W_ = (ymax - ymin) * s + 2 * margin
    H_ = (env[1] - env[0]) * s + top + margin + 7 * size
    draw_state(cv, V, G, R, concept, t, size=size, show_path=True, clip=None if include_path else (env[0] - 2, env[1] + 2, ymin, ymax))
    cv.text((margin, 3.6 * size), "%s: %s" % ("Concept 1" if concept == "c1" else "Concept 2", state_title(concept, t)), size * 1.15, "text", bold=True)
    ang = R["pose"][concept][t]["angles"]
    cv.text((margin, 6.4 * size), "  ".join("%s %.0f deg" % (k, v) for k, v in ang.items()) if t > 0 else "joint angles 0 deg", size * 0.95, "muted")
    if with_dims:
        n = N[concept]["n"] if t == 1.0 else None
        if t == 1.0:
            cv.text((margin, top + (env[1] - env[0]) * s + 3.2 * size), "stowed span %.0f mm, length %.0f mm (nose to tips); tips %.0f mm past the tail" % (n["stowed_span"], n["stowed_length"], n["tips_past_tail"]), size * 0.95, "dim")
            ov = n["overlaps"]
            k = "wingR_wingL" if concept == "c1" else "handR_handL"
            cv.text((margin, top + (env[1] - env[0]) * s + 6.4 * size), "pink = left and right wing overlap (%.0f mm2): one wing must sit a layer above the other" % ov[k], size * 0.95, "over")
        elif t == 0.0:
            cv.text((margin, top + (env[1] - env[0]) * s + 3.2 * size), "span %.0f mm, length nose to tail end %.0f mm; dashed purple = fold lines; amber dashed = path of the tip" % (K, N["c1"]["n"]["open_length"]), size * 0.95, "dim")
            cv.text((margin, top + (env[1] - env[0]) * s + 6.4 * size), "orange dots = pivot pins; green dot (folded view) = hook", size * 0.95, "muted")
        else:
            cv.text((margin, top + (env[1] - env[0]) * s + 3.2 * size), "orange dots = pivot pins; amber dashed = path of the tip from open to folded", size * 0.95, "muted")
    return cv, (0, 0, W_, H_), (W_, H_)


# =============================================================================================== numbers tables
def pill(tag):
    col = {"MODEL": "#7fb4ff", "ASSUMED": "#e89a55", "JUDGEMENT": "#c39bff"}[tag]
    return f"<span class='pill' style='color:{col};border-color:{col}'>{tag}</span>"


def numbers_rows(N, G, R):
    ld = N["ld"]
    rows = []
    for c, cname in (("c1", "Concept 1"), ("c2", "Concept 2")):
        n, cgn, hang = N[c]["n"], N[c]["cg"], N[c]["hang"]
        ph = R["c1"]["phi"] if c == "c1" else None
        r = lambda label, val, tag, note="": rows.append((cname, label, val, tag, note))
        r("open span x length (nose to tail end)", "%.0f x %.1f mm" % (K, n["open_length"]), "MODEL", "v4 outline A at 400 mm")
        r("stowed span (body, tail and wings)", "%.0f mm" % n["stowed_span"], "MODEL", "set by the 92 mm tail fan; wings alone %.0f mm" % n["wings_span"])
        r("stowed length (nose to wing tips)", "%.0f mm" % n["stowed_length"], "MODEL", "tips %.0f mm past the tail end" % n["tips_past_tail"])
        if c == "c1":
            r("fold angle per joint", "shoulder %.1f deg (one joint per side)" % R["c1"]["phi"], "MODEL", "angle that makes the wing lie narrowest along the body")
        else:
            r("fold angle per joint", "shoulder %.1f deg, wrist %.1f deg" % (R["c2"]["phi_sh"], R["c2"]["phi_wr"]), "MODEL", "wrist is negative: the hand turns back the other way, so it adds nothing to the stow")
        ov = n["overlaps"]
        if c == "c1":
            r("overlap, left wing over right wing", "%.0f mm2" % ov["wingR_wingL"], "MODEL", "needs the two wings in different layers")
            r("overlap, each wing over body strip and tail", "%.0f mm2" % ov["wingR_body"], "MODEL", "wings must be offset by a plate from the body plane")
        else:
            r("overlap, hand over hand (L over R)", "%.0f mm2" % ov["handR_handL"], "MODEL", "different layers needed")
            r("overlap, hand over arm, same side", "%.0f mm2" % max(ov["armR_handR"], ov["armL_handL"]), "MODEL", "the two pieces of one wing hardly overlap when folded")
            r("overlap, each arm / hand over body and tail", "%.0f / %.0f mm2" % (ov["armR_body"], ov["handR_body"]), "MODEL", "")
        mh = N[c]["m_hw"]
        r("added mass (all parts)", "%.2f g (%.2f printed, %.2f other), %.1f%% of 9.77 g" % (mh, N[c]["m_printed"], mh - N[c]["m_printed"], 100 * mh / R["base"]["M0"]), "ASSUMED", "PLA %.2f g/cm3, steel pin dia %.1f x %.0f mm, v4 volumes method" % (PLA, PIN_D, PIN_L))
        r("part count", "%d (%d distinct)" % (N[c]["count"], len(N[c]["parts"])), "MODEL", "")
        r("CG, open: v4 build -> with hardware, not re-ballasted", "%.1f -> %.1f mm" % (cgn["cg_open_v4"], cgn["cg_open_hw"]), "ASSUMED", "hardware mass at its position; wing plates at foam %.0f g/m2" % FOAM_AREAL)
        r("CG shift, folded against open", "%+.1f mm (CG %.1f mm folded)" % (cgn["shift_fold"], cgn["cg_fold_hw"]), "MODEL", "wings are %.0f%% of the mass and their centroid moves from x %.0f to %.0f mm" % (100 * cgn["m_wings"] / cgn["M_total"], cgn["wings_cg_open"], cgn["wings_cg_fold"]))
        r("hang by the hook (x %.1f mm): CG ahead of hook open / folded" % hang["x_hook"], "%+.1f / %+.1f mm" % (hang["margin_open"], hang["margin_fold"]), "MODEL", "must stay positive for the nose-down hang" + ("" if hang["margin_fold"] > 0 else "; NOT MET when folded"))
    # loads
    r = lambda c, label, val, tag, note="": rows.append((c, label, val, tag, note))
    sh, wr = ld["shoulder"], ld["wrist"]
    r("Both", "load at CL 0.3: speed, dynamic pressure, lift per wing", "%.2f m/s, %.1f Pa, %.3f N" % (ld["V"], ld["q"], ld["L_half"]), "MODEL", "v4 AVL strip file, A, bend 5 deg; weight 9.77 g")
    r("Both", "drag per wing (induced from AVL + profile)", "%.4f N + %.4f N" % (ld["D_ind"], ld["D_prof"]), "MODEL", "profile drag = CD0 %.2f x q x area, CD0 ASSUMED (flat plate, Re about 30,000)" % CD0)
    r("Both", "hinge moment about the vertical pin, glide / %.0f g pull-out" % N_PULL, "%.2f / %.2f mN.m" % (sh["M_drag_mNm"], N_PULL * sh["M_drag_mNm"]), "MODEL", "drag x arm %.0f mm; lift is parallel to the pin, so it does not turn the joint" % sh["arm_D"])
    r("Both", "bending moment on the hinge from lift, glide / pull-out", "%.1f / %.1f mN.m" % (sh["M_lift_mNm"], N_PULL * sh["M_lift_mNm"]), "MODEL", "carried by the lug and bracket plates (JUDGEMENT), not by the pin alone")
    b = N["c1"]["band"]
    r("Concept 1", "hold-open torque needed (both wings through the gears)", "%.1f mN.m" % b["M_total"], "JUDGEMENT", "safety factor %.0f x %.0f g pull-out drag moment, 2 wings" % (SF, N_PULL))
    r("Concept 1", "band lever at the open stop / worst over the stroke", "%.1f / %.1f mm" % (b["r_eff_open"], b["r_eff_min"]), "MODEL", "from the anchor-post search (post on lug %s mm, on bracket %s mm from the pin frame)" % (tuple(b["local"]), tuple(b["body"])))
    r("Concept 1", "band length between the anchors: open / folded", "%.1f / %.1f mm" % (b["L_open"], b["L_fold"]), "MODEL", "anchors found by the post search (lever must stay positive over the whole stroke)")
    note = "band stiffness %.1f N/mm (ASSUMED); the carrier pin must resist the folded tension" % BAND_K
    if b["stretch"] > BAND_MAX_STRETCH:
        note += "; stretch is above the %.1f x limit: the band needs a longer, softer routing (extra posts)" % BAND_MAX_STRETCH
    r("Concept 1", "band tension: open / folded; stretch folded/free", "%.2f N / %.2f N; x%.2f" % (b["T_open"], b["T_fold"], b["stretch"]), "ASSUMED", note)
    c2 = N["c2"]
    r("Concept 2", "hold-open torque: shoulder / wrist", "%.1f / %.1f mN.m" % (c2["m_sh_req"], c2["m_wr_req"]), "JUDGEMENT", "same factors as above, per joint")
    r("Concept 2", "spring force at an 8 mm lever: shoulder / wrist", "%.2f / %.2f N" % (c2["F_sh"], c2["F_wr"]), "JUDGEMENT", "8 mm lever is an assumption")
    r("Concept 2", "fold string: path open / folded, pay-out", "%.0f / %.0f mm, %.0f mm" % (c2["string"]["lens"][0.0][0], c2["string"]["lens"][1.0][0], c2["string"]["travel"]), "MODEL", "the ring slides this far forward while the wings open")
    return rows


def fmt_table(rows, concept):
    t = "<table><tr><th>quantity</th><th>value</th><th>tag</th><th>basis</th></tr>"
    for c, label, val, tag, note in rows:
        if c in (concept, "Both"):
            t += f"<tr><td>{html.escape(label)}</td><td class='n'>{html.escape(val)}</td><td>{pill(tag)}</td><td>{html.escape(note)}</td></tr>"
    return t + "</table>"


def penalty_text(N, G, R):
    c1n, c2n = N["c1"]["n"], N["c2"]["n"]
    chord_b = G["te"](G["yb"]) - G["le"](G["yb"])
    chord_w = G["te"](G["yw"]) - G["le"](G["yw"])
    L_rise = (G["tail_end"] - G["x_h"]) * math.sin(math.radians(6.0))
    h1 = N["c1"]["holes"]["wing"]
    h2 = N["c2"]["holes"]
    p1 = (f"<b>Open outline.</b> The swing wing is a cut along the body edge: a 1 mm slit {chord_b:.0f} mm long on each side (the hinge gap), and nothing else; no notch is needed in the wing, because the pin sits in a printed lug that lies under the plate. "
          f"<b>Not free:</b> to fold over the body strip and the tail, each wing plate must sit one plate thickness ({T_FOAM:g} mm foam) above or below the body plane, and the two wings in different layers (right +1, left -1). In the open pose that is a {T_FOAM:g} mm step at each body edge, antisymmetric. "
          f"With the 0.6 mm printed plate of the 250 mm build the step is 0.6 mm. The hold eyelets add one 8 mm hole in each wing at x {h1[0][0]:.0f} mm, y {h1[0][1]:.0f} mm (x aft of the root leading edge, y from the centreline; {'inside the plate' if h1[1] else 'NOT inside the plate'}). "
          f"The tail bent up 6 deg rises {L_rise:.1f} mm at its end, about one plate thick, so it touches the folded wing stack: the tail has to be held flat while the wings are folded.")
    p2 = (f"<b>Open outline.</b> Two slits: the body edge ({chord_b:.0f} mm) and the wrist cut ({chord_w:.0f} mm), each side, 1 mm wide, plus a {LAP:g} mm lap where the hand plate lies over the arm plate. The lap is a step of one plate ({T_FOAM:g} mm foam; 0.6 mm printed) along the whole wrist chord on the top surface; "
          f"the hand also has to be one layer above the arm, and the wings stack as in concept 1 (plate offset from the body, left and right in different layers). No notch is needed. The wrist pin and the 15 mm pad sit in the lap, a lump on the upper surface. "
          f"The tail rises {L_rise:.1f} mm when bent 6 deg: it meets the folded stack here too. The string adds six small guides on the leading edge.")
    return p1, p2


def hold_text(N, G, R):
    x_hook = N["hook"]
    a1, a2 = N["c1"], N["c2"]
    h1 = a1["holes"]["wing"]
    hh, ha = a2["holes"]["hand"], a2["holes"]["arm"]
    s1 = (f"The v4 glider hangs nose-down by a string at the trailing-edge end of the centre rail, which is the hook at x = {x_hook:.1f} mm. Folded, the wings lie along the body and put {a1['cg']['m_wings']:.1f} g behind the hook region: the CG moves {a1['cg']['shift_fold']:+.1f} mm "
          f"to x = {a1['cg']['cg_fold_hw']:.1f} mm, still ahead of the hook by {a1['hang']['margin_fold']:.1f} mm (it must stay ahead, or the glider hangs tail-down). The carrier pin passes through the hook loop and through one 8 mm eyelet in each folded wing: "
          f"the eyelet sits at x {h1[0][0]:.0f} mm, y {h1[0][1]:.0f} mm (x aft of the root leading edge, y from the centreline) in the open wing plate ({'inside the plate, 4 mm clear of the edge' if h1[1] else 'NOT inside the plate: no hole position works'}). Pulling the pin frees the glider and the wings at the same moment; the band opens them at the lowest speed of the drop. "
          f"Band tension at the folded end is {a1['band']['T_fold']:.2f} N, which the pin carries in shear.")
    s2 = (f"Same hook ({x_hook:.1f} mm). Folded CG {a2['cg']['cg_fold_hw']:.1f} mm, ahead of the hook by {a2['hang']['margin_fold']:.1f} mm. Here the carrier pin passes through the hook loop and the ring of the fold string; the string is what holds both joints folded against the springs. "
          f"When the pin is pulled the ring is free, the springs open both joints, and the ring slides forward {a2['string']['travel']:.0f} mm; the string is then slack. Eyelets in the hand plates (x {hh[0][0]:.0f}, y {hh[0][1]:.0f} mm, {'inside the plate' if hh[1] else 'NOT inside the plate'}) are the second hold if the string alone is not enough; the arm point (x {ha[0][0]:.0f}, y {ha[0][1]:.0f} mm) is {'inside' if ha[1] else 'not inside'} the arm plate.")
    return s1, s2


# =============================================================================================== page
PAGE_CSS = """:root{color-scheme:dark;--bg:#14181d;--panel:#1c2229;--line:#333c46;--ink:#dde3ea;--muted:#9aa7b4;--link:#7fb4ff;--accent:#e89a55;--thead:#262e37}
body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.5 "Segoe UI",system-ui,sans-serif}main{max-width:1380px;margin:0 auto;padding:24px 18px 60px;display:flex;flex-direction:column;gap:16px}
h1{font-size:26px;margin:0}h2{font-size:20px;margin:0;border-bottom:1px solid var(--line);padding-bottom:5px}h3{font-size:16px;margin:8px 0 2px}p{margin:0 0 6px;max-width:100ch}
.cols{display:grid;grid-template-columns:1fr 1fr;gap:20px;align-items:start}.col{display:flex;flex-direction:column;gap:10px;min-width:0}
table{border-collapse:collapse;background:var(--panel);font-size:13px;width:100%}th,td{border:1px solid var(--line);padding:3px 7px;text-align:left;vertical-align:top}th{background:var(--thead)}td.n{white-space:normal}
.pill{display:inline-block;font-size:10.5px;font-weight:700;border:1px solid;border-radius:9px;padding:0 6px;letter-spacing:.03em}
.note{border-left:3px solid var(--accent);padding:2px 0 2px 12px;color:var(--muted);font-size:14px}.box{background:var(--panel);border:2px solid var(--accent);padding:8px 14px}.fig{display:flex;flex-direction:column;gap:8px}
ul{margin:0;padding-left:22px}code{font-size:13px}"""


def build_page(G, R, N, rows, out: Path):
    s_d, size_d = 1.35, 11.0
    P = [f"<!doctype html><html lang='en'><head><meta charset='utf-8'><title>Fold concepts v4</title><style>{PAGE_CSS}</style></head><body><main>"]
    P.append("<h1>Folding concepts on swift outline A (400 mm foam build)</h1>")
    P.append(f"<p class='note'>Idea development for choosing from; nothing was built, flown or tested. Outline: v4 A at 400 mm ({K:.0f} mm span); fold lines: body edge {G['yb']:.0f} mm, wrist {G['yw']:.1f} mm from the centreline, tail hinge at x = {G['x_h']:.1f} mm. "
             f"Tags: {pill('MODEL')} geometry and arithmetic on that outline, plus v4's AVL strip loads (inviscid); {pill('ASSUMED')} v4's material assumptions (foamboard {FOAM_AREAL:g} g/m&sup2;, PLA {PLA} g/cm&sup3;, steel {STEEL} g/cm&sup3;, band stiffness, profile drag); {pill('JUDGEMENT')} choices made here. "
             f"The research survey (<code>research/birds/folding_mechanisms.md</code>) ranked concept 1 first, concept 2 (its second) next. Printable white copy: <a href='drawings.pdf'>drawings.pdf</a> (concept pages at 1:2 with a scale bar, stowed states at 1:1).</p>")
    P.append("<div class='cols'>")
    desc = {"c1": "<h2>Concept 1: geared single-pivot swing wing</h2><p>Each wing is one rigid piece on one vertical pin at the body edge; two meshed gear sectors on the lugs make left and right move together; one rubber band opens them against stop pegs; the carrier pin holds them folded through an eyelet in each wing.</p>",
            "c2": "<h2>Concept 2: reversed string, two pieces per side</h2><p>Arm and hand plates on a shoulder pin and a wrist pin, hand lapped over the arm; sector gears tie left and right at the root; bands open both joints; a leading-edge fold string, tied at the carrier pin through a ring, holds them folded.</p>"}
    for c in ("c1", "c2"):
        P.append("<div class='col'>")
        P.append(desc[c])
        for t in (0.0, 0.5, 1.0):
            cv, vb, _ = draw_one("dark", G, R, c, t, s_d, size_d, N)
            P.append("<div class='fig'>" + cv.svg_dark(vb) + "</div>")
        mc, vbm = (mech_c1 if c == "c1" else mech_c2)("dark", 4.0 if c == "c1" else 5.0, 12.0, N, G, R)
        P.append("<h3>Mechanism sketch</h3><div class='fig'>" + mc.svg_dark(vbm) + "</div>")
        sc, vbs = section_sketch("dark", 11.0, c)
        P.append("<div class='fig'>" + sc.svg_dark(vbs) + "</div>")
        P.append("<h3>Numbers</h3>" + fmt_table(rows, "Concept 1" if c == "c1" else "Concept 2"))
        pt = penalty_text(N, G, R)
        ht = hold_text(N, G, R)
        P.append("<h3>Open-pose penalty</h3><p>" + (pt[0] if c == "c1" else pt[1]) + "</p>")
        P.append("<h3>Hanging folded under the quad</h3><p>" + (ht[0] if c == "c1" else ht[1]) + "</p>")
        P.append("</div>")
    P.append("</div>")
    c1, c2 = N["c1"], N["c2"]
    P.append("<h2>Comparison</h2><table><tr><th></th><th>Concept 1: geared swing wing</th><th>Concept 2: two pieces, reversed string</th></tr>"
             f"<tr><td>added mass {pill('ASSUMED')}</td><td>{c1['m_hw']:.2f} g</td><td>{c2['m_hw']:.2f} g</td></tr>"
             f"<tr><td>part count</td><td>{c1['count']}</td><td>{c2['count']}</td></tr>"
             f"<tr><td>stowed span x length {pill('MODEL')}</td><td>{c1['n']['stowed_span']:.0f} x {c1['n']['stowed_length']:.0f} mm</td><td>{c2['n']['stowed_span']:.0f} x {c2['n']['stowed_length']:.0f} mm</td></tr>"
             f"<tr><td>CG shift folded against open {pill('MODEL')}</td><td>{c1['cg']['shift_fold']:+.1f} mm</td><td>{c2['cg']['shift_fold']:+.1f} mm</td></tr>"
             "<tr><td>complexity</td><td>one joint per side; one band; gears; stops</td><td>two joints per side; lap; two to three bands; gears; string and six guides; ring</td></tr>"
             "<tr><td>failure mode</td><td>band breaks: drag pushes the wings back toward folded, held only by friction; carrier pin jams: stays folded; gear tooth breaks: wings out of step</td><td>string breaks: both joints open (fails open); spring breaks: the joint falls back under drag; pin jams: stays folded</td></tr>"
             "<tr><td>bench test</td><td>fold and release 20 times, photograph from above and read the sweep angle spread; tip-weight test for rocking and twist play; add weight on a thread over the table edge until the wing leaves its stop (hold-open torque, target above the figure in the numbers); film the opening and check both wings arrive together</td>"
             "<td>same on one half wing: which joint moves first, does the hand stop short; tip-weight play at the wrist (decides whether a jointed wing is acceptable); weights on the fold string against the springs; leave folded 24 h and repeat</td></tr></table>")
    P.append("<h2>What does not work geometrically</h2><ul>"
             f"<li>The folded wings are long: tips reach {c1['n']['tips_past_tail']:.0f} mm past the tail end in both concepts (stowed length {c1['n']['stowed_length']:.0f} mm against {c1['n']['open_length']:.0f} mm open), as a swift's do; the arm is only {G['yw'] - G['yb']:.0f} mm of the {G['s'] - G['yb']:.0f} mm wing, so a second joint does not shorten the stow.</li>"
             f"<li>Concept 2: the wrist angle that makes the hand lie narrowest is {R['c2']['phi_wr']:.1f} deg, i.e. it turns the opposite way to the shoulder; the wrist adds sweep-control freedom later, not stowage.</li>"
             f"<li>Both: left and right wings overlap ({c1['n']['overlaps']['wingR_wingL']:.0f} mm2 in concept 1) and lie over the tail, so layers must be offset by a plate thickness; the {T_FOAM:g} mm foam plate makes that a visible step in the open outline.</li>"
             "<li>Both: the bent-up tail (6 deg) touches the folded stack; the tail has to be held flat while folded.</li></ul>")
    P.append("</main></body></html>")
    (out / "index.html").write_text("\n".join(P), encoding="utf-8")


# =============================================================================================== printable PDF
def pdf_pages(G, R, N, out: Path):
    W, H = ft.LETTER_W, ft.LETTER_H
    pages = []

    def page(elements, title, sub):
        o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}mm" height="{H}mm" viewBox="0 0 {W} {H}">', f'<rect x="0" y="0" width="{W}" height="{H}" fill="#ffffff"/>']
        o.append(f'<text x="10" y="10" font-family="Helvetica" font-size="4.6" fill="#000000" font-weight="bold">{html.escape(title)}</text>')
        o.append(f'<text x="10" y="15" font-family="Helvetica" font-size="2.8" fill="#000000">{html.escape(sub)}</text>')
        return o + elements + ["</svg>"]
    sz = 2.7
    # pages 1 and 2: the three states of one concept at 1:2
    s = 0.5
    for c in ("c1", "c2"):
        els = []
        cvs, sizes = [], []
        pos = {}
        y_cur = 24.0
        # open, centred
        for t in (0.0, 0.5, 1.0):
            cv, vb, (w_, h_) = draw_one("white", G, R, c, t, s, sz, N, margin=2.0, include_path=False)
            cvs.append((t, cv, w_, h_))
        t0, cv0, w0, h0 = cvs[0]
        els += cv0.pdf_elements((W - w0) / 2, y_cur)
        y_cur += h0 + 4
        t1, cv1, w1, h1 = cvs[1]
        t2, cv2, w2, h2 = cvs[2]
        x1 = 8.0
        x2 = W - w2 - 8.0
        els += cv1.pdf_elements(x1, y_cur)
        els += cv2.pdf_elements(x2, y_cur)
        yb_ = y_cur + max(h1, h2) + 4
        els.append(f'<line id="scalebar" x1="10" y1="{H - 14:.3f}" x2="{10 + 50 * s:.3f}" y2="{H - 14:.3f}" stroke="#000000" stroke-width="0.5"/>')
        els.append(f'<text x="10" y="{H - 10:.2f}" font-family="Helvetica" font-size="2.6" fill="#000000">50 mm of the glider, drawn at 1:2 (scale bar 25 mm long); the 400 mm span is drawn 200 mm wide. Stowed states at 1:1 on the last page.</text>')
        pages.append(page(els, "Folding concept %d: %s" % (1 if c == "c1" else 2, "geared single-pivot swing wing" if c == "c1" else "two pieces, reversed string"), "Swift outline A, 400 mm foam build, top view, nose up. Printed at 100 % this sheet is half size."))
    # pages 3 and 4: mechanism sketches
    m1, vb1 = mech_c1("white", 1.15, 2.7, N, G, R)
    sc, vbs = section_sketch("white", 2.7, "c1", s=2.3)
    els = m1.pdf_elements(8.0, 20.0) + sc.pdf_elements(10.0, 20.0 + vb1[3] + 6)
    pages.append(page(els, "Mechanism sketch, concept 1", "Scale about 1.15:1 (not 1:1). Orange = printed, blue = off the shelf, green = hook."))
    m2, vb2 = mech_c2("white", 1.5, 2.7, N, G, R)
    pages.append(page(m2.pdf_elements(8.0, 20.0), "Mechanism sketch, concept 2", "Scale about 1.5:1 (not 1:1). Orange = printed, blue = off the shelf, magenta = fold string, green = hook."))
    # page 4: stowed states at 1:1
    els = []
    for c, cx in (("c1", 52.0), ("c2", 150.0)):
        cv = Canvas("white")
        V = View(cx, 26.0 + 36.0, 1.0, 0.0)
        draw_state(cv, V, G, R, c, 1.0, size=2.7, show_path=False)
        cv.text((cx - 40, 21.0), ("Concept 1 folded" if c == "c1" else "Concept 2 folded") + ", 1:1", 3.4, "text", bold=True)
        els += cv.pdf_elements()
    els.append(f'<line id="scalebar" x1="8" y1="40" x2="8" y2="140" stroke="#000000" stroke-width="0.5"/>')
    els.append('<text x="10" y="90" font-family="Helvetica" font-size="2.6" fill="#000000">100 mm bar (vertical): check after printing at 100 %</text>')
    pages.append(page(els, "Stowed states at 1:1", "Print at 100 %. Green dot = hook; orange dots = pins; pink = left / right wing overlap."))
    paths = []
    for i, pg in enumerate(pages):
        p = out / f"drawings_page{i + 1}.svg"
        p.write_text("\n".join(pg), encoding="utf-8")
        paths.append(p)
    pdf = out / "drawings.pdf"
    subprocess.run([sys.executable, "-B", str(ROOT / "rigid_glider/flat_templates_pdf.py"), str(pdf)] + [str(p) for p in paths], check=True, stdout=subprocess.DEVNULL)
    return pdf


# =============================================================================================== main
def write_numbers(rows, out: Path):
    with (out / "numbers.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["concept", "quantity", "value", "tag", "basis"])
        w.writerows(rows)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    G = geometry()
    R = solve_concepts(G)
    N = analyse(G, R)
    rows = numbers_rows(N, G, R)
    write_numbers(rows, OUT)
    # individual SVG drawings (dark) for reuse
    for c in ("c1", "c2"):
        for t in (0.0, 0.5, 1.0):
            cv, vb, _ = draw_one("dark", G, R, c, t, 1.35, 11.0, N)
            (OUT / f"{c}_{'open' if t == 0 else 'part' if t == 0.5 else 'folded'}.svg").write_text(cv.svg_dark(vb), encoding="utf-8")
    build_page(G, R, N, rows, OUT)
    pdf = pdf_pages(G, R, N, OUT)
    json.dump({"rows": rows}, (OUT / "numbers.json").open("w", encoding="utf-8"), indent=1)
    print("wrote", OUT, pdf.name)
    return G, R, N, rows


if __name__ == "__main__":
    main()
