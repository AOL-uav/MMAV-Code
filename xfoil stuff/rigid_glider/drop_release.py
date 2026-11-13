"""Glider drop-release for a small FPV quad: boom tip latch (pin through two eyelets), servo pad, boom saddles, receiver plate.

DESIGN OUTPUT ONLY.  Nothing here has been printed, fitted or flown.  Every number carries a source tag:
  GEOMETRY (exact from the model), ARITHMETIC (simple sum, inputs stated), JUDGEMENT (a design choice), ASSUMED (hardware not measured), UNKNOWN.
Hardware dimensions Kai has not measured (boom rod, pin wire, servo, frame arm, quad) are named parameters in PARAMS below, tagged ASSUMED.
Measure yours and re-run, e.g.
  py -B rigid_glider/drop_release.py --rod-d 5 --pin-d 0.8 --servo-l 20 --servo-w 9 --servo-h 16 --boom-length 220

Reuses (by import, no edits) the STL writer, edge checker, tag helper and always-dark CSS of flat_test.py.
Writes ONLY under results/runs/rigid_glider/drop_release/ (new folder) unless --out is given.
"""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import argparse
import csv
import html
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
import flat_test as ft                                   # noqa: E402  (imported, never edited)

OUT_DEFAULT = ft.ROOT / "results/runs/rigid_glider/drop_release"

# =============================================================================================== PARAMETERS (Kai: change these)
# (key, default, unit, tag, note, hardware).  hardware=True -> listed on the page as "measure yours and re-run".
PARAMS = [
    # ---- hardware that is UNKNOWN to this script (ASSUMED defaults)
    ("rod_d_mm", 4.0, "mm", "ASSUMED", "Boom rod OUTER diameter. Kai's rod is unknown. Measure with calipers.", True),
    ("rod_wall_mm", 0.5, "mm", "ASSUMED", "Boom rod wall thickness (mass estimate only; tube assumed hollow carbon).", True),
    ("pin_d_mm", 1.0, "mm", "ASSUMED", "Release pin wire diameter (straight, hard, smooth wire). Measure it.", True),
    ("servo_l_mm", 23.0, "mm", "ASSUMED", "Servo body length (the long side, not counting mounting ears). Measure your servo.", True),
    ("servo_w_mm", 12.0, "mm", "ASSUMED", "Servo body width (short side).", True),
    ("servo_h_mm", 22.0, "mm", "ASSUMED", "Servo body height along the output shaft, base to the face the arm sits on.", True),
    ("servo_mass_g", 9.0, "g", "ASSUMED", "Servo mass used for the mass and CG sums (the 9 g class is the worst case).", True),
    ("servo_shaft_from_end_mm", 5.5, "mm", "ASSUMED", "Output shaft centre to the near end of the body, along the length.", True),
    ("servo_arm_below_body_mm", 3.5, "mm", "ASSUMED", "Arm mid-plane to the servo body face it hangs below.", True),
    ("servo_arm_thick_mm", 1.6, "mm", "ASSUMED", "Servo arm thickness.", True),
    ("servo_arm_radius_mm", 10.0, "mm", "ASSUMED", "Distance from the shaft centre to the arm hole the pin hook goes through.", True),
    ("servo_throw_deg", 90.0, "deg", "ASSUMED", "Arm angle you will set between pin-closed and pin-open end points (servo must be able to do it).", True),
    ("frame_arm_w_mm", 12.0, "mm", "ASSUMED", "Width of the frame arm / plate the saddle blocks clamp to. Vapor5 frame is UNKNOWN here.", True),
    ("rx_l_mm", 22.0, "mm", "ASSUMED", "PWM receiver length (for the optional receiver plate).", True),
    ("rx_w_mm", 12.0, "mm", "ASSUMED", "PWM receiver width.", True),
    ("rx_mass_g", 2.0, "g", "ASSUMED", "PWM receiver mass.", True),
    ("zip_tie_w_mm", 2.5, "mm", "ASSUMED", "Zip tie width. Slots are cut for this; measure your ties.", True),
    ("zip_tie_t_mm", 1.0, "mm", "ASSUMED", "Zip tie thickness.", True),
    ("quad_mass_g", 650.0, "g", "ASSUMED", "Quad all-up mass with battery, for the CG-shift sums. Weigh it.", True),
    ("motor_diag_mm", 225.0, "mm", "ASSUMED", "Motor-to-motor DIAGONAL distance (true-X). Measure the Vapor5; frame geometry is UNKNOWN here.", True),
    ("prop_d_in", 5.0, "in", "ASSUMED", "Propeller diameter.", True),
    ("boom_rear_x_mm", -30.0, "mm", "ASSUMED", "Where the rod starts, measured forward of the quad centre (negative = behind the centre).", True),
    ("boom_length_mm", 200.0, "mm", "JUDGEMENT", "Rod END measured forward of the quad centre. Default puts the glider hang point past the front prop edge by the margin below.", True),
    # ---- design constants (JUDGEMENT = chosen here, change if prints misfit)
    ("bore_clear_mm", 0.3, "mm", "JUDGEMENT", "Rod socket diameter = rod diameter + this (printed holes run small). Tune after a test print.", False),
    ("socket_wall_mm", 2.4, "mm", "JUDGEMENT", "Socket tube wall (6 perimeters of 0.4 mm).", False),
    ("slit_mm", 1.0, "mm", "JUDGEMENT", "Clamp slit width along the top of the socket.", False),
    ("socket_len_mm", 22.0, "mm", "JUDGEMENT", "Socket length (rod insertion depth).", False),
    ("tie_groove_depth_mm", 1.0, "mm", "JUDGEMENT", "Depth of the zip-tie groove round socket and rail.", False),
    ("pin_clear_mm", 0.2, "mm", "JUDGEMENT", "Pin hole diameter = pin wire diameter + this.", False),
    ("post_t_mm", 4.0, "mm", "JUDGEMENT", "Thickness of each eyelet post along the pin.", False),
    ("gap_mm", 5.0, "mm", "JUDGEMENT", "Gap between the two eyelet posts (the string loop sits here).", False),
    ("engage_mm", 3.0, "mm", "JUDGEMENT", "Pin tip depth inside the far post when closed.", False),
    ("retract_inside_mm", 1.5, "mm", "JUDGEMENT", "When open, pin tip is pulled back this far INTO the near post (still guided).", False),
    ("bridge_clear_mm", 1.5, "mm", "JUDGEMENT", "Space between the pin and the underside of the bridge over the gap.", False),
    ("bridge_t_mm", 2.0, "mm", "JUDGEMENT", "Bridge thickness.", False),
    ("hook_clear_mm", 3.0, "mm", "JUDGEMENT", "Clearance between the rear end of the pin rail and the arm hole at its most forward position.", False),
    ("servo_rod_gap_mm", 3.0, "mm", "JUDGEMENT", "Free space between the rod and the servo side face (room for the zip tie).", False),
    ("pad_t_mm", 2.0, "mm", "JUDGEMENT", "Servo pad plate thickness.", False),
    ("post_len_mm", 7.0, "mm", "JUDGEMENT", "Length (along the boom) of the vertical post carrying the servo pad.", False),
    ("post_w_mm", 4.4, "mm", "JUDGEMENT", "Width of the vertical post.", False),
    ("saddle_len_mm", 16.0, "mm", "JUDGEMENT", "Length of each frame saddle block along the rod.", False),
    ("saddle_rib_h_mm", 0.4, "mm", "JUDGEMENT", "Height of the anti-slide ribs in the saddle groove.", False),
    ("saddle_spacing_mm", 60.0, "mm", "JUDGEMENT", "Spacing between the two saddle blocks along the rod.", False),
    ("prop_margin_mm", 60.0, "mm", "JUDGEMENT", "Hang point must be at least this far forward of the front prop disc edge. Downwash column size is UNKNOWN and not modelled.", False),
    ("loop_len_mm", 25.0, "mm", "JUDGEMENT", "String loop length (flat, knot to end) from glider nose attachment.", False),
    # ---- friction / servo-force arithmetic inputs
    ("mu_lo", 0.2, "", "ASSUMED", "Low combined friction coefficient (string on wire plus wire in printed hole).", False),
    ("mu_hi", 0.6, "", "ASSUMED", "High combined friction coefficient.", False),
    ("load_factor", 2.0, "", "JUDGEMENT", "Extra factor on the hanging weight for binding / gentle manoeuvre load.", False),
    ("usable_torque_frac", 0.3, "", "JUDGEMENT", "Fraction of stall torque treated as usable continuous pull.", False),
    ("pla_density_g_cm3", ft.PLA_DENSITY_G_CM3, "g/cm3", "ASSUMED", "PLA density, 100 % solid. Weigh your prints.", False),
    ("pin_density_g_cm3", 7.85, "g/cm3", "ASSUMED", "Pin wire density (steel).", False),
    ("rod_density_g_cm3", 1.6, "g/cm3", "ASSUMED", "Rod material density (carbon tube).", False),
    ("misc_mass_g", 1.5, "g", "ASSUMED", "Zip ties and tape on the whole assembly, placed at the quad centre.", False),
]
PARAM_KEYS = [p[0] for p in PARAMS]
# ASSUMED servo stall torques in kg.cm for the three servo classes Kai has.  NOT a product spec.  Replace with your servo's measured value.
SERVO_CLASSES = [("2.1 g class", 2.1, 0.35), ("5 g class", 5.0, 0.9), ("9 g class", 9.0, 1.6)]
GLIDER_MASSES_G = (10.0, 20.0, 30.0)
G = 9.80665

CLI_FLAGS = {"--rod-d": "rod_d_mm", "--pin-d": "pin_d_mm", "--servo-l": "servo_l_mm", "--servo-w": "servo_w_mm",
             "--servo-h": "servo_h_mm", "--boom-length": "boom_length_mm"}

BG, PANEL, INK, MUTED, ACCENT, BLUE, GREEN, RED = "#14181d", "#27313c", "#dde3ea", "#9aa7b4", "#e89a55", "#7fb4ff", "#6cc08b", "#ff8a80"


def default_params():
    return {k: v for k, v, *_ in PARAMS}


def tag_of(key):
    return next(p[3] for p in PARAMS if p[0] == key)


# =============================================================================================== 2D polygon tools
def arc(cx, cy, r, a0, a1, n):
    return [(cx + r * math.cos(a0 + (a1 - a0) * i / n), cy + r * math.sin(a0 + (a1 - a0) * i / n)) for i in range(n + 1)]


def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _seg_touch(p1, p2, p3, p4, eps=1e-9):
    d1, d2, d3, d4 = _cross(p3, p4, p1), _cross(p3, p4, p2), _cross(p1, p2, p3), _cross(p1, p2, p4)
    if ((d1 > eps and d2 < -eps) or (d1 < -eps and d2 > eps)) and ((d3 > eps and d4 < -eps) or (d3 < -eps and d4 > eps)):
        return True

    def on(p, q, r):
        return abs(_cross(p, q, r)) <= eps and min(p[0], q[0]) - eps <= r[0] <= max(p[0], q[0]) + eps and min(p[1], q[1]) - eps <= r[1] <= max(p[1], q[1]) + eps
    return on(p3, p4, p1) or on(p3, p4, p2) or on(p1, p2, p3) or on(p1, p2, p4)


def _pip(poly, p):
    x, y = p
    c = False
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        if (a[1] > y) != (b[1] > y) and x < a[0] + (y - a[1]) * (b[0] - a[0]) / (b[1] - a[1]):
            c = not c
    return c


def _bridge_holes(outer, holes):
    """Weakly-simple polygon: holes joined to the outline by zero-width bridges (used for the CAPS only; walls use the real loops)."""
    poly = list(outer)
    edges_all = []
    for lp in [outer] + list(holes):
        edges_all += [(lp[i], lp[(i + 1) % len(lp)]) for i in range(len(lp))]
    for h in sorted(holes, key=lambda h: -max(p[0] for p in h)):
        mi = max(range(len(h)), key=lambda i: h[i][0])
        M = h[mi]
        cands = sorted(range(len(poly)), key=lambda i: (poly[i][0] - M[0]) ** 2 + (poly[i][1] - M[1]) ** 2)
        for ci in cands:
            V = poly[ci]
            mid = ((M[0] + V[0]) / 2, (M[1] + V[1]) / 2)
            if not _pip(outer, mid) or any(_pip(hh, mid) for hh in holes):
                continue
            if any(not (a == M or b == M or a == V or b == V) and _seg_touch(M, V, a, b) for a, b in edges_all):
                continue
            break
        else:
            raise ValueError("no bridge found for a hole")
        edges_all.append((M, V))
        poly = poly[:ci + 1] + h[mi:] + h[:mi] + [h[mi], poly[ci]] + poly[ci + 1:]
    return poly


def _ear_clip(poly):
    idx = list(range(len(poly)))
    tris = []
    while len(idx) > 3:
        n = len(idx)
        found = False
        for strict in (True, False):
            for k in range(n):
                i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % n]
                a, b, c = poly[i0], poly[i1], poly[i2]
                cr = _cross(a, b, c)
                if (strict and cr <= 1e-9) or (not strict and cr < -1e-9):
                    continue
                ok = True
                if cr > 1e-9:
                    for j in idx:
                        p = poly[j]
                        if j in (i0, i1, i2) or p == a or p == b or p == c:
                            continue
                        if _cross(a, b, p) >= -1e-9 and _cross(b, c, p) >= -1e-9 and _cross(c, a, p) >= -1e-9:
                            ok = False
                            break
                if ok:
                    tris.append((a, b, c))
                    del idx[k]
                    found = True
                    break
            if found:
                break
        if not found:
            raise ValueError("ear clipping failed")
    tris.append(tuple(poly[i] for i in idx))
    return tris


_TRI_CACHE = {}


def triangulate_holes(outer, holes):
    key = (tuple(outer), tuple(tuple(h) for h in holes))
    if key not in _TRI_CACHE:
        _TRI_CACHE[key] = _ear_clip(_bridge_holes(outer, holes) if holes else list(outer))
    return _TRI_CACHE[key]


class Shell:
    """A prism: 'x' = profile (y, z) extruded along x from a0 to a1;  'z' = profile (x, y) extruded along z.  Profile = outer loop + hole loops."""

    def __init__(self, name, kind, outer, holes, a0, a1):
        self.name, self.kind, self.a0, self.a1 = name, kind, a0, a1
        self.outer = list(outer) if ft.area_signed(outer) > 0 else list(reversed(outer))
        self.holes = [list(h) if ft.area_signed(h) < 0 else list(reversed(h)) for h in holes]
        if len(self.holes) > 2:        # break exact collinearity between many aligned slot corners (ear clipping); 0.3 micron, far below print resolution
            self.holes = [[(p[0] + 3e-4 * ((7 * i + 3 * k) % 5), p[1] + 3e-4 * ((11 * i + 5 * k) % 7)) for k, p in enumerate(h)] for i, h in enumerate(self.holes)]
        self._raster = None

    def _map(self, u, v, w):
        return (w, u, v) if self.kind == "x" else (u, v, w)

    def tris(self):
        out = []
        for a, b, c in triangulate_holes(self.outer, self.holes):
            out.append((self._map(*a, self.a1), self._map(*b, self.a1), self._map(*c, self.a1)))
            out.append((self._map(*c, self.a0), self._map(*b, self.a0), self._map(*a, self.a0)))
        for lp in [self.outer] + self.holes:
            n = len(lp)
            for i in range(n):
                a, b = lp[i], lp[(i + 1) % n]
                out.append((self._map(*a, self.a0), self._map(*b, self.a0), self._map(*b, self.a1)))
                out.append((self._map(*a, self.a0), self._map(*b, self.a1), self._map(*a, self.a1)))
        return out

    def raster(self, res=0.1):
        if self._raster is None:
            us = [p[0] for p in self.outer]
            vs = [p[1] for p in self.outer]
            u0, v0 = min(us), min(vs)
            nu, nv = int(math.ceil((max(us) - u0) / res)) + 1, int(math.ceil((max(vs) - v0) / res)) + 1
            rows = []
            loops = [self.outer] + self.holes
            for j in range(nv):
                v = v0 + (j + 0.5) * res
                xs = []
                for lp in loops:
                    for i in range(len(lp)):
                        a, b = lp[i], lp[(i + 1) % len(lp)]
                        if (a[1] <= v) != (b[1] <= v):
                            xs.append(a[0] + (v - a[1]) * (b[0] - a[0]) / (b[1] - a[1]))
                xs.sort()
                row = bytearray(nu)
                for k in range(0, len(xs) - 1, 2):
                    i0 = max(0, int(math.ceil((xs[k] - u0) / res - 0.5)))
                    i1 = min(nu - 1, int(math.floor((xs[k + 1] - u0) / res - 0.5)))
                    if i1 >= i0:
                        row[i0:i1 + 1] = b"\x01" * (i1 - i0 + 1)
                rows.append(row)
            self._raster = (u0, v0, nu, nv, rows, res)
        return self._raster

    def inside(self, u, v):
        u0, v0, nu, nv, rows, res = self.raster()
        i, j = int((u - u0) / res), int((v - v0) / res)
        return 0 <= i < nu and 0 <= j < nv and rows[j][i] == 1

    def bbox3(self):
        us = [p[0] for p in self.outer]
        vs = [p[1] for p in self.outer]
        if self.kind == "x":
            return (self.a0, min(us), min(vs)), (self.a1, max(us), max(vs))
        return (min(us), min(vs), self.a0), (max(us), max(vs), self.a1)


class Part:
    def __init__(self, name, shells):
        self.name, self.shells = name, shells

    def tris(self):
        return [t for s in self.shells for t in s.tris()]

    def union_stats(self, res=0.25):
        """Union volume (mm3) and centroid by voxel sampling (res mm).  Shells overlap on purpose (so the STL has no shared faces); this removes the double count.  +-2 %."""
        vox = set()
        for s in self.shells:
            lo, hi = s.bbox3()
            i0, i1 = [int(math.ceil(lo[a] / res - 0.5)) for a in (0, 1, 2)], [int(math.floor(hi[a] / res - 0.5)) for a in (0, 1, 2)]
            for i in range(i0[0], i1[0] + 1):
                x = (i + 0.5) * res
                for j in range(i0[1], i1[1] + 1):
                    y = (j + 0.5) * res
                    for k in range(i0[2], i1[2] + 1):
                        z = (k + 0.5) * res
                        if (s.inside(y, z) if s.kind == "x" else s.inside(x, y)) and (s.a0 <= (x if s.kind == "x" else z) <= s.a1):
                            vox.add((i, j, k))
        n = len(vox)
        cen = tuple((sum(v[a] for v in vox) + 0.5 * n) * res / n for a in (0, 1, 2)) if n else (0, 0, 0)
        return n * res ** 3, cen


def rect_loop(x0, x1, y0, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def slot_loop(cx, cy, lx, ly):
    return [(cx - lx / 2, cy - ly / 2), (cx - lx / 2, cy + ly / 2), (cx + lx / 2, cy + ly / 2), (cx + lx / 2, cy - ly / 2)]


# =============================================================================================== derived dimensions
def derive(P):
    D = {}
    D["Rb"] = (P["rod_d_mm"] + P["bore_clear_mm"]) / 2
    D["hole_d"] = P["pin_d_mm"] + P["pin_clear_mm"]
    D["kw"] = max(5.0, D["hole_d"] + 4.4)
    r, th = P["servo_arm_radius_mm"], math.radians(P["servo_throw_deg"])
    D["half_sweep"] = r * math.sin(th / 2)
    D["arm_throw"] = 2 * D["half_sweep"]
    D["yp"] = max(0.0, P["rod_d_mm"] / 2 + P["servo_rod_gap_mm"] + P["servo_w_mm"] / 2 - r)      # pin y offset so the servo clears the rod
    D["ys"] = D["yp"] + r                                                                      # servo shaft y
    D["Ro"] = max(D["Rb"] + P["socket_wall_mm"], D["yp"] + D["kw"] / 2 + 0.7)                    # socket outer radius (grows only if the rail would not reach the wall)
    D["zpin"] = -(D["Ro"] + D["kw"] / 2 - 0.6)
    D["keel_zb"] = D["zpin"] - D["kw"] / 2
    D["Ls"] = P["socket_len_mm"]
    D["te"], D["gap"] = P["post_t_mm"], P["gap_mm"]
    D["Lk1"] = D["Ls"] + D["te"]
    D["p2a"] = D["Lk1"] + D["gap"]
    D["p2b"] = D["p2a"] + D["te"]
    tw = P["zip_tie_w_mm"] + 0.8
    D["nx0"], D["nx1"] = D["Ls"] / 2 - tw / 2, D["Ls"] / 2 + tw / 2
    D["travel"] = D["gap"] + P["engage_mm"] + P["retract_inside_mm"]
    D["arm_angle_req"] = math.degrees(2 * math.asin(min(1.0, D["travel"] / (2 * r))))
    D["xs"] = -(P["hook_clear_mm"] + D["half_sweep"])                                          # shaft x
    D["x_hook_closed"] = D["xs"] + D["half_sweep"]
    D["x_hook_open"] = D["x_hook_closed"] - D["travel"]
    D["x_tip_closed"] = D["p2a"] + P["engage_mm"]
    D["x_tip_open"] = D["x_tip_closed"] - D["travel"]
    D["servo_xfront"] = D["xs"] + P["servo_shaft_from_end_mm"]
    D["servo_xrear"] = D["servo_xfront"] - P["servo_l_mm"]
    D["z_arm"] = D["zpin"] + 2.0
    D["servo_zbot"] = D["z_arm"] + P["servo_arm_below_body_mm"]
    D["servo_ztop"] = D["servo_zbot"] + P["servo_h_mm"]
    D["pad_z0"] = D["servo_ztop"]
    D["leg_len"] = round(2.0 + P["servo_arm_thick_mm"] / 2 + 1.0, 1)
    D["pin_cut_len"] = (D["x_tip_closed"] - D["x_hook_closed"]) + D["leg_len"]
    D["hang_beyond_rod_end"] = D["Lk1"] + D["gap"] / 2 - D["Ls"]
    D["post_x0"] = 1.0
    D["post_x1"] = 1.0 + P["post_len_mm"]
    D["post_y0"] = -D["Ro"] - 3.0
    D["post_y1"] = D["post_y0"] + P["post_w_mm"]
    D["rod_len"] = P["boom_length_mm"] - P["boom_rear_x_mm"]
    assert D["Ro"] - 0.3 > D["yp"] + D["kw"] / 2 + 0.3, "pin offset too large for this rod and servo: use a longer servo arm (servo_arm_radius_mm) or a narrower servo"
    return D


# =============================================================================================== profiles
def tube_profile(Ro, Rb, slit, n_out=48, n_bore=32):
    sw = slit / 2
    zo, zb = math.sqrt(Ro ** 2 - sw ** 2), math.sqrt(Rb ** 2 - sw ** 2)
    aL, aR = math.atan2(zo, -sw), math.atan2(zo, sw) + 2 * math.pi
    outer = arc(0, 0, Ro, aL, aR, n_out)
    bore = arc(0, 0, Rb, math.atan2(zb, sw), math.atan2(zb, -sw) - 2 * math.pi, n_bore)
    return outer + bore, []


def keel_profile(D, inset=0.0, top_r=None):
    yp, zp = D["yp"], D["zpin"]
    hw = D["kw"] / 2 - inset
    zb = D["keel_zb"] + inset
    tr = D["Ro"] - 0.3 if top_r is None else top_r
    rc = min(1.5, hw - 0.6)
    yr, yl = yp + hw, yp - hw
    assert tr > max(abs(yr), abs(yl)) + 0.2, "keel does not reach the socket wall"
    pts = arc(yl + rc, zb + rc, rc, math.pi, 1.5 * math.pi, 6) + arc(yr - rc, zb + rc, rc, 1.5 * math.pi, 2 * math.pi, 6)
    for i in range(11):
        y = yr + (yl - yr) * i / 10
        pts.append((y, -math.sqrt(tr * tr - y * y)))
    hole = arc(yp, zp, D["hole_d"] / 2, 2 * math.pi, 0, 16)[:-1]
    return pts, [hole]


def bridge_profile(P, D):
    hw = D["kw"] / 2 - 0.2
    z0 = D["zpin"] + D["hole_d"] / 2 + P["bridge_clear_mm"]
    z1 = z0 + P["bridge_t_mm"]
    assert z1 <= -D["Rb"] - 0.3, "bridge would break into the rod socket"
    return rect_loop(D["yp"] - hw, D["yp"] + hw, z0, z1), []


# =============================================================================================== parts
def build_latch(P, D):
    ov = 0.1
    sh = []

    def X(name, prof, x0, x1):
        sh.append(Shell(name, "x", prof[0], prof[1], x0, x1))
    gd = P["tie_groove_depth_mm"]
    tf = tube_profile(D["Ro"], D["Rb"], P["slit_mm"])
    tn = tube_profile(D["Ro"] - gd, D["Rb"], P["slit_mm"])
    kf = keel_profile(D)
    kn = keel_profile(D, inset=gd, top_r=D["Ro"] - gd - 0.3)
    X("tube_rear", tf, 0, D["nx0"])
    X("tube_neck", tn, D["nx0"] - ov, D["nx1"] + ov)
    X("tube_front", tf, D["nx1"], D["Ls"])
    X("rail_rear", kf, 0, D["nx0"])
    X("rail_neck", kn, D["nx0"] - ov, D["nx1"] + ov)
    X("rail_front", kf, D["nx1"], D["Lk1"] - 0.7)
    X("post1_chamfer1", keel_profile(D, 0.35), D["Lk1"] - 0.8, D["Lk1"] - 0.35)
    X("post1_chamfer2", keel_profile(D, 0.7), D["Lk1"] - 0.45, D["Lk1"])
    X("bridge", bridge_profile(P, D), D["Ls"] - 3.0, D["p2b"] - 1.0)
    X("post2_chamfer2", keel_profile(D, 0.7), D["p2a"], D["p2a"] + 0.45)
    X("post2_chamfer1", keel_profile(D, 0.35), D["p2a"] + 0.35, D["p2a"] + 0.8)
    X("post2_main", kf, D["p2a"] + 0.7, D["p2b"])
    pil = [slot_loop(D["post_x0"] + P["post_len_mm"] * f, (D["post_y0"] + D["post_y1"]) / 2, 1.0, 1.0) for f in (0.25, 0.75)]
    pil = [arc(c[0][0] + 0.5, c[0][1] + 0.5, 0.85, 2 * math.pi, 0, 12)[:-1] for c in pil]               # 1.7 mm self-tapping pilot holes for M2
    sh.append(Shell("side_post", "z", rect_loop(D["post_x0"], D["post_x1"], D["post_y0"], D["post_y1"]), pil, D["keel_zb"], D["pad_z0"]))
    return Part("A_tip_latch", sh), pil


def pad_layout(P, D):
    ys, W, L = D["ys"], P["servo_w_mm"], P["servo_l_mm"]
    x0, x1 = D["servo_xrear"] - 3.0, D["servo_xfront"] + 3.0
    ya, yb = ys - W / 2 - 3.2, ys + W / 2 + 3.2
    xb0, xb1 = -8.0, D["post_x1"] + 1.5
    yc = D["post_y0"] - 0.8
    assert xb0 > x0 and xb1 > x1 and yc < ya
    outline = [(xb0, yc), (xb1, yc), (xb1, ya), (x1, ya), (x1, yb), (x0, yb), (x0, ya), (xb0, ya)]
    tie_x = [D["servo_xrear"] + 0.22 * L, D["servo_xrear"] + 0.62 * L]
    sl, sw = P["zip_tie_w_mm"] + 0.9, P["zip_tie_t_mm"] + 0.6
    slots = []
    for tx in tie_x:
        for ty in (ys - W / 2 - 1.6, ys + W / 2 + 1.6):
            slots.append(slot_loop(tx, ty, sl, sw))
    return outline, slots, tie_x, (x0, x1, ya, yb)


def build_pad(P, D):
    outline, slots, tie_x, _ = pad_layout(P, D)
    cy = (D["post_y0"] + D["post_y1"]) / 2
    screws = [arc(D["post_x0"] + P["post_len_mm"] * f, cy, 1.1, 2 * math.pi, 0, 12)[:-1] for f in (0.25, 0.75)]      # 2.2 mm clearance for M2
    z0 = D["pad_z0"]
    return Part("A2_servo_pad", [Shell("pad", "z", outline, slots + screws, z0, z0 + P["pad_t_mm"])])


def saddle_dims(P, D):
    Rg = P["rod_d_mm"] / 2 + 0.1
    Wb = 2 * Rg + 6.0
    slot_y = max(P["frame_arm_w_mm"] / 2 + 2.0, Wb / 2 + 3.0)
    Wp = 2 * (slot_y + 3.0)
    return Rg, Wb, slot_y, Wp


def build_saddle(P, D):
    Rg, Wb, slot_y, Wp = saddle_dims(P, D)
    L = P["saddle_len_mm"]
    base_t = 3.0
    zc = base_t + Rg
    sl, sw = P["zip_tie_w_mm"] + 0.9, P["zip_tie_t_mm"] + 0.6
    # base is a z-extrusion with profile (x, y): x along the rod, y across
    base = Shell("base", "z", rect_loop(0, L, -Wp / 2, Wp / 2), [slot_loop(L * f, s * slot_y, sl, sw) for f in (0.25, 0.75) for s in (-1, 1)], 0, base_t)
    rail_pts = [(-Wb / 2, base_t - 0.2), (Wb / 2, base_t - 0.2), (Wb / 2, zc), (Rg, zc)] + arc(0, zc, Rg, 0, -math.pi, 24)[1:-1] + [(-Rg, zc), (-Wb / 2, zc)]
    sh = [base, Shell("rail", "x", rail_pts, [], 0, L)]
    rh = P["saddle_rib_h_mm"]
    rib = arc(0, zc, Rg + 0.3, 0, -math.pi, 24) + arc(0, zc, Rg - rh, -math.pi, 0, 24)
    for i, xr in enumerate((0.16, 0.5, 0.84)):
        sh.append(Shell(f"rib{i}", "x", rib, [], L * xr - 0.5, L * xr + 0.5))
    return Part("B_frame_saddle", sh)


def build_rx_plate(P, D):
    L, W = P["rx_l_mm"] + 6.0, P["rx_w_mm"] + 14.0
    sl, sw = P["zip_tie_w_mm"] + 0.9, P["zip_tie_t_mm"] + 0.6
    slots = [slot_loop(L * f, s * (P["rx_w_mm"] / 2 + 1.6), sl, sw) for f in (0.3, 0.7) for s in (-1, 1)]        # receiver ties
    slots += [slot_loop(L * f, s * (W / 2 - 1.6), sl, sw) for f in (0.3, 0.7) for s in (-1, 1)]                      # ties to the frame
    return Part("C_rx_plate", [Shell("plate", "z", rect_loop(0, L, -W / 2, W / 2), slots, 0, 1.6)])


def build(P, mass=True):
    D = derive(P)
    latch, pil = build_latch(P, D)
    parts = {"A_tip_latch": latch, "A2_servo_pad": build_pad(P, D), "B_frame_saddle": build_saddle(P, D), "C_rx_plate": build_rx_plate(P, D)}
    info = {}
    for n, p in parts.items():
        tris = p.tris()
        if n == "A2_servo_pad":
            z0 = D["pad_z0"]
            tris = [tuple((a, b, c - z0) for a, b, c in t) for t in tris]
        info[n] = {"tris": tris}
        lo, hi = ft.bbox(tris)
        info[n]["bbox"] = tuple(h - l for l, h in zip(lo, hi))
        if mass:
            vol, cen = p.union_stats()
            info[n]["volume"], info[n]["centroid"] = vol, cen
            info[n]["mass_g"] = vol * 1e-3 * P["pla_density_g_cm3"]
            info[n]["shell_sum_mm3"] = sum(abs(ft.mesh_stats(s.tris())["volume"]) for s in p.shells)
    return D, parts, info


# =============================================================================================== arithmetic
def pin_travel(gap, engage, retract_inside):
    return gap + engage + retract_inside


def arm_throw(radius, throw_deg):
    return 2 * radius * math.sin(math.radians(throw_deg) / 2)


def arithmetic(P, D, info):
    A = {}
    A["travel"] = D["travel"]
    A["arm_throw"] = D["arm_throw"]
    A["arm_angle_req"] = D["arm_angle_req"]
    A["throw_margin"] = D["arm_throw"] - D["travel"]
    rows = []
    for m in GLIDER_MASSES_G:
        W = m * 1e-3 * G
        rows.append({"m": m, "W": W, "F_lo": P["mu_lo"] * W, "F_hi": P["mu_hi"] * W * P["load_factor"]})
    A["force_rows"] = rows
    r_m = P["servo_arm_radius_mm"] / 1000
    A["servo_rows"] = []
    for name, mass, tq in SERVO_CLASSES:
        F = tq * 0.0980665 / r_m * P["usable_torque_frac"]
        A["servo_rows"].append({"name": name, "mass": mass, "tq": tq, "F_usable": F, "ratio": F / rows[-1]["F_hi"]})
    # boom / prop
    prop_r = P["prop_d_in"] * 25.4 / 2
    mfwd = P["motor_diag_mm"] / 2 * math.cos(math.radians(45))
    A["prop_r"], A["motor_fwd"], A["motor_side"] = prop_r, mfwd, mfwd
    A["prop_edge"] = mfwd + prop_r
    A["hang_x"] = P["boom_length_mm"] + D["hang_beyond_rod_end"]
    A["hang_margin"] = A["hang_x"] - A["prop_edge"]
    A["boom_min"] = A["prop_edge"] + P["prop_margin_mm"] - D["hang_beyond_rod_end"]
    A["side_clear_rod"] = mfwd - prop_r - D["Ro"]
    _, _, _, (px0, px1, pya, pyb) = pad_layout(P, D)
    x0 = P["boom_length_mm"] - D["Ls"]
    best = 1e9
    for s in (-1, 1):
        cx, cy = mfwd, s * mfwd
        for (xa, xb, ya, yb) in ((x0 + px0, x0 + px1, pya, pyb), (x0 + px0, x0 + px1, -pyb, -pya)):
            dx = max(xa - cx, 0, cx - xb)
            dy = max(ya - cy, 0, cy - yb)
            best = min(best, math.hypot(dx, dy) - prop_r)
    A["pad_prop_clear"] = best
    # mass / CG (x forward of quad centre; quad CG assumed AT the centre)
    comp = []
    xl = x0
    rod_area = math.pi * (P["rod_d_mm"] - P["rod_wall_mm"]) * P["rod_wall_mm"]
    rod_m = rod_area * D["rod_len"] * 1e-3 * P["rod_density_g_cm3"]
    comp.append(("boom rod", rod_m, (P["boom_rear_x_mm"] + P["boom_length_mm"]) / 2))
    a1 = info["A_tip_latch"]
    comp.append(("tip latch (A)", a1["mass_g"], xl + a1["centroid"][0]))
    a2 = info["A2_servo_pad"]
    comp.append(("servo pad (A2)", a2["mass_g"], xl + a2["centroid"][0]))
    comp.append(("release pin", math.pi / 4 * P["pin_d_mm"] ** 2 * D["pin_cut_len"] * 1e-3 * P["pin_density_g_cm3"], xl + (D["x_hook_closed"] + D["x_tip_closed"]) / 2))
    bs = info["B_frame_saddle"]["mass_g"]
    comp.append(("2 frame saddles (B)", 2 * bs, P["boom_rear_x_mm"] + 10 + P["saddle_spacing_mm"] / 2))
    comp.append(("receiver + plate (C)", P["rx_mass_g"] + info["C_rx_plate"]["mass_g"], 0.0))
    comp.append(("zip ties / tape", P["misc_mass_g"], 0.0))
    A["xl"] = xl
    A["components"] = comp
    A["servo_x"] = xl + (D["servo_xrear"] + D["servo_xfront"]) / 2
    A["glider_x"] = A["hang_x"]
    base_m = sum(c[1] for c in comp)
    base_mom = sum(c[1] * c[2] for c in comp)
    A["cg_rows"] = []
    for name, sm, _ in SERVO_CLASSES:
        m_h = base_m + sm
        mom_h = base_mom + sm * A["servo_x"]
        row = {"servo": name, "hardware_mass": m_h, "no_glider": mom_h / (P["quad_mass_g"] + m_h), "moment_gmm": mom_h}
        for g in GLIDER_MASSES_G:
            row[g] = (mom_h + g * A["glider_x"]) / (P["quad_mass_g"] + m_h + g)
        A["cg_rows"].append(row)
    nine = A["cg_rows"][-1]
    A["added_mass_servo"] = nine["hardware_mass"]
    # pitching moment of the default servo choice with a 30 g glider
    A["pitch_moment_Nmm_30g"] = (nine["moment_gmm"] + 30 * A["glider_x"]) * 1e-3 * G
    return A


# =============================================================================================== files: STL / CSV
def write_stls(outdir, info):
    files = {}
    for n, i in info.items():
        f = outdir / f"{n}.stl"
        ft.write_stl(f, i["tris"], n)
        files[n] = f
    return files


def all_rows(P, D, A, info):
    rows = [(k, P[k], u, t, n) for k, v, u, t, n, hw in PARAMS]
    g = lambda k, v, u, n: rows.append((k, round(v, 3) if isinstance(v, float) else v, u, "GEOMETRY", n))
    a = lambda k, v, u, n: rows.append((k, round(v, 3) if isinstance(v, float) else v, u, "ARITHMETIC", n))
    g("socket_bore_d_mm", 2 * D["Rb"], "mm", "Rod socket bore diameter = rod_d + bore_clear.")
    g("socket_outer_d_mm", 2 * D["Ro"], "mm", "Socket outer diameter.")
    g("pin_hole_d_mm", D["hole_d"], "mm", "Pin hole diameter = pin_d + pin_clear.")
    g("pin_axis_z_mm", D["zpin"], "mm", "Pin axis below the rod axis.")
    g("pin_axis_y_mm", D["yp"], "mm", "Pin axis sideways offset from the rod axis (so the servo clears the rod).")
    g("rail_width_mm", D["kw"], "mm", "Pin rail / eyelet post width.")
    g("servo_shaft_x_mm", D["xs"], "mm", "Servo shaft x relative to socket rear face (socket starts at x=0, tip forward).")
    g("servo_shaft_y_mm", D["ys"], "mm", "Servo shaft y.")
    g("pad_underside_z_mm", D["pad_z0"], "mm", "Underside of the servo pad plate.")
    g("pin_cut_length_mm", D["pin_cut_len"], "mm", "Straight wire to cut: closed tip to the hook, plus the hook leg.")
    g("pin_hook_leg_mm", D["leg_len"], "mm", "Length of the 90-degree bent leg that goes through the arm hole.")
    a("pin_travel_required_mm", A["travel"], "mm", "gap + engage + retract_inside.")
    a("arm_throw_mm", A["arm_throw"], "mm", "2 r sin(theta/2).")
    a("arm_angle_required_deg", A["arm_angle_req"], "deg", "2 asin(travel / 2r).")
    a("prop_edge_x_mm", A["prop_edge"], "mm", "Front prop disc edge forward of the quad centre (motor geometry ASSUMED).")
    a("hang_point_x_mm", A["hang_x"], "mm", "Glider hang point forward of the quad centre.")
    a("hang_margin_mm", A["hang_margin"], "mm", "Hang point minus front prop disc edge.")
    a("boom_length_min_mm", A["boom_min"], "mm", "Smallest boom_length that meets prop_margin.")
    for n, i in info.items():
        a(f"mass_{n}_g", i["mass_g"], "g", "Solid PLA estimate, voxel union (+-2 %).")
        a(f"bbox_{n}_mm", "x".join(f"{v:.1f}" for v in i["bbox"]), "mm", "Print bounding box as written to the STL.")
    return rows


def write_csv(outdir, rows):
    with (outdir / "parameters.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["name", "value", "unit", "tag", "note"])
        for r in rows:
            assert r[3] in ft.SOURCES
            w.writerow(r)


# =============================================================================================== drawing.svg (dark)
def drawing_svg(P, D, A):
    S = 8.0
    W_, H_ = 1200, 840

    def T(x, y, s, size=12, anchor="start", color=INK, weight="normal"):
        return f'<text x="{x:.1f}" y="{y:.1f}" font-family="Segoe UI,Arial" font-size="{size}" text-anchor="{anchor}" fill="{color}" font-weight="{weight}">{html.escape(str(s))}</text>'

    def L(x1, y1, x2, y2, color=INK, w=1.0, dash=None):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        return f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{color}" stroke-width="{w}"{d}/>'

    def poly(pts, fill=PANEL, stroke=INK, w=1.0, dash=None):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        return f'<polygon points="{" ".join(f"{x:.1f},{y:.1f}" for x, y in pts)}" fill="{fill}" stroke="{stroke}" stroke-width="{w}"{d}/>'
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W_} {H_}" width="{W_}" height="{H_}">', f'<rect width="100%" height="100%" fill="{BG}"/>']
    xmin, xmax = -34.0, D["p2b"] + 4
    ox = 40.0
    X = lambda x: ox + (x - xmin) * S
    # ---------- side view (x right, z up)
    zmax = D["pad_z0"] + P["pad_t_mm"] + 2
    loop_draw = min(P["loop_len_mm"], 8.0)
    zmin = D["zpin"] - loop_draw - 1.0
    oy = 100.0
    Zs = lambda z: oy + (zmax - z) * S
    out.append(T(40, 30, "SIDE VIEW (looking at the left side of the boom; quad is to the left, glider hangs on the right)", 14, weight="bold"))
    Ro, Rb, ro = D["Ro"], D["Rb"], P["rod_d_mm"] / 2
    sv = []
    # rod
    sv.append(poly([(X(xmin), Zs(ro)), (X(D["Ls"]), Zs(ro)), (X(D["Ls"]), Zs(-ro)), (X(xmin), Zs(-ro))], "none", MUTED, 1.2, "5 3"))
    # tube
    sv.append(poly([(X(0), Zs(Ro)), (X(D["Ls"]), Zs(Ro)), (X(D["Ls"]), Zs(-Ro)), (X(0), Zs(-Ro))], PANEL, INK, 1.2))
    sv.append(L(X(D["nx0"]), Zs(Ro), X(D["nx0"]), Zs(Ro - P["tie_groove_depth_mm"]), ACCENT, 1.4))
    sv.append(L(X(D["nx1"]), Zs(Ro), X(D["nx1"]), Zs(Ro - P["tie_groove_depth_mm"]), ACCENT, 1.4))
    # rail + posts
    zt, zb = -Ro + 0.3, D["keel_zb"]
    sv.append(poly([(X(0), Zs(zt)), (X(D["Lk1"]), Zs(zt)), (X(D["Lk1"]), Zs(zb)), (X(0), Zs(zb))], PANEL, INK, 1.2))
    sv.append(poly([(X(D["p2a"]), Zs(zt)), (X(D["p2b"]), Zs(zt)), (X(D["p2b"]), Zs(zb)), (X(D["p2a"]), Zs(zb))], PANEL, INK, 1.2))
    bz0 = D["zpin"] + D["hole_d"] / 2 + P["bridge_clear_mm"]
    sv.append(poly([(X(D["Ls"] - 3), Zs(bz0 + P["bridge_t_mm"])), (X(D["p2b"] - 1), Zs(bz0 + P["bridge_t_mm"])), (X(D["p2b"] - 1), Zs(bz0)), (X(D["Ls"] - 3), Zs(bz0))], "#35414d", INK, 1.2))
    # side post + pad
    sv.append(poly([(X(D["post_x0"]), Zs(D["pad_z0"])), (X(D["post_x1"]), Zs(D["pad_z0"])), (X(D["post_x1"]), Zs(zb)), (X(D["post_x0"]), Zs(zb))], "none", INK, 1.0, "4 3"))
    _, _, _, (px0, px1, pya, pyb) = pad_layout(P, D)
    sv.append(poly([(X(px0), Zs(D["pad_z0"] + P["pad_t_mm"])), (X(D["post_x1"] + 1.5), Zs(D["pad_z0"] + P["pad_t_mm"])), (X(D["post_x1"] + 1.5), Zs(D["pad_z0"])), (X(px0), Zs(D["pad_z0"]))], "#35414d", INK, 1.2))
    # servo envelope
    sv.append(poly([(X(D["servo_xrear"]), Zs(D["servo_ztop"])), (X(D["servo_xfront"]), Zs(D["servo_ztop"])), (X(D["servo_xfront"]), Zs(D["servo_zbot"])), (X(D["servo_xrear"]), Zs(D["servo_zbot"]))], "none", BLUE, 1.4, "6 3"))
    # arm and pin
    sv.append(L(X(D["xs"] - D["half_sweep"]), Zs(D["z_arm"]), X(D["xs"] + D["half_sweep"]), Zs(D["z_arm"]), GREEN, 2.0))
    sv.append(L(X(D["x_hook_closed"]), Zs(D["zpin"]), X(D["x_tip_closed"]), Zs(D["zpin"]), ACCENT, 3.0))
    sv.append(L(X(D["x_hook_closed"]), Zs(D["zpin"]), X(D["x_hook_closed"]), Zs(D["zpin"] + D["leg_len"]), ACCENT, 3.0))
    # loop and glider string
    gx = D["Lk1"] + D["gap"] / 2
    sv.append(f'<ellipse cx="{X(gx):.1f}" cy="{Zs(D["zpin"] - loop_draw / 2):.1f}" rx="{1.2 * S:.1f}" ry="{loop_draw / 2 * S:.1f}" fill="none" stroke="{RED}" stroke-width="1.6"/>')
    out += sv
    # markers
    anchors = [(1, -22, ro), (2, 6, Ro), (3, (D["nx0"] + D["nx1"]) / 2, Ro), (4, 2, D["keel_zb"] + 1), (5, D["Lk1"] - 2, D["keel_zb"] + 1),
               (6, gx, D["zpin"] - 4), (7, D["p2b"] - 2, D["keel_zb"] + 1), (8, (D["Ls"] + D["p2b"]) / 2, bz0 + P["bridge_t_mm"]),
               (9, (D["x_hook_closed"] + 8), D["zpin"]), (10, D["xs"] + D["half_sweep"] * 0.2, D["z_arm"]), (11, D["servo_xrear"] + 3, (D["servo_zbot"] + D["servo_ztop"]) / 2),
               (12, D["servo_xrear"] + 2, D["pad_z0"] + P["pad_t_mm"]), (13, (D["post_x0"] + D["post_x1"]) / 2, (zb + D["pad_z0"]) / 2), (14, gx + 0.5, D["zpin"] - loop_draw)]

    def markers(anchors, row_y, mapx, mapy, xlo, xhi):
        pts = sorted([(mapx(ax), n, mapy(ay)) for n, ax, ay in anchors])
        pos = []
        last = xlo - 30
        for px, n, py in pts:
            cx = max(px, last + 28)
            pos.append((cx, n, px, py))
            last = cx
        shift = max(0, pos[-1][0] - xhi)
        res = []
        for cx, n, px, py in pos:
            cx -= shift
            res.append(L(cx, row_y + 11, px, py, MUTED, 0.7))
            res.append(f'<circle cx="{cx:.1f}" cy="{row_y:.1f}" r="11" fill="{BG}" stroke="{ACCENT}" stroke-width="1.2"/>')
            res.append(T(cx, row_y + 4, n, 12, "middle", ACCENT, "bold"))
        return res
    out += markers(anchors, 62, X, lambda z: Zs(z), 40, 800)
    # dimension line under the side view
    dy = Zs(zmin) + 8 + 6
    out.append(L(X(0), dy, X(D["p2b"]), dy, MUTED, 1))
    out.append(L(X(0), dy - 4, X(0), dy + 4, MUTED))
    out.append(L(X(D["p2b"]), dy - 4, X(D["p2b"]), dy + 4, MUTED))
    out.append(T((X(0) + X(D["p2b"])) / 2, dy + 16, f"latch length {D['p2b']:.1f} mm  (socket {D['Ls']:.0f}, rail+posts to the tip)", 12, "middle", MUTED))
    # ---------- top view (x right, y up)
    ytop = max(pyb, D["ys"] + P["servo_w_mm"] / 2) + 2
    ybot = D["post_y0"] - 2
    oy2 = 548.0
    Ys = lambda y: oy2 + (ytop - y) * S
    out.append(T(40, oy2 - 56, "TOP VIEW (looking down; the servo pad is drawn dashed because it is ABOVE everything else)", 14, weight="bold"))
    tv = []
    tv.append(poly([(X(xmin), Ys(ro)), (X(D["Ls"]), Ys(ro)), (X(D["Ls"]), Ys(-ro)), (X(xmin), Ys(-ro))], "none", MUTED, 1.2, "5 3"))
    tv.append(poly([(X(0), Ys(Ro)), (X(D["Ls"]), Ys(Ro)), (X(D["Ls"]), Ys(-Ro)), (X(0), Ys(-Ro))], PANEL, INK, 1.2))
    for xx in (D["nx0"], D["nx1"]):
        tv.append(L(X(xx), Ys(Ro), X(xx), Ys(-Ro), ACCENT, 1.4))
    tv.append(L(X(0), Ys(0), X(D["Ls"]), Ys(0), "#35414d", P["slit_mm"] * S))
    yk0, yk1 = D["yp"] - D["kw"] / 2, D["yp"] + D["kw"] / 2
    tv.append(poly([(X(D["Ls"]), Ys(yk0)), (X(D["Lk1"]), Ys(yk0)), (X(D["Lk1"]), Ys(yk1)), (X(D["Ls"]), Ys(yk1))], PANEL, INK, 1.2))
    tv.append(poly([(X(D["p2a"]), Ys(yk0)), (X(D["p2b"]), Ys(yk0)), (X(D["p2b"]), Ys(yk1)), (X(D["p2a"]), Ys(yk1))], PANEL, INK, 1.2))
    tv.append(poly([(X(D["Ls"] - 3), Ys(yk1 - 0.2)), (X(D["p2b"] - 1), Ys(yk1 - 0.2)), (X(D["p2b"] - 1), Ys(yk0 + 0.2)), (X(D["Ls"] - 3), Ys(yk0 + 0.2))], "none", INK, 1.0, "3 2"))
    tv.append(poly([(X(D["post_x0"]), Ys(D["post_y0"])), (X(D["post_x1"]), Ys(D["post_y0"])), (X(D["post_x1"]), Ys(D["post_y1"])), (X(D["post_x0"]), Ys(D["post_y1"]))], PANEL, INK, 1.2))
    outline, slots, tie_x, _ = pad_layout(P, D)
    tv.append(poly([(X(x), Ys(y)) for x, y in outline], "none", INK, 1.0, "6 3"))
    for s in slots:
        tv.append(poly([(X(x), Ys(y)) for x, y in s], "none", MUTED, 0.8))
    tv.append(poly([(X(D["servo_xrear"]), Ys(D["ys"] + P["servo_w_mm"] / 2)), (X(D["servo_xfront"]), Ys(D["ys"] + P["servo_w_mm"] / 2)), (X(D["servo_xfront"]), Ys(D["ys"] - P["servo_w_mm"] / 2)), (X(D["servo_xrear"]), Ys(D["ys"] - P["servo_w_mm"] / 2))], "none", BLUE, 1.4, "6 3"))
    tv.append(f'<circle cx="{X(D["xs"]):.1f}" cy="{Ys(D["ys"]):.1f}" r="{2.0 * S / 2:.1f}" fill="none" stroke="{GREEN}" stroke-width="1.6"/>')
    # arm in both end positions and mid
    for ang, dash in ((-P["servo_throw_deg"] / 2, None), (0, "3 3"), (P["servo_throw_deg"] / 2, None)):
        hx = D["xs"] + P["servo_arm_radius_mm"] * math.sin(math.radians(ang))
        hy = D["ys"] - P["servo_arm_radius_mm"] * math.cos(math.radians(ang))
        tv.append(L(X(D["xs"]), Ys(D["ys"]), X(hx), Ys(hy), GREEN, 1.6, dash))
    tv.append(L(X(D["x_hook_closed"]), Ys(D["yp"]), X(D["x_tip_closed"]), Ys(D["yp"]), ACCENT, 3.0))
    out += tv
    anchors2 = [(1, -22, 0), (2, 6, -Ro), (3, (D["nx0"] + D["nx1"]) / 2, -Ro), (4, 2, yk0), (5, D["Lk1"] - 2, yk0), (6, D["Lk1"] + D["gap"] / 2, yk0),
                (7, D["p2b"] - 2, yk0), (8, D["Ls"] + 4, yk1 - 0.2), (9, D["x_hook_closed"] + 10, D["yp"]), (10, D["xs"] - 4, D["ys"] - 5),
                (11, D["servo_xrear"] + 6, D["ys"] + P["servo_w_mm"] / 2), (12, D["servo_xrear"] + 1, pyb), (13, D["post_x0"] + 3, D["post_y0"])]
    out += markers(anchors2, oy2 - 28 + 0, X, Ys, 40, 800)
    # put second marker row *below* the title: shift handled by row y; leaders go down to anchors
    # ---------- legend and key dimensions
    lx = 840
    legend = [(1, "boom rod (your rod, dashed)"), (2, "A: socket tube, clamps on the rod, slit on top"), (3, "zip-tie groove round socket and rail"),
              (4, "pin rail: long guide hole, rear support"), (5, "eyelet post 1 (near the quad)"), (6, "gap with the string loop on the pin"),
              (7, "eyelet post 2 (far end of the pin)"), (8, "bridge over the gap (loop falls out downward)"), (9, "release pin (orange)"),
              (10, "servo arm, hook through the arm hole (green)"), (11, "servo, your size (blue dashes)"), (12, "A2: servo pad with zip-tie slots"),
              (13, "post for the pad (M2 screws or glue)"), (14, "string loop and glider")]
    out.append(T(lx, 30, "Parts (numbers in the drawings)", 14, weight="bold"))
    for i, (n, s) in enumerate(legend):
        yy = 56 + i * 22
        out.append(f'<circle cx="{lx + 10}" cy="{yy - 4}" r="9" fill="{BG}" stroke="{ACCENT}" stroke-width="1"/>')
        out.append(T(lx + 10, yy, n, 11, "middle", ACCENT, "bold"))
        out.append(T(lx + 28, yy, s, 12))
    ky = 56 + len(legend) * 22 + 30
    out.append(T(lx, ky, "Key dimensions (mm)", 14, weight="bold"))
    dims = [f"rod socket bore {2 * D['Rb']:.2f}  (rod {P['rod_d_mm']:.2f} ASSUMED)", f"socket outer {2 * Ro:.2f}", f"pin hole {D['hole_d']:.2f}  (pin {P['pin_d_mm']:.2f} ASSUMED)",
            f"pin axis {abs(D['zpin']):.2f} below, {D['yp']:.2f} beside the rod axis", f"post {D['te']:.1f}, gap {D['gap']:.1f}, engage {P['engage_mm']:.1f}",
            f"pin travel needed {D['travel']:.1f}, arm throw {D['arm_throw']:.1f}", f"servo shaft {abs(D['xs']):.1f} behind socket rear, {D['ys']:.1f} to the side",
            f"pad underside {D['pad_z0']:.1f} above rod axis", f"pin wire cut length about {D['pin_cut_len']:.0f}"]
    for i, s in enumerate(dims):
        out.append(T(lx, ky + 22 + i * 20, s, 12, color=MUTED))
    out.append(T(40, H_ - 14, "Design output only. Nothing here has been printed, fitted or flown. Servo, rod, pin and frame sizes are ASSUMED: measure yours and re-run.", 12, color=RED))
    out.append("</svg>")
    return "\n".join(out)


# =============================================================================================== index.html
def n1(x, d=1):
    return f"{x:.{d}f}"


def index_html(P, D, A, info, files, rows):
    h = html.escape
    tg = ft.tag
    css = ft.CSS + ".warn{border-left:3px solid var(--bad);padding:2px 0 2px 12px;color:var(--ink)}li.todo::marker{content:'[ ] '}"
    pb = lambda n: "x".join(f"{v:.1f}" for v in info[n]["bbox"])
    hw = [p for p in PARAMS if p[5]]
    cli_for = {v: k for k, v in CLI_FLAGS.items()}
    Pg = []
    Pg.append(f"<!doctype html><html lang='en'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'><title>Glider drop-release for a small FPV quad</title><style>{css}</style></head><body><main>")
    Pg.append("<section><h1>Glider drop-release for a small FPV quad</h1>")
    Pg.append("<p class='box'><b>Design output only. Nothing here has been printed, fitted or flown.</b> Every size that depends on hardware Kai has not measured (boom rod, pin wire, servo, frame arm, quad geometry) is a named, tagged parameter at the top of <code>rigid_glider/drop_release.py</code>. Measure yours and re-run the script; the parts, drawing, numbers and this page all regenerate.</p>")
    Pg.append("<p>Tags: GEOMETRY = exact from the model, ARITHMETIC = a sum from stated inputs, JUDGEMENT = a design choice made here, ASSUMED = hardware or physics not measured, UNKNOWN = not known and not modelled.</p></section>")
    # what it is
    Pg.append("<section><h2>What it is</h2>")
    Pg.append("<p>Last time an open hook under the Vapor5 let rotor downwash shake the glider off within 3 m of take-off. This concept (1) holds the glider on a light boom that sticks out forward past the front propellers, so it hangs outside the downwash column, and (2) replaces the open hook with a positive pin release: a straight wire pin slides through two printed eyelets with the glider's string loop trapped between them. A small servo pulls the pin straight back toward the quad and the loop drops out.</p>")
    Pg.append(f"<p>The servo is driven by a separate PWM radio receiver carried on the quad, so nothing in the flight controller changes. How big the downwash column is, and how calm the air is outside it, are {tg('UNKNOWN')} here and are <b>not modelled</b>. The boom length only sets a margin outside the propeller disc.</p></section>")
    # diagram
    Pg.append("<section><h2>Assembly diagram</h2><figure><img src='drawing.svg' alt='Side and top view of the tip latch, servo pad and boom'><figcaption>drawing.svg: side and top view with numbered parts (legend in the drawing). Servo, rod and pin are drawn at the ASSUMED sizes below.</figcaption></figure></section>")
    # print list
    Pg.append("<section><h2>Print list</h2><div class='scroll'><table><tr><th>Part</th><th>File</th><th>Qty</th><th>Print bounding box, mm (x y z)</th><th>Solid PLA mass each, g</th><th>Orientation / note</th></tr>")
    notes = {"A_tip_latch": "Print upright as modelled: the flat rail bottom on the bed, rod socket axis horizontal, the vertical post standing. Brim recommended (narrow footprint). UNSURE: the tall thin post and the roof of the socket bore may need tuning; no supports are designed in.",
             "A2_servo_pad": "Print flat, as written (z = 0 is the pad underside). Screw to the post with two M2 screws or glue with CA.",
             "B_frame_saddle": "Print flat on its base plate; rod groove faces up. Print 2. The groove has three anti-slide ribs; the side walls and the plate on top stop rotation.",
             "C_rx_plate": "Print flat. Optional; or just tape the receiver."}
    qty = {"A_tip_latch": 1, "A2_servo_pad": 1, "B_frame_saddle": 2, "C_rx_plate": 1}
    for n in ("A_tip_latch", "A2_servo_pad", "B_frame_saddle", "C_rx_plate"):
        Pg.append(f"<tr><td>{n}</td><td><a href='{n}.stl'>{n}.stl</a></td><td class='n'>{qty[n]}</td><td class='n'>{pb(n)}{tg('GEOMETRY')}</td><td class='n'>{n1(info[n]['mass_g'], 2)}{tg('ARITHMETIC')}</td><td>{h(notes[n])}</td></tr>")
    Pg.append(f"</table></div><p class='note'>Mass = voxel-union volume x {P['pla_density_g_cm3']} g/cm&sup3;{tg('ASSUMED')}, 100 % solid, +-2 %. A real print with 2-3 perimeters and low infill will weigh less. Weigh yours. Parts are built from overlapping solids (no shared faces) so every STL edge is shared by exactly two triangles; a slicer unions them.</p>")
    Pg.append(f"<p>Also needed, not printed: a straight pin of {P['pin_d_mm']:.2f} mm wire{tg('ASSUMED')} about {D['pin_cut_len']:.0f} mm long{tg('ARITHMETIC')}, a boom rod{tg('ASSUMED')}, zip ties of {P['zip_tie_w_mm']} mm width{tg('ASSUMED')}, and optional M2 screws. Part C is trivial (a plate with zip-tie slots), so it is included; taping the receiver is also fine.</p></section>")
    # assembly
    Pg.append("<section><h2>Assembly steps</h2><ol>")
    steps = [
        f"Test-fit the rod in the socket of A. The bore is {n1(2 * D['Rb'], 2)} mm{tg('GEOMETRY')} for a {P['rod_d_mm']:.2f} mm rod{tg('ASSUMED')}. If it is too tight or loose, change <code>bore_clear_mm</code> and re-run.",
        "Slide the socket onto the rod end so the rod end is flush with the front face of the socket tube. Pull a zip tie round the groove (round the tube and the rail together) and tighten. Pull-test it by hand before trusting it.",
        f"Pass the pin through the hole in the rail from the rear to check it slides freely, then pull it back. Pin hole {n1(D['hole_d'], 2)} mm{tg('GEOMETRY')}. Ream with a drill of that size if the print is tight.",
        f"Zip-tie the servo to the pad A2 using the four slots (two ties round the servo body). Servo size used: {P['servo_l_mm']} x {P['servo_w_mm']} x {P['servo_h_mm']} mm{tg('ASSUMED')}; the arm must end up in line with the pin. If your servo is a different size, re-run with its numbers.",
        "Fix the pad A2 on top of the vertical post of A with two M2 screws (1.7 mm pilot holes in the post, 2.2 mm clear holes in the pad) or glue.",
        "Centre the servo, fit the arm pointing across the boom, and make the pin bend (below). Hook the pin into the arm hole, then thread it through the rail, the gap and post 2.",
        f"Zip-tie the two saddle blocks B to the frame arm or plate at about {P['saddle_spacing_mm']:.0f} mm spacing{tg('JUDGEMENT')} with the rod in the grooves, plate on top of the rod (or the same block upside down for a rod on top). Tie slots are sized for {P['frame_arm_w_mm']} mm arm width{tg('ASSUMED')}.",
        "Tape or tie the PWM receiver (plate C optional) on the quad away from the servo lead tension. Route the servo lead along the boom with a loose loop and tie it down so it cannot reach a propeller.",
        "With the servo powered and centred by hand, set the end points so the pin travels the required distance (table below) and no further, then bench test.",
    ]
    for s in steps:
        Pg.append(f"<li>{s}</li>")
    Pg.append("</ol>")
    Pg.append(f"<div class='box'><h3>The pin and the bend Kai must make</h3><p>Use a straight, hard, smooth wire of {P['pin_d_mm']:.2f} mm{tg('ASSUMED')}, cut to about {D['pin_cut_len']:.0f} mm{tg('ARITHMETIC')} (closed tip to the arm hole plus the leg). At the rear end make <b>one 90 degree bend</b> and leave a leg of about {D['leg_len']} mm{tg('GEOMETRY')} pointing up. The leg goes up through the outer hole of the servo arm from below (hole at {P['servo_arm_radius_mm']} mm from the shaft{tg('ASSUMED')}). Then bend the last 1 to 2 mm of the leg over by about 45 degrees (or add a tiny piece of heat-shrink) so it cannot slip out of the hole. Do this with the arm already on the servo and the servo at the pin-closed end of its travel. The hooked leg ties the pin to the arm, so it cannot fall out; at the open end point the tip stays inside post 1 (retract-inside), so it is still guided, and the long rail hole plus the hooked leg stop it rotating. The arm end points, not the printed parts, set the pin travel: do not over-travel. Because the arm hole moves on an arc, the pin end swings sideways by about {n1(P['servo_arm_radius_mm'] * (1 - math.cos(math.radians(P['servo_throw_deg'] / 2))), 1)} mm{tg('ARITHMETIC')} over the full throw: the leg needs a loose hole and the wire a little flex. A pin too stiff to flex will bind: check on the bench.</p></div></section>")
    # arithmetic
    Pg.append("<section><h2>Numbers (ARITHMETIC; assumptions stated)</h2>")
    Pg.append("<h3>Pin travel versus servo arm throw</h3>")
    Pg.append(f"<p>Pin travel needed = gap + engagement + retract-inside = {P['gap_mm']} + {P['engage_mm']} + {P['retract_inside_mm']} = <b>{n1(A['travel'])} mm</b>{tg('ARITHMETIC')} (all three are design values{tg('JUDGEMENT')}). Arm throw at radius r and total angle &theta; is 2 r sin(&theta;/2) = 2 x {P['servo_arm_radius_mm']} x sin({P['servo_throw_deg']/2:g} deg) = <b>{n1(A['arm_throw'])} mm</b>{tg('ARITHMETIC')} with r and &theta; {tg('ASSUMED')}. Margin {n1(A['throw_margin'])} mm. The angle you actually need is {n1(A['arm_angle_req'], 0)} deg{tg('ARITHMETIC')}, so set the end points for that and the rest of the arm travel is spare. The pin rail ends {n1(P['hook_clear_mm'])} mm{tg('JUDGEMENT')} behind the arm hole's most forward position.</p>")
    Pg.append("<h3>Pull force on the pin against friction</h3>")
    Pg.append(f"<p>The glider hangs from the pin, so the pin carries the glider weight W = m g and slides against it. Pull force = &mu; W. Friction coefficient range {P['mu_lo']} to {P['mu_hi']}{tg('ASSUMED')} (string on wire plus wire in a printed hole, combined; not measured). High case also multiplied by a load/binding factor of {P['load_factor']}{tg('JUDGEMENT')}.</p>")
    Pg.append("<div class='scroll'><table><tr><th>Glider mass, g</th><th>Weight, N</th><th>Pull, low &mu;, N</th><th>Pull, high &mu; x factor, N</th></tr>")
    for r in A["force_rows"]:
        Pg.append(f"<tr><td class='n'>{r['m']:g}</td><td class='n'>{r['W']:.3f}</td><td class='n'>{r['F_lo']:.3f}</td><td class='n'>{r['F_hi']:.3f}</td></tr>")
    Pg.append(f"</table></div><p>Servo pull at the pin = stall torque / arm radius, then x {P['usable_torque_frac']}{tg('JUDGEMENT')} usable. <b>Stall torques below are ASSUMED round numbers for each size class, not any product's spec.</b> Use your servo's real figure. Arm radius {P['servo_arm_radius_mm']} mm{tg('ASSUMED')}.</p>")
    Pg.append("<div class='scroll'><table><tr><th>Servo class</th><th>Stall torque, kg.cm</th><th>Usable pull, N</th><th>Ratio to high-case 30 g pull</th></tr>")
    for s in A["servo_rows"]:
        Pg.append(f"<tr><td>{s['name']}</td><td class='n'>{s['tq']}{tg('ASSUMED')}</td><td class='n'>{s['F_usable']:.2f}{tg('ARITHMETIC')}</td><td class='n'>{s['ratio']:.1f}x{tg('ARITHMETIC')}</td></tr>")
    Pg.append("</table></div><p>On these assumptions even the smallest class has margin over the friction pull. If the pin binds because of a bent pin or tight holes the margin vanishes, which is what the bench test is for.</p>")
    Pg.append("<h3>Boom tip versus the front propeller disc</h3>")
    Pg.append(f"<p>Assumed true-X frame with motor-to-motor diagonal {P['motor_diag_mm']:.0f} mm{tg('ASSUMED')}, so each front motor sits {n1(A['motor_fwd'])} mm forward and {n1(A['motor_side'])} mm to the side of the centre{tg('ARITHMETIC')}. Propeller {P['prop_d_in']:g} inch{tg('ASSUMED')} gives a disc radius of {n1(A['prop_r'])} mm. Front disc edge = {n1(A['prop_edge'])} mm forward of the centre{tg('ARITHMETIC')}. <b>Measure the real motor spacing and re-run.</b></p>")
    Pg.append(f"<p>Default boom length {P['boom_length_mm']:.0f} mm (rod end forward of the centre){tg('JUDGEMENT')}. The glider hang point is {n1(D['hang_beyond_rod_end'])} mm past the rod end{tg('GEOMETRY')}, i.e. {n1(A['hang_x'])} mm forward of the centre. That is <b>{n1(A['hang_margin'])} mm</b> beyond the front disc edge{tg('ARITHMETIC')}, against the required margin of {P['prop_margin_mm']:.0f} mm{tg('JUDGEMENT')}. The shortest boom that meets the margin is {n1(A['boom_min'])} mm{tg('ARITHMETIC')}. The rod passes {n1(A['side_clear_rod'])} mm clear of each front disc sideways{tg('ARITHMETIC')}, and the servo pad corners are {n1(A['pad_prop_clear'])} mm clear of the disc{tg('ARITHMETIC')}. The size of the downwash column and how calm the air outside it is are {tg('UNKNOWN')} and not modelled; the margin is a judgement, not a result. The hanging glider's own length behind the hang point is {tg('UNKNOWN')} here: check it clears the props.</p>")
    Pg.append("<h3>Added mass and balance</h3>")
    Pg.append(f"<p>Quad mass {P['quad_mass_g']:.0f} g{tg('ASSUMED')} with its centre of gravity taken at the frame centre{tg('ASSUMED')}. Everything below is measured forward of that point. CG shift = sum(m x) / total mass. Part masses are the solid-PLA estimates above, rod {P['rod_density_g_cm3']} g/cm&sup3; hollow tube{tg('ASSUMED')}, pin steel, plus {P['rx_mass_g']} g receiver{tg('ASSUMED')} and {P['misc_mass_g']} g ties/tape{tg('ASSUMED')}.</p>")
    Pg.append("<div class='scroll'><table><tr><th>Item</th><th>Mass, g</th><th>Position, mm forward of centre</th></tr>")
    for nme, m, x in A["components"]:
        Pg.append(f"<tr><td>{nme}</td><td class='n'>{m:.2f}{tg('ARITHMETIC')}</td><td class='n'>{x:.0f}{tg('ARITHMETIC')}</td></tr>")
    Pg.append(f"<tr><td>servo (class below)</td><td class='n'>by class</td><td class='n'>{A['servo_x']:.0f}{tg('ARITHMETIC')}</td></tr><tr><td>glider on the hook</td><td class='n'>10 / 20 / 30</td><td class='n'>{A['glider_x']:.0f}{tg('ARITHMETIC')}</td></tr></table></div>")
    Pg.append("<div class='scroll'><table><tr><th>Servo class (mass)</th><th>Hardware mass, g</th><th>CG shift, no glider, mm</th><th>CG shift, 10 g glider</th><th>20 g</th><th>30 g</th></tr>")
    for r in A["cg_rows"]:
        Pg.append(f"<tr><td>{r['servo']} ({[s[1] for s in SERVO_CLASSES if s[0] == r['servo']][0]} g{tg('ASSUMED')})</td><td class='n'>{r['hardware_mass']:.1f}</td><td class='n'>{r['no_glider']:.1f}</td><td class='n'>{r[10.0]:.1f}</td><td class='n'>{r[20.0]:.1f}</td><td class='n'>{r[30.0]:.1f}</td></tr>")
    nine = A["cg_rows"][-1]
    Pg.append(f"</table></div><p>All shifts are <b>forward</b>, ARITHMETIC. With the {SERVO_CLASSES[-1][1]} g servo and a 30 g glider the pitching moment about the centre is {A['pitch_moment_Nmm_30g']:.0f} N.mm; when the glider is released the CG jumps back by {nine[30.0] - nine['no_glider']:.1f} mm{tg('ARITHMETIC')}. Whether the quad can take that without re-balancing the battery is for Kai to judge; it depends on the real quad mass and CG, both {tg('ASSUMED')}. A forward-shifted battery or a rear counterweight is the usual remedy; none is designed here.</p></section>")
    # wiring
    Pg.append("<section><h2>Wiring and radio setup: checklist to confirm</h2><p class='note'>This is a list of things to check against <b>your own receiver's documentation</b>. It is not a verified procedure and none of it has been tested.</p><ul>")
    for t in ["A second ExpressLRS receiver with PWM outputs, bound with the same binding phrase, responds to the same transmitter as the quad's own receiver.",
              "Telemetry is disabled on the extra receiver, so two receivers do not both transmit back to the radio.",
              "Channel 5 is the ELRS arming channel on the usual setup, so the servo goes on a different AUX channel that has a spare switch on the radio. Check which channel numbers your receiver maps to which pin.",
              "Power: take 5 V from a pad whose BEC current rating covers the servo's peak current plus whatever else it feeds. Small servos preferred. Check the rating, and check the receiver's own supply range.",
              "Set the PWM receiver's failsafe for that channel to the pin-CLOSED position, so a lost link holds the glider rather than dropping it."]:
        Pg.append(f"<li class='todo'>{h(t)} <i>(confirm in your receiver's docs)</i></li>")
    Pg.append("</ul></section>")
    # preflight
    Pg.append("<section><h2>Pre-flight checklist</h2><ul>")
    for t in ["Bench test the release 10 times in a row with a dummy weight of the heaviest glider mass (30 g) hanging on the string loop. Every release must be clean; the pin must slide back without sticking.",
              "Check the propellers clear the boom, the string and the hanging glider at full stick deflection (all axes, props spinning is NOT part of this check: props off).",
              "Failsafe check on the bench (props off): turn the transmitter off and confirm the pin stays closed (hold), not released.",
              "Check the rod cannot slide or rotate in the socket and the saddles under hand loading; check zip ties are tight and the tails trimmed.",
              "Check the servo lead and the pin hook cannot catch on a propeller.",
              "Weigh the quad with everything fitted; check the balance (CG) against the numbers above before flying.",
              "Dummy drop before a real glider: fly with a dummy weight and release it over soft ground.",
              "Fly gently with a slight forward drift. Release in a steady hover or in slow forward flight, not during a manoeuvre."]:
        Pg.append(f"<li class='todo'>{h(t)}</li>")
    Pg.append("</ul>")
    Pg.append(f"<p><b>String loop (JUDGEMENT):</b> a loop about {P['loop_len_mm']:.0f} mm long{tg('JUDGEMENT')} tied from a thin, smooth, low-stretch braided line (about 0.3 to 0.5 mm; polyester or fishing-braid thread), knot dabbed with glue, tied through the glider's release hole or nose. Avoid stiff or kinked monofilament that can jam in the gap. The gap is {P['gap_mm']} mm{tg('JUDGEMENT')} so a small knot and two strands fit. No stiff eye on the glider is needed.</p></section>")
    # assumptions
    Pg.append("<section><h2>Assumed or unknown: measure yours and re-run</h2><p>The script accepts the main ones as flags; any other parameter can be edited at the top of the script.</p><div class='scroll'><table><tr><th>Parameter</th><th>Default used</th><th>Unit</th><th>Tag</th><th>What to measure</th><th>Re-run flag</th></tr>")
    for k, v, u, t, nte, _ in hw:
        Pg.append(f"<tr><td><code>{k}</code></td><td class='n'>{P[k]:g}</td><td>{u}</td><td>{t}</td><td>{h(nte)}</td><td><code>{cli_for.get(k, 'edit PARAMS')}</code></td></tr>")
    Pg.append("</table></div><ul>")
    for t in ["Size of the downwash column and how calm the air outside it is: UNKNOWN, not modelled.",
              "Vapor5 frame dimensions, arm width and mounting holes: UNKNOWN.",
              "Servo models and dimensions (2.1 g, 5 g, 9 g): UNKNOWN. Stall torques in the table are ASSUMED round numbers.",
              "Friction between the string, the pin and the printed holes: ASSUMED range only.",
              "Whether the PLA parts print and assemble as drawn (bore fit, tie groove strength, the thin tall post, the bridging over the pin gap): UNKNOWN until printed.",
              "Nothing here has been printed, fitted or flown."]:
        Pg.append(f"<li>{h(t)}</li>")
    Pg.append("</ul></section>")
    # all parameters
    Pg.append("<section><h2>All parameters</h2><div class='scroll'><table><tr><th>Name</th><th>Value</th><th>Unit</th><th>Tag</th><th>Note</th></tr>")
    for k, v, u, t, nte in rows:
        Pg.append(f"<tr><td><code>{k}</code></td><td class='n'>{h(str(v))}</td><td>{u}</td><td><span class='tag'>[{t}]</span></td><td>{h(nte)}</td></tr>")
    Pg.append("</table></div><p>Same table as <a href='parameters.csv'>parameters.csv</a>.</p></section>")
    Pg.append("</main></body></html>")
    return "".join(Pg)


# =============================================================================================== main
def apply_overrides(P, args):
    for flag, key in CLI_FLAGS.items():
        v = getattr(args, key, None)
        if v is not None:
            P[key] = v
    for kv in args.set or []:
        k, v = kv.split("=", 1)
        if k not in P:
            raise SystemExit(f"unknown parameter {k}")
        P[k] = float(v)
    return P


def main(argv=None):
    ap = argparse.ArgumentParser(description="Glider drop-release parts generator (design output only).")
    ap.add_argument("--out", default=str(OUT_DEFAULT))
    for flag, key in CLI_FLAGS.items():
        ap.add_argument(flag, dest=key, type=float, help=f"override {key}")
    ap.add_argument("--set", action="append", metavar="NAME=VALUE", help="override any other parameter")
    args = ap.parse_args(argv)
    P = apply_overrides(default_params(), args)
    outdir = Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)
    D, parts, info = build(P)
    A = arithmetic(P, D, info)
    files = write_stls(outdir, info)
    rows = all_rows(P, D, A, info)
    write_csv(outdir, rows)
    (outdir / "drawing.svg").write_text(drawing_svg(P, D, A), encoding="utf-8")
    (outdir / "index.html").write_text(index_html(P, D, A, info, files, rows), encoding="utf-8")
    print(f"wrote {outdir}")
    for n, i in info.items():
        rep = ft.edge_report(i["tris"])
        print(f"  {n}: bbox {'x'.join(f'{v:.1f}' for v in i['bbox'])} mm, mass {i['mass_g']:.2f} g solid, tris {len(i['tris'])}, edges {rep[0]} bad {rep[1]} same-dir {rep[2]} degenerate {rep[3]}, volume {ft.mesh_stats(i['tris'])['volume']:.0f} mm3")
    print(f"  pin travel {A['travel']:.1f} mm, arm throw {A['arm_throw']:.1f} mm, hang margin {A['hang_margin']:.1f} mm, boom min {A['boom_min']:.1f} mm")
    return D, A, info


if __name__ == "__main__":
    main()
