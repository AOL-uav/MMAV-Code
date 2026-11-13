"""Flat-wing flight-test package generator: two plain flat printed wings (F45, F60), tip fins, a fuselage stick, drawings, a
test page and a log template.  NOTHING here is a prediction that a wing will fly: our XFOIL chain cannot model flat plates
(results/runs/rigid_glider/plate/summary.txt).  Every number carries a source tag:
  GEOMETRY (exact), ARITHMETIC (simple sum, inputs stated), AVL (outline only, no section effects), ASSUMED (what), UNKNOWN (to be measured).

Writes ONLY under results/runs/rigid_glider/flat_test/ (new folder).  Imports (never edits) avl.run_avl and geometry.Wing; AVL runs happen
in flat_test/avl_work/.   Regenerate:   py -B rigid_glider/flat_test.py [--span 250] [--ar 4.5 6.0] [--thickness 0.6] [--plain] [--no-foam] [--no-plain-extra] [--out DIR] [--no-avl]
"""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import argparse
import csv
import html
import json
import math
import struct
from pathlib import Path

# =============================================================================================== PARAMETERS (Kai: change these)
SPAN_MM = 250.0                    # wing span
AR_LIST = [4.5, 6.0]               # aspect ratios; F45 = P07 outline scaled, F60 = slimmer.  Names are "F" + round(10*AR)
TAPER = 0.5                        # tip chord / root chord
SWEEP_QC_DEG = 25.0                # quarter-chord sweep
DIHEDRAL_DEG = 0.0                 # ZERO: a one-piece wing printed flat on the bed cannot have dihedral (the model used 3 deg per panel)
THICKNESS_MM = 0.6                 # default plate thickness = 3 layers of 0.2 mm
THICKNESS_TABLE_MM = [0.4, 0.6, 0.8, 1.0]   # thicknesses used in the mass / bend tables
LAYER_MM = 0.2                     # print layer height
RIB_WIDTH_MM = 1.2                 # spar rib on the printed wings (default; --plain removes it): width
RIB_HEIGHT_MM = 2.5                # height above the plate (printed on top, FLOWN RIB-DOWN)
RIB_TIP_STOP_MM = 3.0              # the rib stops this far short of the tip edge (and the tip tabs)
RIB_CENTRE_GAP_EXTRA_MM = 0.4      # centre gap in the rib = stick width + this, so the stick glues flat to the plate
RIB_OVERLAP_MM = 0.1               # the rib shells overlap the plate by this much (a slicer joins them)
FOAM_SPAN_MM = 400.0               # foamboard specimen B45: the F45 outline (= P07) at this span
FOAM_AR = 4.5
FOAM_THICKNESS_MM = 5.0            # ASSUMED foamboard thickness: Kai to measure
FOAM_AREAL_MASS_G_M2 = 300.0       # ASSUMED areal mass of light craft foamboard with paper: to be weighed
STICK_LENGTH_WARN_MM = 180.0       # the page warns if the stick is longer than this
FIN_HEIGHT_MM_AT_400 = 24.0        # fin height at span 400 mm; scales with span/400 (15 mm at 250)
FIN_THICKNESS_MM = None            # None = same as the wing plate
TAB_LEN_FRAC_OF_TIP_CHORD = 0.20   # each of the two tabs on a wing tip edge
TAB_POS_FRACS = (0.25, 0.75)       # tab centres along the tip chord, fraction from the leading edge
SLOT_CLEARANCE_MM = 0.1            # slot width = plate thickness + this
SLOT_LEN_CLEARANCE_MM = 0.1        # slot length = tab length + this
STICK_WIDTH_MM = 4.0               # stick thickness in y (printed lying on its side)
STICK_DEPTH_MM = 7.0               # stick height below the saddle (room for the ballast hole row)
NOSE_FRAC_OF_ROOT_CHORD = 0.40     # stick extends this fraction of the root chord ahead of the root leading edge
BALLAST_HOLE_MM = 3.2              # square through-holes (an M3 screw fits) along the nose AND under the wing
BALLAST_PITCH_MM = 5.0             # hole pitch; hole centres sit half a pitch off the 5 mm marks
RELEASE_HOLE_MM = 2.5              # square hole near the nose for a release pin / catapult hook
TICK_WIDTH_MM = 0.4                # tick notch cut up from the bottom edge: 1 mm ticks, longer at 5, longest and wider at 10
TICK_DEPTHS_MM = (0.6, 1.2, 1.8)   # at 1 mm, 5 mm, 10 mm
LOOP_NOTCH_WIDTH_MM = 1.0          # the 5 mm and 10 mm marks are wider notches: they also seat the drone-drop thread loop
DROP_HEIGHT_M = 10.0               # JUDGEMENT: suggested drone-drop height
DROP_FORWARD_NOTE = 'roughly the glide speed for the wing loading'
COINS_G = (('US penny', 2.5), ('US dime', 2.27), ('US nickel', 5.0))   # ASSUMED nominal masses: weigh yours
TICK_MARGIN_MM = 5                 # ticks extend this far beyond the CG test range
PLA_DENSITY_G_CM3 = 1.24           # ASSUMED, 100 % solid
E_PLA_GPA = 3.3                    # ASSUMED Young's modulus for the rough bend estimate
RHO_AIR = 1.225                    # ASSUMED sea-level air density, kg/m3
NU_AIR = 1.5e-5                    # ASSUMED kinematic viscosity, m2/s
GLUE_MASS_G = 0.0                  # glue / tape not included: weigh it (UNKNOWN)
CG_START_FRAC_MAC_AHEAD_OF_NP = 0.15   # JUDGEMENT: first CG setting is this fraction of MAC ahead of the neutral point
CG_END_FRAC_MAC_BEHIND_NP = 0.08   # JUDGEMENT: the printed table runs aft to at least this fraction of MAC BEHIND the neutral point (then the stop rule)
CG_STEP_MM = 1.0                   # JUDGEMENT: step aft
RELEASES_PER_SETTING = 5           # JUDGEMENT
RELEASES_AT_GLIDING_SETTING = 10   # JUDGEMENT
PASS_STEADY_OF_10 = 8              # PROPOSED pass rule: >= 8 of 10 steady at one setting ...
BALLAST_STATION_FRAC_OF_NOSE = 0.75    # default ballast station (fraction of the nose length ahead of the root LE) used in speed tables
BALLAST_STATIONS_N = 4             # ballast table stations: this many of the nose holes, spread from the rearmost to the foremost
PLAIN_EXTRA = True                 # also generate the OPTIONAL plain (no rib) F45 wing, folder F45_plain_span250
PLAIN_EXTRA_AR = 4.5
CL_LIST = (0.3, 0.5)               # lift coefficients for speed / Reynolds
AVL_NCHORD, AVL_NSPAN = 20, 32     # same panel density as the final deck of the earlier study
OUTCOMES = ["steady glide", "dive", "pitch-up or stall", "pitch oscillation", "tumble", "roll-off left", "roll-off right", "yaw wander"]
OUTCOME_DEFS = {"pitch oscillation": "repeated swooping or bounding, nose up and down", "tumble": "flips end over end"}
SOURCES = ("GEOMETRY", "ARITHMETIC", "AVL", "ASSUMED", "UNKNOWN", "JUDGEMENT", "PROPOSED")

ROOT = Path(__file__).resolve().parents[1]
AVL_EXE = ROOT / "tools/avl/avl352.exe"
OUT_DEFAULT = ROOT / "results/runs/rigid_glider/flat_test"


def T(value, source, unit="", note=""):
    """A tagged number (or string): every number written to geometry.json goes through here."""
    assert source in SOURCES, source
    return {"value": value, "unit": unit, "source": source, "note": note}


# =============================================================================================== planform geometry
def wing_geometry(span, ar, taper=TAPER, sweep=SWEEP_QC_DEG):
    S = span * span / ar
    cr = 2 * S / (span * (1 + taper))
    ct = taper * cr
    tipx = span / 2 * math.tan(math.radians(sweep)) + (cr - ct) / 4
    mac = 2 * cr * (1 + taper + taper ** 2) / (3 * (1 + taper))
    mac_y = span * (1 + 2 * taper) / (6 * (1 + taper))
    return dict(span=span, ar=ar, taper=taper, sweep=sweep, area=S, cr=cr, ct=ct, tipx=tipx, mac=mac, mac_y=mac_y,
                mac_le_x=tipx * mac_y / (span / 2), mean_chord=S / span)


def plate_polygon(g, t, tabs=True, tab_len=None, tab_out=None):
    """Closed outline (x aft from root LE, y to the right), counter-clockwise.  Tabs project out of each tip edge by tab_out."""
    b2, tipx, ct = g["span"] / 2, g["tipx"], g["ct"]
    tab_len = tab_len if tab_len is not None else TAB_LEN_FRAC_OF_TIP_CHORD * ct
    tab_out = tab_out if tab_out is not None else (FIN_THICKNESS_MM or t)
    right = [(0.0, 0.0), (tipx, b2)]
    if tabs:
        for p in TAB_POS_FRACS:
            xc = tipx + p * ct
            right += [(xc - tab_len / 2, b2), (xc - tab_len / 2, b2 + tab_out), (xc + tab_len / 2, b2 + tab_out), (xc + tab_len / 2, b2)]
    right += [(tipx + ct, b2), (g["cr"], 0.0)]
    left = [(x, -y) for x, y in reversed(right[1:-1])]            # mirror, traversed back toward the root leading edge
    poly = right[:-1] + [right[-1]] + left
    return orient_ccw(poly)


def area_signed(poly):
    return 0.5 * sum(poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1] for i in range(len(poly)))


def orient_ccw(poly):
    return poly if area_signed(poly) > 0 else list(reversed(poly))


def polygon_centroid(poly):
    a = area_signed(poly)
    cx = sum((poly[i][0] + poly[(i + 1) % len(poly)][0]) * (poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1])
             for i in range(len(poly))) / (6 * a)
    cy = sum((poly[i][1] + poly[(i + 1) % len(poly)][1]) * (poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1])
             for i in range(len(poly))) / (6 * a)
    return cx, cy


# =============================================================================================== meshes
def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def triangulate(poly):
    """Ear clipping for a simple counter-clockwise polygon; returns index triples."""
    idx = list(range(len(poly)))
    tris = []
    while len(idx) > 3:
        n = len(idx)
        for k in range(n):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % n]
            a, b, c = poly[i0], poly[i1], poly[i2]
            if _cross(a, b, c) <= 1e-12:
                continue
            ok = True
            for j in idx:
                if j in (i0, i1, i2):
                    continue
                p = poly[j]
                if _cross(a, b, p) >= -1e-12 and _cross(b, c, p) >= -1e-12 and _cross(c, a, p) >= -1e-12:
                    ok = False
                    break
            if ok:
                tris.append((i0, i1, i2))
                del idx[k]
                break
        else:
            raise ValueError("ear clipping failed")
    tris.append(tuple(idx))
    return tris


def prism(poly, z0, z1):
    """Triangles of a closed solid: polygon extruded from z0 to z1 (outward-facing, counter-clockwise polygon)."""
    out = []
    for i, j, k in triangulate(poly):
        out.append(((*poly[i], z1), (*poly[j], z1), (*poly[k], z1)))
        out.append(((*poly[k], z0), (*poly[j], z0), (*poly[i], z0)))
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        out += _wall(a, b, z0, z1)
    return out


def _wall(a, b, z0, z1):
    return [((*a, z0), (*b, z0), (*b, z1)), ((*a, z0), (*b, z1), (*a, z1))]


def grid_solid(xs, ys, keep, z0, z1):
    """Rectilinear solid on a regular grid (watertight by construction).  keep(i, j) -> bool for the cell [xs[i],xs[i+1]] x [ys[j],ys[j+1]]."""
    out = []
    kept = {(i, j) for i in range(len(xs) - 1) for j in range(len(ys) - 1) if keep(i, j)}
    for (i, j) in sorted(kept):
        x0, x1, y0, y1 = xs[i], xs[i + 1], ys[j], ys[j + 1]
        A, B, C, D = (x0, y0), (x1, y0), (x1, y1), (x0, y1)
        out.append(((*A, z1), (*B, z1), (*C, z1)))
        out.append(((*A, z1), (*C, z1), (*D, z1)))
        out.append(((*C, z0), (*B, z0), (*A, z0)))
        out.append(((*D, z0), (*C, z0), (*A, z0)))
        for (p, q), nb in (((A, B), (i, j - 1)), ((B, C), (i + 1, j)), ((C, D), (i, j + 1)), ((D, A), (i - 1, j))):
            if nb not in kept:
                out += _wall(p, q, z0, z1)
    return out


def grid_boundary(xs, ys, keep):
    """Merged boundary segments ((x0,y0),(x1,y1)) of a grid region (for drawings and DXF)."""
    kept = {(i, j) for i in range(len(xs) - 1) for j in range(len(ys) - 1) if keep(i, j)}
    horiz, vert = {}, {}
    for (i, j) in kept:
        x0, x1, y0, y1 = xs[i], xs[i + 1], ys[j], ys[j + 1]
        if (i, j - 1) not in kept:
            horiz.setdefault(y0, []).append((x0, x1))
        if (i, j + 1) not in kept:
            horiz.setdefault(y1, []).append((x0, x1))
        if (i - 1, j) not in kept:
            vert.setdefault(x0, []).append((y0, y1))
        if (i + 1, j) not in kept:
            vert.setdefault(x1, []).append((y0, y1))

    def merge(iv):
        iv = sorted(iv)
        res = [list(iv[0])]
        for a, b in iv[1:]:
            if abs(a - res[-1][1]) < 1e-9:
                res[-1][1] = b
            else:
                res.append([a, b])
        return res
    segs = []
    for y, iv in horiz.items():
        segs += [((a, y), (b, y)) for a, b in merge(iv)]
    for x, iv in vert.items():
        segs += [((x, a), (x, b)) for a, b in merge(iv)]
    return segs


def mesh_stats(tris):
    """volume, centroid (x, y, z), edge-sharing check."""
    vol = cx = cy = cz = 0.0
    for a, b, c in tris:
        v = (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6
        vol += v
        cx += v * (a[0] + b[0] + c[0]) / 4
        cy += v * (a[1] + b[1] + c[1]) / 4
        cz += v * (a[2] + b[2] + c[2]) / 4
    return dict(volume=vol, centroid=(cx / vol, cy / vol, cz / vol) if vol else None)


def edge_report(tris, nd=6):
    """-> (n_edges, edges_not_shared_by_exactly_two, edges_with_same_direction_twice).  Consistent winding => each directed edge once, reverse once."""
    key = lambda p: tuple(round(v, nd) for v in p)
    directed = {}
    for t in tris:
        k = [key(p) for p in t]
        for i in range(3):
            e = (k[i], k[(i + 1) % 3])
            directed[e] = directed.get(e, 0) + 1
    undirected = {}
    for (a, b), n in directed.items():
        undirected[frozenset((a, b))] = undirected.get(frozenset((a, b)), 0) + n
    bad = sum(1 for n in undirected.values() if n != 2)
    same = sum(1 for n in directed.values() if n > 1)
    degenerate = sum(1 for t in tris if len({key(p) for p in t}) < 3)
    return len(undirected), bad, same, degenerate


def write_stl(path: Path, tris, name="part"):
    with path.open("wb") as f:
        f.write(name.encode("ascii")[:80].ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for a, b, c in tris:
            u = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
            v = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
            n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
            m = math.sqrt(sum(x * x for x in n)) or 1.0
            f.write(struct.pack("<12fH", n[0] / m, n[1] / m, n[2] / m, *a, *b, *c, 0))


def read_stl(path: Path):
    data = path.read_bytes()
    n = struct.unpack("<I", data[80:84])[0]
    tris = []
    for i in range(n):
        rec = struct.unpack("<12fH", data[84 + 50 * i:84 + 50 * (i + 1)])
        tris.append((tuple(rec[3:6]), tuple(rec[6:9]), tuple(rec[9:12])))
    return tris


def bbox(tris):
    pts = [p for t in tris for p in t]
    lo = tuple(min(p[i] for p in pts) for i in range(3))
    hi = tuple(max(p[i] for p in pts) for i in range(3))
    return lo, hi


# =============================================================================================== parts
def fin_dims(span, t):
    return dict(height=FIN_HEIGHT_MM_AT_400 * span / 400.0, thickness=FIN_THICKNESS_MM or t)


def tab_dims(g, t):
    ct = g["ct"]
    L = TAB_LEN_FRAC_OF_TIP_CHORD * ct
    centres = [p * ct for p in TAB_POS_FRACS]                       # from the tip leading edge
    return dict(length=L, thickness_z=t, out=FIN_THICKNESS_MM or t, centres=centres)


def slot_dims(g, t):
    td = tab_dims(g, t)
    return dict(length=td["length"] + SLOT_LEN_CLEARANCE_MM, width=t + SLOT_CLEARANCE_MM, centres=td["centres"])


def rib_geometry(g, t):
    """Spar rib: one parallelogram strip per half wing along the quarter-chord line (a chevron), starting RIB_CENTRE_GAP/2 off the centreline
    (so the stick glues flat to the plate) and stopping RIB_TIP_STOP_MM short of the tip edge.  Printed on top of the plate; FLOWN RIB-DOWN."""
    w, H = RIB_WIDTH_MM, RIB_HEIGHT_MM
    gap = STICK_WIDTH_MM + RIB_CENTRE_GAP_EXTRA_MM
    b2 = g["span"] / 2
    y0, y1 = gap / 2, b2 - RIB_TIP_STOP_MM
    xq = lambda y: g["cr"] / 4 + (g["tipx"] + g["ct"] / 4 - g["cr"] / 4) * y / b2
    polys = [orient_ccw([(xq(y0) - w / 2, s * y0), (xq(y1) - w / 2, s * y1), (xq(y1) + w / 2, s * y1), (xq(y0) + w / 2, s * y0)]) for s in (1, -1)]
    area_each = w * (y1 - y0)
    return dict(polys=polys, width=w, height=H, gap=gap, y0=y0, y1=y1, area_each=area_each, volume=2 * area_each * H, x_centroid=xq((y0 + y1) / 2))


def wing_mesh(g, t, rib=True):
    """Plate (z 0..t) with tabs, plus the rib as two separate closed shells (z t-0.1 .. t+height; they overlap the plate by 0.1 mm so a slicer joins them)."""
    poly = plate_polygon(g, t)
    tris = prism(poly, 0.0, t)
    if rib:
        R = rib_geometry(g, t)
        for p in R["polys"]:
            tris += prism(p, t - RIB_OVERLAP_MM, t + R["height"])
    return tris, poly


def second_moment_T(c, t, w, H):
    """T-section second moment of area (mm4) about its own neutral axis: plate c x t with a rib w x H on top.  -> (I, neutral axis height above the plate bottom)."""
    A1, y1, A2, y2 = c * t, t / 2, w * H, t + H / 2
    yn = (A1 * y1 + A2 * y2) / (A1 + A2)
    return c * t ** 3 / 12 + A1 * (y1 - yn) ** 2 + w * H ** 3 / 12 + A2 * (y2 - yn) ** 2, yn


def fin_mesh(g, t):
    fd, sd = fin_dims(g["span"], t), slot_dims(g, t)
    ct, h = g["ct"], fd["height"]
    xs = sorted({0.0, ct} | {c + s * sd["length"] / 2 for c in sd["centres"] for s in (-1, 1)})
    ym = h / 2
    ys = [0.0, ym - sd["width"] / 2, ym + sd["width"] / 2, h]

    def keep(i, j):
        xc = (xs[i] + xs[i + 1]) / 2
        if j == 1:
            for c in sd["centres"]:
                if abs(xc - c) < sd["length"] / 2:
                    return False
        return True
    return grid_solid(xs, ys, keep, 0.0, fd["thickness"]), (xs, ys, keep)


def stick_layout(g):
    """Rectilinear side profile (x aft from root LE, v up from the saddle plane; saddle top at v = 0)."""
    nose = NOSE_FRAC_OF_ROOT_CHORD * g["cr"]
    x_lo, x_hi, D = -nose, g["cr"], STICK_DEPTH_MM
    notches = []                                                     # (x0, x1, v0, v1) removed regions
    return dict(x_lo=x_lo, x_hi=x_hi, depth=D, nose=nose, notches=notches)


def stick_mesh(g, cg_lo, cg_hi):
    L = stick_layout(g)
    x_lo, x_hi, D = L["x_lo"], L["x_hi"], L["depth"]
    rects = []                                                       # removed rectangles (x0, x1, v0, v1), v measured up from the saddle (negative)
    k_lo = int(math.floor((cg_lo - TICK_MARGIN_MM) / 5.0) * 5)
    k_hi = int(math.ceil((cg_hi + TICK_MARGIN_MM) / 5.0) * 5)
    ticks = []
    for k in range(k_lo, k_hi + 1):
        if not (x_lo + 1.0 < k < x_hi - 1.0):
            continue
        if k % 10 == 0:
            d, w = TICK_DEPTHS_MM[2], 1.2 * LOOP_NOTCH_WIDTH_MM
        elif k % 5 == 0:
            d, w = TICK_DEPTHS_MM[1], LOOP_NOTCH_WIDTH_MM
        else:
            d, w = TICK_DEPTHS_MM[0], TICK_WIDTH_MM
        rects.append((k - w / 2, k + w / 2, -D, -D + d))
        ticks.append(dict(x=k, depth=d, width=w))
    holes = []
    rel_x = x_lo + RELEASE_HOLE_MM / 2 + 1.5
    holes.append(("release", rel_x, -D / 2, RELEASE_HOLE_MM))
    # ballast row: centres at 2.5 mm + 5 mm * k (between the 5 mm notches), from the nose to the end of the saddle
    k = math.ceil((rel_x + RELEASE_HOLE_MM / 2 + 1.5 + BALLAST_HOLE_MM / 2 - 2.5) / BALLAST_PITCH_MM)
    while True:
        xc = 2.5 + BALLAST_PITCH_MM * k
        if xc + BALLAST_HOLE_MM / 2 > x_hi - 1.5:
            break
        holes.append(("ballast", xc, -D / 2, BALLAST_HOLE_MM))
        k += 1
    for kind, xc, vc, s in holes:
        rects.append((xc - s / 2, xc + s / 2, vc - s / 2, vc + s / 2))
    xs = sorted({x_lo, x_hi} | {r[0] for r in rects} | {r[1] for r in rects})
    ys = sorted({-D, 0.0} | {r[2] for r in rects} | {r[3] for r in rects})

    def keep(i, j):
        xc, vc = (xs[i] + xs[i + 1]) / 2, (ys[j] + ys[j + 1]) / 2
        return not any(r[0] < xc < r[1] and r[2] < vc < r[3] for r in rects)
    # print orientation: profile lies flat on the bed: X = x, Y = v + D, Z = 0..STICK_WIDTH
    tris = grid_solid(xs, [v + D for v in ys], keep, 0.0, STICK_WIDTH_MM)
    return tris, dict(layout=L, ticks=ticks, holes=holes, grid=(xs, ys, keep))


# =============================================================================================== AVL (outline only)
def avl_deck(g, with_fins, fin_h, name):
    m = lambda mm: mm / 1000.0
    s = (f"{name}\n0.0\n0 0 0.0\n{g['area'] * 1e-6:.9f} {m(g['mac']):.9f} {m(g['span']):.9f}\n{m(g['mac_le_x']):.9f} 0.0 0.0\n"
         f"SURFACE\nWing\n{AVL_NCHORD} 1.0 {AVL_NSPAN} -2.0\nYDUPLICATE\n0.0\n"
         f"SECTION\n0.0 0.0 0.0 {m(g['cr']):.9f} 0.0\nNACA\n0000\n"
         f"SECTION\n{m(g['tipx']):.9f} {m(g['span'] / 2):.9f} 0.0 {m(g['ct']):.9f} 0.0\nNACA\n0000\n")
    if with_fins:
        s += (f"SURFACE\nTipFin\n5 1.0 8 -2.0\nYDUPLICATE\n0.0\n"
              f"SECTION\n{m(g['tipx']):.9f} {m(g['span'] / 2):.9f} {m(fin_h / 2):.9f} {m(g['ct']):.9f} 0.0\nNACA\n0000\n"
              f"SECTION\n{m(g['tipx']):.9f} {m(g['span'] / 2):.9f} {m(-fin_h / 2):.9f} {m(g['ct']):.9f} 0.0\nNACA\n0000\n")
    return s


def run_avl_case(g, with_fins, fin_h, workdir: Path):
    """-> dict(xnp_mm, cn_beta_cl03, cl_beta_cl03, cl_at, alpha_cl03_deg, cla_per_rad).  AVL knows only the outline (flat camber line, no thickness)."""
    from avl import run_avl
    from geometry import Wing
    w = Wing(span_m=g["span"] / 1000, aspect_ratio=g["ar"], taper=g["taper"], quarter_chord_sweep_deg=g["sweep"], washout_deg=0.0,
             dihedral_deg=0.0, section="NACA_0000", cg_x_m=g["mac_le_x"] / 1000)
    name = "fins" if with_fins else "bare"
    deck = avl_deck(g, with_fins, fin_h, name)
    pts = run_avl(w, name, [0.0, 3.0, 6.0], workdir / name / "scan", AVL_EXE, deck_text=deck)
    cla_deg = (pts[2].cl - pts[0].cl) / 6.0
    a03 = 0.3 / cla_deg
    p = run_avl(w, name, [a03], workdir / name / "cl03", AVL_EXE, deck_text=deck)[0]
    return dict(xnp_mm=1000 * sum(q.xnp_m for q in pts + [p]) / 4, xnp_spread_mm=1000 * (max(q.xnp_m for q in pts + [p]) - min(q.xnp_m for q in pts + [p])),
                cn_beta=p.cnb_per_rad, cl_beta=p.clb_per_rad, cl_at=p.cl, alpha_deg=a03, cla_per_rad=cla_deg * 180 / math.pi)


# =============================================================================================== arithmetic
def mass_g(volume_mm3):
    return volume_mm3 * 1e-3 * PLA_DENSITY_G_CM3                 # mm3 -> cm3


def bend_mm(W_n, span_mm, mean_chord_mm, t_mm, rib=None):
    """ROUGH / ASSUMED: each half wing a uniform cantilever carrying half the weight spread uniformly (ignores sweep, taper, inertia relief).
    rib = (width, height) mm: use the T-section (plate + rib) second moment of area at the mean chord; None = plate only."""
    L = span_mm / 1000
    I = (second_moment_T(mean_chord_mm, t_mm, rib[0], rib[1])[0] if rib else mean_chord_mm * t_mm ** 3 / 12) * 1e-12     # mm4 -> m4
    q = (W_n / 2) / (L / 2)
    return 1000 * q * (L / 2) ** 4 / (8 * E_PLA_GPA * 1e9 * I)


def speed(W_n, area_mm2, cl):
    return math.sqrt(2 * W_n / (RHO_AIR * area_mm2 * 1e-6 * cl))


def ballast_g(M_g, cg0, target, station):
    """Mass at `station` that moves an assembly (mass M_g, CG cg0) to `target`; negative = target is aft of cg0 and nose ballast cannot get there."""
    return M_g * (target - cg0) / (station - target)


def sweep_settings(np_mm, mac_mm):
    start = float(math.ceil(np_mm - CG_START_FRAC_MAC_AHEAD_OF_NP * mac_mm))
    end = float(math.ceil(np_mm + CG_END_FRAC_MAC_BEHIND_NP * mac_mm))      # at least this far behind the neutral point
    out = []
    x = start
    while x <= end + 1e-9:
        out.append(x)
        x += CG_STEP_MM
    return out


# =============================================================================================== one wing
def wing_name(ar):
    return f"F{int(round(ar * 10))}"


def specimen_name(ar, foam=False):
    return ("B" if foam else "F") + f"{int(round(ar * 10))}"


def build_wing(span, ar, thickness, outdir: Path, rib=True, avl=True, foam=False, optional=False):
    """One specimen.  foam=False: printed plate (with a spar rib unless rib=False), STL parts.  foam=True: foamboard wing, cutting templates + printed stick."""
    g = wing_geometry(span, ar)
    name = specimen_name(ar, foam)
    kind = "foam" if foam else "printed"
    t = thickness
    fd = fin_dims(span, t)
    plain = (not foam) and (not rib)
    base = name
    name = base + ("-plain" if plain else "")
    folder = outdir / f"{base}{'_plain' if plain else ''}_span{int(round(span))}"
    folder.mkdir(parents=True, exist_ok=True)
    fin_area = g["ct"] * fd["height"]
    fin_pct = 100 * fin_area / g["area"]
    R = rib_geometry(g, t) if (not foam and rib) else None
    td = sd = None
    if not foam:
        td, sd = tab_dims(g, t), slot_dims(g, t)

    # ---- AVL
    A = None
    if avl:
        wd = outdir / "avl_work" / folder.name
        A = dict(bare=run_avl_case(g, False, fd["height"], wd), fins=run_avl_case(g, True, fd["height"], wd))
        np_ref = A["fins"]["xnp_mm"]
        settings = sweep_settings(np_ref, g["mac"])
    else:
        np_ref, settings = None, []
    cg_lo = settings[0] if settings else g["mac_le_x"] + 0.1 * g["mac"]
    cg_hi = settings[-1] if settings else g["mac_le_x"] + 0.4 * g["mac"]

    # ---- meshes + STL (printed) / templates (foam)
    stris, sinfo = stick_mesh(g, cg_lo, cg_hi)
    write_stl(folder / "stick.stl", stris, f"{name} stick")
    sst, sbb = mesh_stats(stris), bbox(stris)
    size = lambda bb: [bb[1][i] - bb[0][i] for i in range(3)]
    if foam:
        wtris = ftris = None
        wst = fst = wbb = fbb = None
        wpoly, fgrid = plate_polygon(g, t, tabs=False), None
        (folder / "template_half_wing_letter.svg").write_text(template_half_wing_svg(g, name, t, settings, np_ref, A), encoding="utf-8")
        (folder / "template_fins_letter.svg").write_text(template_fins_svg(g, fd, name), encoding="utf-8")
    else:
        wtris, wpoly = wing_mesh(g, t, rib)
        ftris, fgrid = fin_mesh(g, t)
        write_stl(folder / "wing.stl", wtris, f"{name} wing")
        write_stl(folder / "fin.stl", ftris, f"{name} fin (print 2)")
        wst, fst = mesh_stats(wtris), mesh_stats(ftris)
        wbb, fbb = bbox(wtris), bbox(ftris)

    # ---- masses and CG (x aft of root LE, mm)
    plate_area = abs(area_signed(plate_polygon(g, t, tabs=False)))
    xc_plate, _ = polygon_centroid(plate_polygon(g, t, tabs=False))
    fin_x_cg = g["tipx"] + g["ct"] / 2
    stick_x_cg = sst["centroid"][0]
    nose_holes = sorted(h[1] for h in sinfo["holes"] if h[0] == "ballast" and h[1] < 0)
    default_station = min(nose_holes, key=lambda x: abs(x + BALLAST_STATION_FRAC_OF_NOSE * sinfo["layout"]["nose"]))
    n_st = min(BALLAST_STATIONS_N, len(nose_holes))
    table_stations = sorted({nose_holes[round(i * (len(nose_holes) - 1) / max(n_st - 1, 1))] for i in range(n_st)})
    tts = [t] if foam else list(THICKNESS_TABLE_MM)
    thick = {}
    for tt in tts:
        if foam:
            m_plate = g["area"] * 1e-6 * FOAM_AREAL_MASS_G_M2
            m_fins = 2 * fin_area * 1e-6 * FOAM_AREAL_MASS_G_M2
            m_rib = 0.0
        else:
            m_plate = mass_g(plate_area * tt)
            m_fins = 2 * mass_g(fin_area * (FIN_THICKNESS_MM or tt))
            m_rib = mass_g(R["volume"]) if R else 0.0
        m_wing = m_plate + m_rib
        x_wing = xc_plate if not m_rib else (m_plate * xc_plate + m_rib * R["x_centroid"]) / m_wing
        m_stick = mass_g(sst["volume"])
        M0 = m_wing + m_fins + m_stick + GLUE_MASS_G
        cg0 = (m_wing * x_wing + m_fins * fin_x_cg + m_stick * stick_x_cg) / (m_wing + m_fins + m_stick)
        mid = settings[len(settings) // 2] if settings else None
        mb = max(0.0, ballast_g(M0, cg0, mid, default_station)) if mid is not None else None
        Mfly = M0 + (mb or 0.0)
        W = Mfly * 1e-3 * 9.80665
        if foam:
            bend_plain = bend_rib = I_plate = I_rib = None
        else:
            bend_plain = bend_mm(W, span, g["mean_chord"], tt)
            bend_rib = bend_mm(W, span, g["mean_chord"], tt, (R["width"], R["height"])) if R else bend_plain
            I_plate = g["mean_chord"] * tt ** 3 / 12
            I_rib = second_moment_T(g["mean_chord"], tt, R["width"], R["height"])[0] if R else I_plate
        row = dict(plate_mass=m_plate, rib_mass=m_rib, wing_mass=m_wing, fins_mass=m_fins, stick_mass=m_stick, allup_noballast=M0, cg_noballast=cg0,
                   ws_noballast=M0 * 1e-3 * 9.80665 / (g["area"] * 1e-6), ballast_mid=mb, allup_ballast=Mfly,
                   ws_ballast=W / (g["area"] * 1e-6), tip_bend=bend_rib, tip_bend_plain=bend_plain, I_plate=I_plate, I_spec=I_rib,
                   wing_only_ws=m_wing * 1e-3 * 9.80665 / (g["area"] * 1e-6),
                   t_pct_mean=100 * tt / g["mean_chord"], t_pct_tip=100 * tt / g["ct"])
        for cl in CL_LIST:
            v = speed(W, g["area"], cl)
            row[f"V_cl{cl}"] = v
            row[f"Re_mean_cl{cl}"] = v * g["mean_chord"] * 1e-3 / NU_AIR
            row[f"Re_tip_cl{cl}"] = v * g["ct"] * 1e-3 / NU_AIR
            row[f"Re_root_cl{cl}"] = v * g["cr"] * 1e-3 / NU_AIR
        thick[tt] = row

    ballast_rows = []
    nose = sinfo["layout"]["nose"]
    for tt in tts:
        r = thick[tt]
        for cg in settings:
            for stn in table_stations:
                ballast_rows.append(dict(wing=name, thickness_mm=tt, cg_mm_aft_of_root_LE=cg, cg_mm_ahead_of_NP=round(np_ref - cg, 2),
                                         ballast_station_mm=round(stn, 1), ballast_g=ballast_g(r["allup_noballast"], r["cg_noballast"], cg, stn)))
    stick_len = sinfo["layout"]["x_hi"] - sinfo["layout"]["x_lo"]

    # ---- geometry.json
    mat_note = "ASSUMED: Kai to measure" if foam else "plate thickness chosen for the printed part"
    G = {
        "wing": name, "specimen_kind": kind, "source_legend": {s: s for s in SOURCES},
        "span_mm": T(span, "GEOMETRY", "mm"), "aspect_ratio": T(ar, "GEOMETRY"), "taper": T(g["taper"], "GEOMETRY"),
        "quarter_chord_sweep_deg": T(g["sweep"], "GEOMETRY", "deg"), "dihedral_deg": T(DIHEDRAL_DEG, "GEOMETRY", "deg", "zero: one-piece flat wing (model used 3 deg per panel)"),
        "thickness_mm": T(t, "ASSUMED" if foam else "GEOMETRY", "mm", mat_note),
        "area_mm2": T(g["area"], "GEOMETRY", "mm2", "span^2 / AR"), "root_chord_mm": T(g["cr"], "GEOMETRY", "mm"), "tip_chord_mm": T(g["ct"], "GEOMETRY", "mm"),
        "mean_chord_mm": T(g["mean_chord"], "GEOMETRY", "mm", "area / span"), "mac_mm": T(g["mac"], "GEOMETRY", "mm"),
        "mac_le_x_mm": T(g["mac_le_x"], "GEOMETRY", "mm", "aft of root leading edge"), "mac_y_mm": T(g["mac_y"], "GEOMETRY", "mm", "from centreline"),
        "tip_le_x_mm": T(g["tipx"], "GEOMETRY", "mm", "aft of root leading edge"), "tip_te_x_mm": T(g["tipx"] + g["ct"], "GEOMETRY", "mm"),
        "root_te_x_mm": T(g["cr"], "GEOMETRY", "mm"),
        "thickness_pct_mean_chord": T(100 * t / g["mean_chord"], "ASSUMED" if foam else "GEOMETRY", "%", "uses the ASSUMED thickness" if foam else ""),
        "thickness_pct_tip_chord": T(100 * t / g["ct"], "ASSUMED" if foam else "GEOMETRY", "%", "uses the ASSUMED thickness" if foam else ""),
        "fin_height_mm": T(fd["height"], "GEOMETRY", "mm", "24 mm x span/400"), "fin_chord_mm": T(g["ct"], "GEOMETRY", "mm", "= tip chord"),
        "fin_thickness_mm": T(fd["thickness"], "ASSUMED" if foam else "GEOMETRY", "mm"), "fin_area_each_mm2": T(fin_area, "GEOMETRY", "mm2"),
        "fin_area_each_pct_of_wing": T(fin_pct, "GEOMETRY", "%"),
        "wing_flat_plate_centroid_x_mm": T(xc_plate, "ARITHMETIC", "mm", "area centroid of the outline (uniform plate), aft of root LE"),
        "stick_volume_mm3": T(sst["volume"], "GEOMETRY", "mm3"),
        "print_bbox_stick_mm": T(size(sbb), "GEOMETRY", "mm", "lying on its side"),
        "stick_length_mm": T(stick_len, "GEOMETRY", "mm"),
        "stick_longer_than_180mm": T(stick_len > STICK_LENGTH_WARN_MM, "GEOMETRY", "", "flag: the stick may not fit a small print bed"),
        "stick_nose_ahead_of_root_le_mm": T(nose, "GEOMETRY", "mm", "40 % of root chord"),
        "stick_depth_mm": T(STICK_DEPTH_MM, "GEOMETRY", "mm"), "stick_width_mm": T(STICK_WIDTH_MM, "GEOMETRY", "mm"),
        "stick_tick_range_mm": T([sinfo["ticks"][0]["x"], sinfo["ticks"][-1]["x"]], "GEOMETRY", "mm", "aft of root LE"),
        "stick_centroid_x_mm": T(stick_x_cg, "ARITHMETIC", "mm", "volume centroid of the stick mesh"),
        "ballast_holes_x_mm": T([h[1] for h in sinfo["holes"] if h[0] == "ballast"], "GEOMETRY", "mm"),
        "ballast_hole_size_mm": T(BALLAST_HOLE_MM, "GEOMETRY", "mm", "square; M3 clearance"),
        "ballast_hole_pitch_mm": T(BALLAST_PITCH_MM, "GEOMETRY", "mm"),
        "drop_loop_notches_x_mm": T([tk["x"] for tk in sinfo["ticks"] if tk["x"] % 5 == 0], "GEOMETRY", "mm", "5 mm marks: wider notches that seat the drone-drop thread loop"),
        "release_hole_x_mm": T([h[1] for h in sinfo["holes"] if h[0] == "release"][0], "GEOMETRY", "mm"),
    }
    if foam:
        G["material_thickness_mm"] = T(t, "ASSUMED", "mm", "foamboard thickness: Kai to measure; measured value goes in the page table")
        G["material_areal_mass_g_m2"] = T(FOAM_AREAL_MASS_G_M2, "ASSUMED", "g/m2", "typical light craft foamboard with paper; to be weighed")
        G["thickness_above_published_range"] = T(100 * t / g["mean_chord"] > 4.3, "ASSUMED", "", "published plate tests used 2.6 to 4.3 % thick plates")
    else:
        G.update({
            "tab_length_mm": T(td["length"], "GEOMETRY", "mm"), "tab_centres_from_tip_LE_mm": T(td["centres"], "GEOMETRY", "mm"),
            "tab_projection_mm": T(td["out"], "GEOMETRY", "mm"), "slot_length_mm": T(sd["length"], "GEOMETRY", "mm"),
            "slot_width_mm": T(sd["width"], "GEOMETRY", "mm", "plate thickness + clearance"), "slot_clearance_mm": T(SLOT_CLEARANCE_MM, "GEOMETRY", "mm"),
            "print_bbox_wing_mm": T(size(wbb), "GEOMETRY", "mm", "x, y, z (z = plate thickness plus rib height, if any; includes tabs)"),
            "print_bbox_fin_mm": T(size(fbb), "GEOMETRY", "mm"),
            "wing_volume_mm3": T(wst["volume"], "GEOMETRY", "mm3", "from the STL (rib shells overlap the plate by 0.1 mm)"), "fin_volume_mm3": T(fst["volume"], "GEOMETRY", "mm3"),
            "rib_included": T(bool(R), "GEOMETRY", "", "printed on top, FLOWN RIB-DOWN (rib and stick on the underside)"),
        })
        if R:
            G.update({"rib_width_mm": T(R["width"], "GEOMETRY", "mm"), "rib_height_mm": T(R["height"], "GEOMETRY", "mm", "above the plate"),
                      "rib_tip_stop_mm": T(RIB_TIP_STOP_MM, "GEOMETRY", "mm", "stops this far short of the tip edge (and the tip tabs)"),
                      "rib_centre_gap_mm": T(R["gap"], "GEOMETRY", "mm", "stick width + 0.4 mm, so the stick glues flat to the plate"),
                      "rib_volume_mm3": T(R["volume"], "GEOMETRY", "mm3", "union volume, both halves"), "rib_x_centroid_mm": T(R["x_centroid"], "ARITHMETIC", "mm")})
    G["thickness_table"] = {}
    for tt, r in thick.items():
        e = {
            "wing_mass_g": T(r["wing_mass"], "ARITHMETIC", "g", ("area x areal mass (both ASSUMED)" if foam else f"plate {tt} mm + rib; PLA {PLA_DENSITY_G_CM3} g/cm3 (density ASSUMED, 100% solid)")),
            "plate_mass_g": T(r["plate_mass"], "ARITHMETIC", "g", "plate only"), "rib_mass_g": T(r["rib_mass"], "ARITHMETIC", "g", "rib only (0 if none)"),
            "fins_mass_g": T(r["fins_mass"], "ARITHMETIC", "g", "2 fins"), "stick_mass_g": T(r["stick_mass"], "ARITHMETIC", "g", "stick mesh volume x PLA density"),
            "allup_no_ballast_g": T(r["allup_noballast"], "ARITHMETIC", "g", "wing + 2 fins + stick; glue/tape not included"),
            "cg_no_ballast_mm": T(r["cg_noballast"], "ARITHMETIC", "mm", "mass-weighted centroids of wing (plate + rib), fins, stick; aft of root LE"),
            "wing_only_loading_n_m2": T(r["wing_only_ws"], "ARITHMETIC", "N/m2", "wing weight / wing area"),
            "loading_no_ballast_n_m2": T(r["ws_noballast"], "ARITHMETIC", "N/m2"),
            "ballast_for_middle_setting_g": T(r["ballast_mid"], "ARITHMETIC", "g", f"at the nose hole at {default_station:.1f} mm; 0 if none needed"),
            "allup_with_ballast_g": T(r["allup_ballast"], "ARITHMETIC", "g"), "loading_with_ballast_n_m2": T(r["ws_ballast"], "ARITHMETIC", "N/m2"),
            "thickness_pct_mean_chord": T(r["t_pct_mean"], "ASSUMED" if foam else "GEOMETRY", "%"), "thickness_pct_tip_chord": T(r["t_pct_tip"], "ASSUMED" if foam else "GEOMETRY", "%"),
            **{f"speed_cl{cl}_m_s": T(r[f"V_cl{cl}"], "ARITHMETIC", "m/s", f"with-ballast mass, rho {RHO_AIR} (ASSUMED)") for cl in CL_LIST},
            **{f"re_mean_chord_cl{cl}": T(r[f"Re_mean_cl{cl}"], "ARITHMETIC", "", f"nu {NU_AIR} (ASSUMED)") for cl in CL_LIST},
            **{f"re_tip_chord_cl{cl}": T(r[f"Re_tip_cl{cl}"], "ARITHMETIC", "") for cl in CL_LIST},
            **{f"re_root_chord_cl{cl}": T(r[f"Re_root_cl{cl}"], "ARITHMETIC", "") for cl in CL_LIST},
        }
        if foam:
            e["tip_bend_1g_mm"] = T(None, "UNKNOWN", "mm", "not estimated: foamboard stiffness is unknown; flex the finished wing and look")
        else:
            note = (f"ROUGH/ASSUMED: uniform cantilever, E {E_PLA_GPA} GPa (ASSUMED), mean chord, no inertia relief, no sweep")
            e["tip_bend_1g_mm"] = T(r["tip_bend"], "ARITHMETIC", "mm", note + ("; T-section plate + rib" if R else "; plain plate"))
            e["tip_bend_plate_only_1g_mm"] = T(r["tip_bend_plain"], "ARITHMETIC", "mm", note + "; plate only")
            e["second_moment_plate_only_mm4"] = T(r["I_plate"], "ARITHMETIC", "mm4", "mean chord x t^3 / 12")
            e["second_moment_specimen_mm4"] = T(r["I_spec"], "ARITHMETIC", "mm4", "T-section (plate + rib) about its neutral axis; plain = plate only")
        G["thickness_table"][f"{tt:g}"] = e
    G["cg_sweep"] = {"settings_mm_aft_of_root_le": T(settings, "JUDGEMENT", "mm", "start 15% MAC ahead of the AVL neutral point (with fins), step 1 mm, aft to at least 8% MAC behind it; then the stop rule (flat_test page, section 6)"),
                     "start_frac_mac_ahead": T(CG_START_FRAC_MAC_AHEAD_OF_NP, "JUDGEMENT"), "step_mm": T(CG_STEP_MM, "JUDGEMENT", "mm")}
    if A:
        for key, nm in (("bare", "bare"), ("fins", "with_tip_fins")):
            a = A[key]
            G[f"avl_{nm}"] = {"neutral_point_mm": T(a["xnp_mm"], "AVL", "mm", "aft of root LE; outline only (flat camber line, no thickness, no section effects)"),
                              "neutral_point_spread_mm": T(a["xnp_spread_mm"], "AVL", "mm", "max-min over the 4 AVL runs"),
                              "cn_beta_per_rad_at_cl0.3": T(a["cn_beta"], "AVL", "1/rad", f"at CL {a['cl_at']:.3f}"),
                              "cl_beta_per_rad_at_cl0.3": T(a["cl_beta"], "AVL", "1/rad"), "alpha_for_cl0.3_deg": T(a["alpha_deg"], "AVL", "deg"),
                              "lift_slope_per_rad": T(a["cla_per_rad"], "AVL", "1/rad")}
        G["cg_sweep"]["neutral_point_ahead_of_start_mm"] = T(np_ref - settings[0], "AVL", "mm")
        G["cg_sweep"]["end_frac_mac_behind_np"] = T(CG_END_FRAC_MAC_BEHIND_NP, "JUDGEMENT")
        G["cg_sweep"]["settings_behind_neutral_point_mm"] = T([x for x in settings if x > np_ref], "AVL", "mm", "settings behind the AVL neutral point (with fins)")
    if t in thick:
        G["cg_sweep"]["bare_aircraft_balance_mm"] = T(thick[t]["cg_noballast"], "ARITHMETIC", "mm", "wing + fins + stick, no ballast; masses ASSUMED")
        G["cg_sweep"]["mac_mm_for_start"] = T(g["mac"], "GEOMETRY", "mm")
    (folder / "geometry.json").write_text(json.dumps(G, indent=1))

    # ---- DXF + SVG
    write_dxf(folder / "parts.dxf", g, t, wpoly, fgrid, fd, sd, sinfo, (cg_lo, cg_hi), np_ref, R)
    (folder / "drawing.svg").write_text(dark_svg(drawing_svg(g, t, wpoly, fd, sinfo, A, settings, xc_plate, None, name, R, foam)), encoding="utf-8")
    return dict(name=name, kind=kind, optional=optional, bare_balance=thick[t]["cg_noballast"] if t in thick else None, folder=folder, g=g, t=t, fd=fd, td=td, sd=sd, A=A, settings=settings, thick=thick, ballast_rows=ballast_rows, rib=R, plain=plain,
                default_station=default_station, table_stations=table_stations,
                sizes=dict(wing=size(wbb) if wbb else None, fin=size(fbb) if fbb else None, stick=size(sbb)), stick=sinfo,
                stats=dict(wing=wst, fin=fst, stick=sst), fin_pct=fin_pct, stick_len=stick_len,
                xc_plate=xc_plate, geometry=G, np_ref=np_ref, plate_area=plate_area)


# =============================================================================================== foamboard cutting templates (1:1, US Letter portrait, mm)
LETTER_W, LETTER_H = 215.9, 279.4
TEMPLATE_MARGIN = 10.0
SCALE_BAR_MM = 100.0


def _t(x, y, s, size=3.4, anchor="start", color="#18222d", weight="normal"):
    return f'<text x="{x:.2f}" y="{y:.2f}" font-family="Segoe UI,Arial" font-size="{size}" text-anchor="{anchor}" fill="{color}" font-weight="{weight}">{html.escape(str(s))}</text>'


def _l(x1, y1, x2, y2, color="#18222d", w=0.35, dash=None, extra=""):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    return f'<line {extra} x1="{x1:.3f}" y1="{y1:.3f}" x2="{x2:.3f}" y2="{y2:.3f}" stroke="{color}" stroke-width="{w}"{d}/>'


def _scale_bar(x, y):
    o = [_l(x, y, x + SCALE_BAR_MM, y, "#000", 0.5, extra='id="scalebar"')]
    for k in range(0, int(SCALE_BAR_MM) + 1, 10):
        o.append(_l(x + k, y - 2.0, x + k, y + 2.0, "#000", 0.35))
    o.append(_t(x, y + 6.0, "Scale bar: this line must measure exactly 100 mm on your print.", 2.8))
    o.append(_t(x, y + 10.0, "If it does not, print at 100 % (actual size), not fit-to-page.", 2.8))
    return o


def template_half_wing_svg(g, name, t, settings, np_ref, A):
    ox, yc, b2 = 22.0, 244.0, g["span"] / 2
    X, Y = (lambda x: ox + x), (lambda y: yc - y)
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{LETTER_W}mm" height="{LETTER_H}mm" viewBox="0 0 {LETTER_W} {LETTER_H}">',
         '<rect width="100%" height="100%" fill="white"/>',
         _t(12, 14, f"{name}: ONE HALF of the wing, 1:1, US Letter portrait", 5, weight="bold"),
         _t(12, 19.5, f"Foamboard {t:g} mm (ASSUMED: measure yours). Span {g['span']:g}, root chord {g['cr']:.1f}, tip chord {g['ct']:.1f}, sweep {g['sweep']:g} deg, zero dihedral.", 2.8),
         _t(12, 23.7, "Cut along the solid outline. Trace it on the board, then flip the template over on the dashed", 2.8),
         _t(12, 27.9, "centreline and trace the other half. The wing is one piece. Print at 100 % (actual size).", 2.8)]
    poly = [(0, 0), (g["tipx"], b2), (g["tipx"] + g["ct"], b2), (g["cr"], 0)]
    if settings:
        o.append(f'<rect x="{X(settings[0]):.2f}" y="{Y(b2) - 4:.2f}" width="{settings[-1] - settings[0]:.2f}" height="{b2 + 8:.2f}" fill="#28834f" fill-opacity="0.18"/>')
        o.append(_t(X(settings[0]), Y(b2) - 6, f"CG test range {settings[0]:g}-{settings[-1]:g} mm aft of root LE (JUDGEMENT)", 2.8, color="#28834f"))
    o.append(f'<polygon points="{" ".join(f"{X(x):.3f},{Y(y):.3f}" for x, y in poly)}" fill="#e6eef6" fill-opacity="0.5" stroke="#000" stroke-width="0.5"/>')
    o.append(_l(X(0) - 6, Y(0), X(g["cr"] + 14), Y(0), "#555", 0.35, "3 2"))
    o.append(_t(X(0) - 6, Y(0) + 4.5, "centreline: flip here for the other half", 2.8, color="#555"))
    if np_ref is not None:
        o.append(_l(X(np_ref), Y(b2) - 4, X(np_ref), Y(0) + 4, "#b0261d", 0.4, "2 1.5"))
        o.append(_t(X(np_ref) + 1.5, Y(b2) + 12, f"AVL neutral point {np_ref:.1f} (AVL, outline only)", 2.8, color="#b0261d"))
    o.append(_l(X(0), Y(0) + 8, X(0), Y(0) - 8, "#000", 0.35))
    o.append(_t(X(0) + 2, Y(0) + 10.5, "root leading edge (x = 0)", 2.8))
    o.append(_t(X(g["tipx"] + g["ct"]) - 2, Y(b2) - 1.5, "tip edge: fin butt-glues here, centred on the board mid-plane", 2.8, "end", "#c2621a"))
    o.extend(_scale_bar(ox, 258.0))
    o.append("</svg>")
    return "\n".join(o)


def template_fins_svg(g, fd, name):
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{LETTER_W}mm" height="{LETTER_H}mm" viewBox="0 0 {LETTER_W} {LETTER_H}">',
         '<rect width="100%" height="100%" fill="white"/>',
         _t(12, 14, f"{name}: TWO tip fins, 1:1, US Letter portrait", 5, weight="bold"),
         _t(12, 19.5, f"Each fin {g['ct']:.1f} x {fd['height']:.1f} mm (chord = tip chord, height 24 mm x span/400): about 4 % of wing area.", 2.8),
         _t(12, 23.7, "Cut two from the same foamboard. Butt-glue each to the wing tip edge, centred on the board", 2.8),
         _t(12, 27.9, "mid-plane (no tabs). Check they stand square. Print at 100 % (actual size).", 2.8)]
    y = 40.0
    for i in range(2):
        o.append(f'<rect x="22" y="{y:.2f}" width="{g["ct"]:.3f}" height="{fd["height"]:.3f}" fill="#fbeedd" stroke="#000" stroke-width="0.5"/>')
        o.append(f'<rect x="22" y="{y + fd["height"] / 2 - FOAM_THICKNESS_MM / 2:.3f}" width="{g["ct"]:.3f}" height="{FOAM_THICKNESS_MM:.3f}" fill="none" stroke="#c2621a" stroke-width="0.25" stroke-dasharray="2 1.5"/>')
        o.append(_t(22 + g["ct"] + 4, y + fd["height"] / 2, f"fin {i + 1}: {g['ct']:.1f} x {fd['height']:.1f} mm", 3.0))
        y += fd["height"] + 15
    o.append(_t(22, y + 2, "Dashed band = where the wing tip edge lands on the fin (board thickness, ASSUMED 5 mm,", 2.8, color="#c2621a"))
    o.append(_t(22, y + 6.2, "centred on the fin height). Re-mark it with your measured thickness.", 2.8, color="#c2621a"))
    o.extend(_scale_bar(22.0, 258.0))
    o.append("</svg>")
    return "\n".join(o)


# =============================================================================================== DXF
def write_dxf(path, g, t, wpoly, fgrid, fd, sd, sinfo, cg_range, np_ref, rib=None):
    ent = []

    def line(layer, p, q, dx=0.0, dy=0.0):
        ent.append(f"0\nLINE\n8\n{layer}\n10\n{p[0] + dx:.4f}\n20\n{p[1] + dy:.4f}\n30\n0.0\n11\n{q[0] + dx:.4f}\n21\n{q[1] + dy:.4f}\n31\n0.0\n")
    n = len(wpoly)
    for i in range(n):
        line("WING_OUTLINE", wpoly[i], wpoly[(i + 1) % n])
    if rib:
        for poly in rib["polys"]:
            for i in range(len(poly)):
                line("RIB_OUTLINE", poly[i], poly[(i + 1) % len(poly)])
    line("REFERENCE", (g["mac_le_x"], -g["span"] / 2 - 6), (g["mac_le_x"] + g["mac"], -g["span"] / 2 - 6))
    for nm, x in (("CG_START", cg_range[0]), ("CG_END", cg_range[1])):
        line("REFERENCE", (x, -g["span"] / 2 - 3), (x, g["span"] / 2 + 3))
    if np_ref is not None:
        line("REFERENCE", (np_ref, -g["span"] / 2 - 3), (np_ref, g["span"] / 2 + 3))
    dy = g["span"] / 2 + 30                                          # fin laid out below the wing (in +y here; x from 0)
    if fgrid is not None:
        xs, ys, keep = fgrid
        for p, q in grid_boundary(xs, ys, lambda i, j: True):
            line("FIN_OUTLINE", p, q, 0.0, dy)
        outline = set(grid_boundary(xs, ys, keep))
        full = set(grid_boundary(xs, ys, lambda i, j: True))
        for p, q in outline - full:
            line("FIN_SLOT", p, q, 0.0, dy)
    else:                                                            # foamboard fin: plain rectangle, no slots
        c, h = g["ct"], fd["height"]
        for p, q in (((0, 0), (c, 0)), ((c, 0), (c, h)), ((c, h), (0, h)), ((0, h), (0, 0))):
            line("FIN_OUTLINE", p, q, 0.0, dy)
    sx, sy, skeep = sinfo["grid"]
    dy2 = dy + fd["height"] + 30
    segs = grid_boundary(sx, [v + sinfo["layout"]["depth"] for v in sy], skeep)
    for p, q in segs:
        line("STICK_OUTLINE", p, q, 0.0, dy2)
    path.write_text("0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n0\nSECTION\n2\nTABLES\n0\nTABLE\n2\nLAYER\n70\n7\n"
                    + "".join(f"0\nLAYER\n2\n{l}\n70\n0\n62\n{c}\n6\nCONTINUOUS\n" for l, c in
                            (("WING_OUTLINE", 7), ("RIB_OUTLINE", 6), ("FIN_OUTLINE", 3), ("FIN_SLOT", 1), ("STICK_OUTLINE", 5), ("REFERENCE", 8), ("0", 7)))
                    + "0\nENDTAB\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n" + "".join(ent) + "0\nENDSEC\n0\nEOF\n", encoding="ascii")


# =============================================================================================== SVG drawing

DARK_SVG_MAP = {"#18222d": "#dde3ea", "#566473": "#9aa7b4", "#e6eef6": "#27313c", "#c2621a": "#e89a55", "#28834f": "#6cc08b", "#b0261d": "#ff8a80",
                "#793da4": "#c39bff", "#555": "#9aa7b4", "#222": "#dde3ea", "#1769aa": "#7fb4ff"}


def dark_svg(svg: str) -> str:
    """On-screen drawings only (drawing.svg): dark background, light lines and labels.  Print templates do NOT go through this."""
    svg = svg.replace('<rect width="100%" height="100%" fill="white"/>', '<rect width="100%" height="100%" fill="#14181d"/>')
    for a, b in DARK_SVG_MAP.items():
        svg = svg.replace(f'"{a}"', f'"{b}"')
    return svg

def _txt(x, y, s, size=11, anchor="start", color="#18222d", rot=None, weight="normal"):
    tr = f' transform="rotate({rot} {x:.1f} {y:.1f})"' if rot is not None else ""
    return f'<text x="{x:.1f}" y="{y:.1f}" font-family="Segoe UI,Arial" font-size="{size}" text-anchor="{anchor}" fill="{color}" font-weight="{weight}"{tr}>{html.escape(str(s))}</text>'


def _ln(x1, y1, x2, y2, color="#18222d", w=1.0, dash=None):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    return f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{color}" stroke-width="{w}"{d}/>'


def drawing_svg(g, t, wpoly, fd, sinfo, A, settings, xc_plate, trow, name, rib=None, foam=False):
    s = 750.0 / g["span"]                                           # px per mm (3.0 at 250 mm span)
    x0 = 70 + (sinfo["layout"]["nose"] + 6) * s                                                 # screen x of x = 0 mm (room for the nose)
    ytop = 100                                                       # screen y of y = -span/2
    sy = lambda y: ytop + (y + g["span"] / 2) * s
    sx = lambda x: x0 + x * s
    W = int(x0 + (g["tipx"] + g["ct"] + 60) * s) + 40
    stick_top = sy(g["span"] / 2) + 190
    H = int(stick_top + 175)
    matdesc = (f"foamboard {t:g} mm (ASSUMED)" if foam else (f"flat plate {t:g} mm + spar rib {RIB_WIDTH_MM:g} x {RIB_HEIGHT_MM:g} mm (flown rib-down)" if rib else f"plain flat plate {t:g} mm"))
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">', '<rect width="100%" height="100%" fill="white"/>',
         _txt(20, 26, f"{name}: span {g['span']:g} mm, AR {g['ar']:g}, taper {g['taper']:g}, sweep {g['sweep']:g} deg, {matdesc}, zero dihedral", 15, weight="bold"),
         _txt(20, 46, "Top view, nose to the left. All dimensions in mm. Source tags: G = GEOMETRY, AVL = outline only (no section effects), J = JUDGEMENT.", 11, color="#566473")]
    pts = " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in wpoly)
    o.append(f'<polygon points="{pts}" fill="#e6eef6" stroke="#18222d" stroke-width="1.5"/>')
    if rib:
        for rp in rib["polys"]:
            o.append('<polygon points="' + " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in rp) + '" fill="#555" fill-opacity="0.45" stroke="#222" stroke-width="0.8" stroke-dasharray="4 2"/>')
        o.append(_txt(20, 64, f"Spar rib (grey, dashed): printed on top of the plate, FLOWN RIB-DOWN. Rib and stick are both on the underside. Fins are centred on the plate mid-plane.", 11, color="#555"))
    elif not foam:
        o.append(_txt(20, 64, "Plain plate: no rib.", 11, color="#555"))
    else:
        o.append(_txt(20, 64, "Foamboard wing: cut from board (templates), no rib; stick glued underneath; fins butt-glued on the board mid-plane.", 11, color="#555"))
    # fins: top view = thin rectangles of chord ct at each tip
    for sg in (1, -1):
        yy = sy(sg * g["span"] / 2)
        o.append(f'<rect x="{sx(g["tipx"]):.1f}" y="{(yy if sg > 0 else yy - max(fd["thickness"] * s, 2.5)):.1f}" width="{g["ct"] * s:.1f}" height="{max(fd["thickness"] * s, 2.5):.1f}" fill="#c2621a"/>')
    o.append(_txt(sx(g["tipx"] + g["ct"]) + 6, sy(-g["span"] / 2) + 4, f"tip fin (side on): chord {g['ct']:.1f}, height {fd['height']:.1f}, centred on wing plane", 10, color="#c2621a"))
    # CG test range band, NP, MAC
    if settings:
        o.append(f'<rect x="{sx(settings[0]):.1f}" y="{ytop - 18}" width="{(settings[-1] - settings[0]) * s:.1f}" height="{g["span"] * s + 36:.1f}" fill="#28834f" fill-opacity="0.15"/>')
        o.append(_txt(sx(settings[0]), ytop - 24, f"CG test range {settings[0]}-{settings[-1]} mm (J)", 11, color="#28834f"))
    if A:
        for key, col, dy in (("fins", "#b0261d", 0), ("bare", "#793da4", 14)):
            x = A[key]["xnp_mm"]
            o.append(_ln(sx(x), ytop - 12, sx(x), ytop + g["span"] * s + 12, col, 1.5, "6 3"))
            o.append(_txt(sx(x) + 4, ytop + g["span"] * s + 12 + dy, f"AVL neutral point {'with fins' if key == 'fins' else 'bare'} {x:.1f} (AVL, outline only)", 10, color=col))
    o.append(_ln(sx(g["mac_le_x"]), sy(g["mac_y"]), sx(g["mac_le_x"] + g["mac"]), sy(g["mac_y"]), "#555", 3))
    o.append(_txt(sx(g["mac_le_x"]), sy(g["mac_y"]) - 6, f"MAC {g['mac']:.1f} (G), LE at {g['mac_le_x']:.1f}", 10, color="#555"))
    o.append(_ln(sx(xc_plate) - 5, sy(0), sx(xc_plate) + 5, sy(0), "#1769aa", 2))
    o.append(_txt(sx(xc_plate), sy(0) - 8, f"flat-wing area centroid {xc_plate:.1f} (ARITHMETIC)", 10, color="#1769aa"))
    # dimensions
    dim = lambda a, b, off, label: o.extend([_ln(*a, *b, "#555", 0.8), _txt((a[0] + b[0]) / 2, (a[1] + b[1]) / 2 - 4, label, 10, "middle", "#18222d")])
    ys_dim = sy(g["span"] / 2) + 44
    o.append(_ln(sx(0) - 60, sy(-g["span"] / 2), sx(0) - 60, sy(g["span"] / 2), "#555", 0.8))
    o.append(_txt(sx(0) - 66, sy(0), f"span {g['span']:g} (G)", 11, "middle", rot=-90))
    o.append(_ln(sx(0), sy(g["span"] / 2) + 62, sx(g["cr"]), sy(g["span"] / 2) + 62, "#555", 0.8))
    o.append(_txt(sx(g["cr"] / 2), sy(g["span"] / 2) + 58, f"root chord {g['cr']:.1f} (G)", 10, "middle"))
    o.append(_ln(sx(g["tipx"]), sy(g["span"] / 2) + 46, sx(g["tipx"] + g["ct"]), sy(g["span"] / 2) + 46, "#555", 0.8))
    o.append(_txt(sx(g["tipx"] + g["ct"] / 2), sy(g["span"] / 2) + 42, f"tip chord {g['ct']:.1f} (G)", 10, "middle"))
    o.append(_ln(sx(0), sy(-g["span"] / 2) - 8, sx(g["tipx"]), sy(-g["span"] / 2) - 8, "#555", 0.8))
    o.append(_txt(sx(g["tipx"] / 2), sy(-g["span"] / 2) - 12, f"tip LE {g['tipx']:.1f} aft of root LE (G)", 10, "middle"))
    o.append(_txt(sx(g["cr"]) + 8, sy(0) + 16, f"root TE x = {g['cr']:.1f}", 10))
    o.append(_txt(sx(0) - 6, sy(0) + 4, "root LE = origin", 10, "end"))
    # ---- stick side view, aligned with the top-view x scale
    L = sinfo["layout"]
    D = L["depth"]
    xs_, ys_, keep = sinfo["grid"]
    top = stick_top
    vy = lambda v: top - v * s * 2                                   # side view vertical scale x2 for readability
    o.append(_txt(20, top - 40, f"Stick side view (printed lying on its side, {STICK_WIDTH_MM:g} mm thick). Vertical scale is x2. Wing root glues onto the flat top over the full root chord 0 to {g['cr']:.1f}.", 11, color="#566473"))
    for (xa, ya), (xb, yb) in grid_boundary(xs_, ys_, keep):
        o.append(_ln(sx(xa), vy(ya), sx(xb), vy(yb), "#18222d", 1.0))
    o.append(_ln(sx(0), vy(0) - 6, sx(g["cr"]), vy(0) - 6, "#c2621a", 3))
    o.append(_txt(sx(g["cr"] / 2), vy(0) - 12, "saddle: wing root glues here", 10, "middle", "#c2621a"))
    for tk in sinfo["ticks"]:
        if tk["x"] % 10 == 0:
            o.append(_txt(sx(tk["x"]), vy(-D) + 24, tk["x"], 10, "middle"))
            o.append(_ln(sx(tk["x"]), vy(-D) + 4, sx(tk["x"]), vy(-D) + 10, "#555", 0.8))
    o.append(_txt(sx(sinfo["ticks"][0]["x"]) - 8, vy(-D) + 24, "mm aft of root LE:", 10, "end"))
    if settings:
        o.append(f'<rect x="{sx(settings[0]):.1f}" y="{vy(0) - 20:.1f}" width="{(settings[-1] - settings[0]) * s:.1f}" height="{(D * s * 2) + 60:.1f}" fill="#28834f" fill-opacity="0.15"/>')
    if A:
        for key, col in (("fins", "#b0261d"), ("bare", "#793da4")):
            o.append(_ln(sx(A[key]["xnp_mm"]), vy(0) - 20, sx(A[key]["xnp_mm"]), vy(-D) + 30, col, 1.2, "6 3"))
    o.append(_txt(sx(-L["nose"]), vy(-D) + 48, f"nose {L['nose']:.1f} ahead of root LE (G); square holes: ballast / release hook", 10))
    o.append(_txt(20, H - 26, "Ticks: 1 mm short, 5 mm wider notch (also seats the drone-drop loop), 10 mm deepest.", 10, color="#566473"))
    o.append(_txt(20, H - 12, "Green band = CG test range (JUDGEMENT). Dashed = AVL neutral point (outline only, no section effects).", 10, color="#566473"))
    o.append("</svg>")
    return "\n".join(o)


# =============================================================================================== CSV files
LOG_COLUMNS = ["date", "wing", "thickness_mm", "config", "measured_mass_g", "measured_cg_mm", "launch_method", "release_no", "outcome", "release_height_m", "release_speed_mps", "distance_m", "time_s", "notes"]


def write_top_csvs(outdir: Path, results, thickness):
    f2 = lambda v: "" if v is None else f"{v:.2f}"
    with (outdir / "scale_table.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        cols = [("wing", ""), ("material", ""), ("span_mm", "GEOMETRY"), ("thickness_mm", "GEOMETRY; ASSUMED for foamboard"), ("area_cm2", "GEOMETRY"), ("root_chord_mm", "GEOMETRY"), ("tip_chord_mm", "GEOMETRY"),
                ("mac_mm", "GEOMETRY"), ("plate_mass_g", "ARITHMETIC"), ("rib_mass_g", "ARITHMETIC"), ("wing_mass_g", "ARITHMETIC"), ("fins_mass_g", "ARITHMETIC"), ("stick_mass_g", "ARITHMETIC"),
                ("allup_no_ballast_g", "ARITHMETIC"), ("wing_only_loading_n_m2", "ARITHMETIC"), ("loading_no_ballast_n_m2", "ARITHMETIC"), ("ballast_middle_setting_g", "ARITHMETIC"),
                ("allup_with_ballast_g", "ARITHMETIC"), ("loading_with_ballast_n_m2", "ARITHMETIC"), ("speed_cl0.3_m_s", "ARITHMETIC"), ("speed_cl0.5_m_s", "ARITHMETIC"),
                ("re_mean_chord_cl0.3", "ARITHMETIC"), ("re_tip_chord_cl0.3", "ARITHMETIC"), ("re_mean_chord_cl0.5", "ARITHMETIC"), ("re_tip_chord_cl0.5", "ARITHMETIC"),
                ("thickness_pct_mean_chord", "GEOMETRY; ASSUMED for foamboard"), ("thickness_pct_tip_chord", "GEOMETRY; ASSUMED for foamboard"),
                ("tip_bend_plate_only_mm_ROUGH", "ARITHMETIC+ASSUMED"), ("tip_bend_specimen_mm_ROUGH", "ARITHMETIC+ASSUMED; T-section if a rib is fitted")]
        w.writerow([f"{c}[{s}]" if s else c for c, s in cols])
        for r in results:
            g = r["g"]
            mat = "foamboard" if r["kind"] == "foam" else ("PLA plain plate" if r["plain"] else "PLA plate + spar rib")
            for tt, x in r["thick"].items():
                w.writerow([r["name"], mat, g["span"], tt, f"{g['area'] / 100:.2f}", f"{g['cr']:.2f}", f"{g['ct']:.2f}", f"{g['mac']:.2f}",
                            f2(x["plate_mass"]), f2(x["rib_mass"]), f2(x["wing_mass"]), f2(x["fins_mass"]), f2(x["stick_mass"]), f2(x["allup_noballast"]), f2(x["wing_only_ws"]), f2(x["ws_noballast"]),
                            f2(x["ballast_mid"]), f2(x["allup_ballast"]), f2(x["ws_ballast"]),
                            f2(x["V_cl0.3"]), f2(x["V_cl0.5"]), f"{x['Re_mean_cl0.3']:.0f}", f"{x['Re_tip_cl0.3']:.0f}", f"{x['Re_mean_cl0.5']:.0f}", f"{x['Re_tip_cl0.5']:.0f}",
                            f2(x["t_pct_mean"]), f2(x["t_pct_tip"]), f2(x["tip_bend_plain"]), f2(x["tip_bend"])])
    with (outdir / "ballast_table.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["wing", "thickness_mm", "cg_mm_aft_of_root_LE[JUDGEMENT]", "cg_mm_ahead_of_AVL_NP_with_fins[AVL]", "ballast_station_mm_aft_of_root_LE[GEOMETRY]",
                    "ballast_g[ARITHMETIC; negative = nose ballast cannot reach this CG]"])
        for r in results:
            for b in r["ballast_rows"]:
                w.writerow([b["wing"], b["thickness_mm"], b["cg_mm_aft_of_root_LE"], b["cg_mm_ahead_of_NP"], b["ballast_station_mm"], f"{b['ballast_g']:.2f}"])
    with (outdir / "log_template.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(LOG_COLUMNS)
        w.writerow(["EXAMPLE (ignored by the report script)", "EXAMPLE-F45", 0.6, "rib-down, tip fins", 14.1, 41.5, "drone drop, open hook", 1, "steady glide", 10, 7, 12.4, 4.2, "EXAMPLE ONLY - not a real flight; heading change from video goes in notes"])
        w.writerow(["EXAMPLE", "EXAMPLE-F45", 0.6, "rib-down, tip fins", 14.1, 41.5, "hand", 2, "dive", "", "", "", "", "EXAMPLE ONLY - hand launch, gross trim only"])
        w.writerow(["EXAMPLE", "EXAMPLE-B45", 5.0, "foam, tip fins", 22.0, 52.0, "drone drop, open hook", 1, "roll-off left", 10, 7, "", 1.5, "EXAMPLE ONLY"])
    with (outdir / "preflight_sheet.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["wing", "part", "assumed_mass_g[ARITHMETIC+ASSUMED]", "measured_mass_g[UNKNOWN: fill in]"])
        for r in results:
            x = r["thick"].get(thickness) or list(r["thick"].values())[0]
            if r["kind"] == "foam":
                w.writerow([r["name"], f"foam wing, {FOAM_AREAL_MASS_G_M2:g} g/m2 x area (ASSUMED)", f"{x['wing_mass']:.2f}", ""])
                w.writerow([r["name"], "2 foam fins", f"{x['fins_mass']:.2f}", ""])
            else:
                w.writerow([r["name"], f"wing plate {thickness:g} mm" + ("" if r["plain"] else " + rib"), f"{x['wing_mass']:.2f}", ""])
                w.writerow([r["name"], "2 fins", f"{x['fins_mass']:.2f}", ""])
            w.writerow([r["name"], "stick", f"{x['stick_mass']:.2f}", ""])
            w.writerow([r["name"], "glue / tape / ballast", "not assumed", ""])
            w.writerow([r["name"], "complete aircraft", f"{x['allup_noballast']:.2f} (before ballast)", ""])


def write_assumptions(outdir: Path, results):
    rows = [
        ("PLA density", PLA_DENSITY_G_CM3, "g/cm3", "ASSUMED", "Used for every printed mass. 100 % solid. Replace with weighed parts.", "weigh each part"),
        ("PLA stiffness E", E_PLA_GPA, "GPa", "ASSUMED", "Only used for the rough tip-bend estimate. Printed flat plates are stiffer along the print lines than across them.", "measure with a deflection test"),
        ("Air density", RHO_AIR, "kg/m3", "ASSUMED", "Sea level, standard day. Used for speed.", "none needed for a rough check"),
        ("Air viscosity", NU_AIR, "m2/s", "ASSUMED", "Used for Reynolds number.", "none needed for a rough check"),
        ("Printed plate thickness (default)", THICKNESS_MM, "mm", "GEOMETRY", "3 layers of 0.2 mm. The printed part may differ by a layer.", "measure with calipers"),
        ("Spar rib (printed wings, default)", f"{RIB_WIDTH_MM:g} mm wide x {RIB_HEIGHT_MM:g} mm high above the plate, along the quarter chord of each half, {RIB_TIP_STOP_MM:g} mm short of the tip, gap {STICK_WIDTH_MM + RIB_CENTRE_GAP_EXTRA_MM:g} mm at the centre", "", "GEOMETRY",
         "Printed on top of the plate, flown rib-down with the stick. With a rib the wing is a flat plate with a spar underneath, not a plain plate. Reasoning, not a measured fact: the upper surface is where flow separation matters most, so the disturbance goes underneath.", "print one plain wing (--plain) to compare"),
        ("Rib stiffness estimate", "T-section second moment of area at the mean chord", "", "ASSUMED", "ROUGH: treats the rib as running the whole half span; ignores sweep and taper.", "flex the real wing and look"),
        ("Dihedral", DIHEDRAL_DEG, "deg", "GEOMETRY", "Zero. A one-piece flat wing cannot have dihedral. The earlier model used 3 deg per panel, so these wings differ from what was modelled.", "-"),
        ("Foamboard thickness", FOAM_THICKNESS_MM, "mm", "ASSUMED", "Typical craft foamboard. Kai to measure; the page has a blank for it. It is above the 2.6-4.3 % of the published plate tests.", "measure with calipers"),
        ("Foamboard areal mass", FOAM_AREAL_MASS_G_M2, "g/m2", "ASSUMED", "Typical light craft foamboard with paper. To be weighed (cut a known area and weigh it).", "weigh a measured area"),
        ("Glue, tape, paint mass", GLUE_MASS_G, "g", "UNKNOWN", "Not included in any mass here.", "weigh the finished aircraft"),
        ("Tip fin size", f"{FIN_HEIGHT_MM_AT_400:g} mm x span/400 tall, chord = tip chord", "", "GEOMETRY", "Each fin is about 4 % of wing area. Fins are a design choice, not a result. Printed fins are centred on the plate mid-plane (not including the rib); foamboard fins are butt-glued centred on the board mid-plane.", "-"),
        ("Fin / tab clearance (printed)", f"slot = plate + {SLOT_CLEARANCE_MM} mm wide, tab + {SLOT_LEN_CLEARANCE_MM} mm long", "", "GEOMETRY", "A printer may need more or less. Test-fit before gluing.", "adjust in flat_test.py"),
        ("Stick shape", f"{STICK_WIDTH_MM:g} mm x {STICK_DEPTH_MM:g} mm, nose {NOSE_FRAC_OF_ROOT_CHORD:g} root chords ahead", "", "GEOMETRY", "Square holes because the part is built on a grid.", "-"),
        ("Foamboard edge shaping", "round the leading edge to about a half-ellipse 2 board-thicknesses long; taper the last 20 mm of the trailing edge to about 1 mm; same on both halves", "", "JUDGEMENT", "A procedure we picked, not a result. Use a sanding block and check both sides match.", "-"),
        ("CG test start", f"{CG_START_FRAC_MAC_AHEAD_OF_NP:g} MAC ahead of the AVL neutral point", "", "JUDGEMENT", "Picked to start on the safe (nose-heavy) side. Not a prediction.", "move if flights say so"),
        ("CG table end", f"at least {CG_END_FRAC_MAC_BEHIND_NP:g} MAC behind the AVL neutral point, then the stop rule", "", "JUDGEMENT", "Printed so the sweep does not stop at the neutral point: the published free-flight study (Li et al. 2022, unswept plates) is a reason to keep testing past it, not a prediction.", "follow the stop rule"),
        ("Stop rule", "keep going aft in 1 mm steps while any setting still glides or still dives; stop when, after at least one gliding setting, two settings in a row give no steady glide, or at the bare-aircraft balance point", "", "JUDGEMENT", "Beyond the bare-aircraft balance point ballast must go in the holes under the wing instead of the nose.", "agree before flying"),
        ("Li et al. 2022", "J. Fluid Mech. 937, A6, doi 10.1017/jfm.2022.89", "", "ASSUMED", "Published free-flight result on unswept rectangular plates (paper sheets in air, plastic plates in water, Re about 1,000 to 10,000). Quoted in its own terms on the page. Not our wings.", "-"),
        ("Optional plain wing", "F45-plain: the F45 wing with no rib", "", "JUDGEMENT", "Optional. One plain wing shows whether the rib changes the behaviour. Floppier (rough tip-bend figure on the page).", "print only if wanted"),
        ("CG step", CG_STEP_MM, "mm", "JUDGEMENT", "Step aft between settings.", "-"),
        ("Releases per setting", f"{RELEASES_PER_SETTING}, then {RELEASES_AT_GLIDING_SETTING} at any setting that glides", "", "JUDGEMENT", "A small count; luck can matter.", "-"),
        ("Pass rule", f">= {PASS_STEADY_OF_10} of {RELEASES_AT_GLIDING_SETTING} steady at one setting and both 1 mm neighbours also glide", "", "PROPOSED", "To be agreed before flying. 'Also glide' = at least half of that neighbour's releases are steady glides.", "agree first"),
        ("AVL neutral point and Cn_beta", "AVL 3.52, flat camber line, zero thickness", "", "AVL", "AVL knows only the outline. It does not know the thickness, a rib, the low Reynolds number or the edges.", "compare with a measured balance point"),
        ("Weight on the wing for bend estimate", "all-up mass with ballast, 1 g", "", "ASSUMED", "Each half carries half the weight spread evenly; ignores sweep, taper and inertia relief. Shows which thicknesses are floppy, nothing more.", "look at the real wing under load"),
        ("Ballast hardware", f"M3 screws and nuts in {BALLAST_HOLE_MM:g} mm square holes at {BALLAST_PITCH_MM:g} mm pitch; fine trim with a taped-on washer or coin slid along the stick", "", "UNKNOWN", "M3 hardware masses vary: weigh yours. The measured CG on the knife edge is what counts, not the ballast table.", "weigh what you use"),
        ("US penny", 2.5, "g", "ASSUMED", "Nominal mass. Weigh yours.", "weigh"),
        ("US dime", 2.27, "g", "ASSUMED", "Nominal mass. Weigh yours.", "weigh"),
        ("US nickel", 5.0, "g", "ASSUMED", "Nominal mass. Weigh yours.", "weigh"),
        ("Drone drop height", DROP_HEIGHT_M, "m", "JUDGEMENT", "Suggested release height from a 5-inch FPV quad. The same every time.", "change if the field or the flights say so"),
        ("Phugoid period", "T = pi x sqrt(2) x V / g, about 0.45 x V seconds", "s", "ARITHMETIC", "Textbook estimate of the slow nose-up / nose-down oscillation period of a glider at speed V. It shows why a few seconds of flight are needed.", "read it from the video"),
        ("Published plate data", "Ananda, Sukumar & Selig 2015", "", "ASSUMED", "4.3 % plate (2.6-4.3 % range), AR 2-5, unswept, Re 60,000 and above. These wings are swept, fly lower and (printed) carry a rib. Not the same test.", "-"),
    ]
    with (outdir / "assumptions.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["item", "value", "unit", "source_tag", "what_it_means", "how_to_replace_with_measurement"])
        w.writerows(rows)
    return rows


# =============================================================================================== index.html
CSS = """:root{color-scheme:dark;--bg:#14181d;--panel:#1c2229;--line:#333c46;--ink:#dde3ea;--muted:#9aa7b4;--link:#7fb4ff;--accent:#e89a55;--thead:#262e37;--bad:#ff8a80}
body{margin:0;background:var(--bg);color:var(--ink);font:16px/1.55 "Segoe UI",system-ui,sans-serif}
main{max-width:1100px;margin:0 auto;padding:32px 20px 64px;display:flex;flex-direction:column;gap:30px}
h1{font-size:28px;margin:0}h2{font-size:21px;margin:0}h3{font-size:16px;margin:0}p{margin:0;max-width:74ch}
section{display:flex;flex-direction:column;gap:12px}
figure{margin:0;background:var(--panel);border:1px solid var(--line);padding:12px}
figure img{display:block;max-width:100%;height:auto}
figcaption{font-size:14px;color:var(--muted);margin-top:8px}
.note{border-left:3px solid var(--accent);padding:2px 0 2px 12px;color:var(--muted);font-size:15px}
.box{background:var(--panel);border:2px solid var(--accent);padding:12px 16px}
table{border-collapse:collapse;font-size:14.5px;background:var(--panel)}
th,td{border:1px solid var(--line);padding:6px 10px;text-align:left;vertical-align:top}td.n{text-align:right;font-variant-numeric:tabular-nums}
.scroll{overflow-x:auto}a{color:var(--link)}code{font-size:14px}
ol,ul{margin:0;padding-left:22px;max-width:76ch;display:flex;flex-direction:column;gap:6px}
.cols{display:grid;grid-template-columns:repeat(auto-fit,minmax(340px,1fr));gap:20px}
.tag{font-size:11px;color:var(--muted);white-space:nowrap}
th{background:var(--thead)}
.err{color:var(--bad)}
table.cgt td,table.cgt th{padding:4px 5px;font-size:13px}table.cgt td:first-child,table.cgt th:first-child{white-space:nowrap}
td.behind,th.behind{background:#2c2418}
td.ahead,th.ahead{background:#18242c}
"""


def tag(s):
    return f' <span class="tag">[{s}]</span>'


def fmt(x, n=1):
    return "-" if x is None else f"{x:.{n}f}"


def drop_table(rs, masses):
    """Speed at CL 0.3 / 0.5 and phugoid period against measured mass (ARITHMETIC)."""
    P = ["<div class='scroll'><table><tr><th>Measured mass, g</th>" + "".join(f"<th>{r['name']} speed at CL 0.3 / 0.5, m/s</th><th>{r['name']} loading, N/m&sup2;</th><th>{r['name']} bob period at those speeds, s</th>" for r in rs) + "</tr>"]
    for mg in masses:
        cells = ""
        for r in rs:
            W = mg * 1e-3 * 9.80665
            v3, v5 = speed(W, r["g"]["area"], 0.3), speed(W, r["g"]["area"], 0.5)
            cells += (f"<td class='n'>{v3:.1f} / {v5:.1f}</td><td class='n'>{W / (r['g']['area'] * 1e-6):.1f}</td>"
                      f"<td class='n'>{math.pi * math.sqrt(2) * v3 / 9.80665:.1f} / {math.pi * math.sqrt(2) * v5 / 9.80665:.1f}</td>")
        P.append(f"<tr><td class='n'>{mg}</td>{cells}</tr>")
    P.append("</table></div>")
    return "".join(P)


def index_html(results, thickness, rows_assump, outdir: Path):
    h = html.escape
    printed = [r for r in results if r["kind"] == "printed" and not r.get("optional")]
    plainx = [r for r in results if r.get("optional")]
    foam = [r for r in results if r["kind"] == "foam"]
    P = []
    spec = lambda r: r["thick"].get(thickness) or list(r["thick"].values())[0]
    re_vals = [spec(r)[f"Re_{k}_cl{cl}"] for r in results if not r.get("optional") for k in ("tip", "mean") for cl in CL_LIST] or [0]
    re_lo, re_hi = min(re_vals), max(re_vals)
    rib_on = any(r["rib"] for r in printed)
    if foam:
        fr_ = foam[0]
        fp_mean, fp_tip = spec(fr_)["t_pct_mean"], spec(fr_)["t_pct_tip"]
    P.append(f"<!doctype html><html lang='en'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'><title>Flat-wing flight test</title><style>{CSS}</style></head><body><main>")
    P.append("<h1>Flat-wing flight test</h1>")
    P.append("<p class='note'>This page is a test plan, not a prediction. Nothing here says a flat wing will fly. Every number has a tag: GEOMETRY (exact from the drawing), ARITHMETIC (a simple sum, inputs stated), AVL (a computer calculation that knows only the outline), ASSUMED (a stated guess), JUDGEMENT (a choice we made), PROPOSED (to be agreed), UNKNOWN (to be measured by flying or weighing).</p>")
    P.append("<section><h2>1. What this test is for</h2><p>We want to find out, by flying, whether a thin flat wing can glide the same way every time, and what we lose by making it slimmer.</p>"
             "<p>There are three specimens. <b>Two printed wings</b> (F45 and F60) have the same span, 250 mm, and differ only in how slim they are. <b>One foamboard wing</b> (B45) is bigger (400 mm span) and thicker. The foamboard wing differs from the printed ones in size, thickness <i>and</i> material, so it is its own test, not a third point on the same comparison.</p>"
             "<p>There is also <b>one optional extra wing</b>: F45-plain, the F45 wing with no rib. It is not part of the main comparison (see section 3).</p>"
             "<p>The test has one control you change: where the balance point (centre of gravity, CG) sits along the body. You move it a millimetre at a time and drop each setting several times.</p></section>")
    P.append("<section><h2>2. What we know and what we do not know</h2><div class='box'><ul>"
             "<li><b>Our computer model cannot tell us anything about a flat plate.</b> Its air-flow solver (XFOIL) did not give usable answers for this shape. When we forced it to, results depended on how the back edge was drawn and jumped suddenly. So there is no predicted balance range, glide ratio or &quot;will fly&quot; for these wings.</li>"
             "<li>Published wind-tunnel tests of flat plates 2.6 to 4.3 % thick (Ananda, Sukumar &amp; Selig 2015) stopped at Reynolds number 60,000. Reynolds number is a measure of how thick the air feels to the wing; low means harder.</li>"
             f"<li>These wings fly at roughly {re_lo:,.0f} to {re_hi:,.0f} (ARITHMETIC, from the speed table below; the tip is the lowest). That is below the published tests.</li>"
             "<li>Those published wings were not swept 25 degrees and were not this small.</li>"
             "<li>There is no data on a lip (a small upturned back edge) on a plate. Not part of this test.</li>"
             "<li>The AVL neutral point is for the outline only. It is the best number we have for where to start, nothing more. The test sweep runs on past it (see the next block).</li>"
             "<li>Zero dihedral (wings flat, not tilted up). The earlier model used 3 degrees per panel. A one-piece flat wing cannot have it, so these wings are different from what was modelled.</li>"
             + (f"<li><b>The printed wings carry a spar rib</b> ({RIB_WIDTH_MM:g} mm wide, {RIB_HEIGHT_MM:g} mm high, along the quarter chord of each half). With a rib the section is no longer a plain flat plate: it is a flat plate with a spar underneath, which is what a practical flat wing would be. Any difference from the published plain-plate tests may partly come from the rib. If you want to separate the two effects, print one plain wing (run the generator with <code>--plain</code>) as a comparison.</li>"
                "<li>The rib prints on top of the plate, but the wing is <b>flown rib-down</b>: the rib and the stick are both on the underside. Reasoning, not a measured fact: the upper surface is the one where flow separation matters most, so we put the disturbance underneath.</li>" if rib_on else
                "<li>These printed wings are plain plates (no rib), as in the published tests apart from size, sweep and thickness.</li>")
             + (f"<li><b>Foamboard is thick for this job.</b> {FOAM_THICKNESS_MM:g} mm foamboard (ASSUMED, measure yours) on the 400 mm wing is {fp_mean:.1f} % of the mean chord and {fp_tip:.1f} % of the tip chord. That is above the 2.6 to 4.3 % of the published plate tests, and the 250 mm printed wings are about 1 to 2 % thick. Foamboard is also too thick for the 250 mm wings, which is why the foamboard wing is the larger one.</li>" if foam else "")
             + "</ul><h3>A published free-flight result, and why we keep testing past the AVL neutral point</h3>"
             "<p>Li, Goodwill, Wang &amp; Ristroph (2022), &quot;Centre of mass location, flight modes, stability and dynamic modelling of gliders&quot;, J. Fluid Mech. 937, A6, doi 10.1017/jfm.2022.89 (open access), dropped weighted flat plates and moved the centre of mass along the chord. We report their result in their own terms: unswept rectangular plates, as a fraction of chord. It says nothing about the wings in this test.</p>"
             "<ul>"
             "<li><b>Paper sheets in air</b>, 15 cm span x 5 cm chord, weighted leading edge, small side fins. Centre of mass measured from mid-chord toward the leading edge, as a fraction of chord: 0 and 0.08 tumbled; 0.14 was unsteady (bounding); 0.24 glided; 0.31 dived. Measured from the leading edge that is 50 %, 42 %, 36 %, 26 % and 19 % of chord (ARITHMETIC: 0.5 minus the value).</li>"
             "<li><b>Plastic plates in water</b>, 20.3 cm span x 2.54 cm chord x 0.15 cm thick, unswept rectangles, Reynolds number about 1,000 to 10,000: steady gliding for a centre of mass 0.22 to 0.29 ahead of mid-chord, that is 21 % to 28 % of chord from the leading edge (ARITHMETIC).</li>"
             "<li>Their stated mechanism: the centre of pressure on the plate moves with angle of attack (as the flow goes from attached to separated). A steady glide is where it coincides with the balance point.</li>"
             "<li>Ananda, Sukumar &amp; Selig (2015) give an aerodynamic centre of 20 to 27 % of chord at low angle of attack, moving aft toward stall (read from the first 13 of 15 pages of the paper).</li>"
             "</ul><p><b>What we do not know, in plain words.</b> (a) That study used unswept rectangles at a lower Reynolds number than ours. (b) On a swept, tapered wing the tips may let go first as the nose rises, which would make settings behind the neutral point pitch up rather than glide. We do not know. (c) Our printed wings have a spar rib; the study&#39;s plates did not. (d) So this is a reason to keep testing past the AVL neutral point, not a prediction. Our model has no plate result.</p>"
             "<p>One practical consequence: as the balance point moves aft, the nose ballast gets smaller, so the glider also gets lighter across the sweep. Weigh it at each setting.</p></div></section>")
    P.append("<section><h2>3. The specimens</h2><div class='cols'>")
    for r in results:
        P.append(f"<figure><a href='{r['folder'].name}/drawing.svg'><img src='{r['folder'].name}/drawing.svg' alt='{r['name']} drawing'></a><figcaption>{r['name']}{' (optional extra: no rib; one plain wing shows whether the rib changes the behaviour)' if r.get('optional') else ''}: drawing with dimensions, stick, CG scale. Click to enlarge.</figcaption></figure>")
    P.append("</div>")
    P.append("<div class='scroll'><table><tr><th></th>" + "".join(f"<th>{r['name']}</th>" for r in results) + "</tr>")

    def row(label, fn, src):
        P.append(f"<tr><td>{label}{tag(src)}</td>" + "".join(f"<td class='n'>{fn(r)}</td>" for r in results) + "</tr>")
    row("Material", lambda r: "foamboard" if r["kind"] == "foam" else ("PLA, plain plate" if r["plain"] else "PLA, plate + spar rib"), "GEOMETRY")
    row("Aspect ratio (span squared / area)", lambda r: f"{r['g']['ar']:g}", "GEOMETRY")
    row("Span, mm", lambda r: f"{r['g']['span']:g}", "GEOMETRY")
    row("Area, cm&sup2;", lambda r: f"{r['g']['area'] / 100:.1f}", "GEOMETRY")
    row("Root chord (front to back at the centre), mm", lambda r: f"{r['g']['cr']:.1f}", "GEOMETRY")
    row("Tip chord, mm", lambda r: f"{r['g']['ct']:.1f}", "GEOMETRY")
    row("Mean aerodynamic chord (MAC), mm", lambda r: f"{r['g']['mac']:.1f}", "GEOMETRY")
    row("MAC front edge, mm aft of root front edge", lambda r: f"{r['g']['mac_le_x']:.1f}", "GEOMETRY")
    row("Tip front edge, mm aft of root front edge", lambda r: f"{r['g']['tipx']:.1f}", "GEOMETRY")
    row("Plate / board thickness, mm", lambda r: (f"{r['t']:g} (ASSUMED; measure: ______)" if r["kind"] == "foam" else f"{r['t']:g}"), "GEOMETRY for printed; ASSUMED for foamboard")
    row("Thickness as % of mean chord", lambda r: f"{100 * r['t'] / r['g']['mean_chord']:.2f}", "GEOMETRY; ASSUMED for foamboard")
    row("Thickness as % of tip chord", lambda r: f"{100 * r['t'] / r['g']['ct']:.2f}", "GEOMETRY; ASSUMED for foamboard")
    row("Foamboard areal mass, g/m&sup2;", lambda r: (f"{FOAM_AREAL_MASS_G_M2:g} (ASSUMED; weigh: ______)" if r["kind"] == "foam" else "-"), "ASSUMED")
    row("Each tip fin: height x chord, mm", lambda r: f"{r['fd']['height']:.1f} x {r['g']['ct']:.1f}", "GEOMETRY")
    row("Each tip fin, % of wing area", lambda r: f"{r['fin_pct']:.2f}", "GEOMETRY")
    row("Neutral point, bare, mm aft of root front edge", lambda r: fmt(r["A"]["bare"]["xnp_mm"]) if r["A"] else "n/a", "AVL, outline only")
    row("Neutral point, with tip fins, mm", lambda r: fmt(r["A"]["fins"]["xnp_mm"]) if r["A"] else "n/a", "AVL, outline only")
    row("Cn_beta at CL 0.3, bare (yaw stiffness, per radian)", lambda r: fmt(r["A"]["bare"]["cn_beta"], 4) if r["A"] else "n/a", "AVL, outline only")
    row("Cn_beta at CL 0.3, with tip fins", lambda r: fmt(r["A"]["fins"]["cn_beta"], 4) if r["A"] else "n/a", "AVL, outline only")
    row("Flat-wing area centre (balance point of the bare plate), mm", lambda r: f"{r['xc_plate']:.1f}", "ARITHMETIC")
    P.append("</table></div><p class='note'>AVL knows only the outline. It treats the wing as a flat sheet of no thickness, in smooth air with no low-Reynolds-number effects, and it does not know about the rib or the edges. Cn_beta is how strongly the wing turns back into the wind when it yaws; larger is steadier. Dihedral is zero in these runs.</p></section>")
    P.append("<section><h2>4. What to print and what to cut</h2><h3>Printed wings (F45, F60)</h3><ul>")
    for r in printed:
        n = r["folder"].name
        P.append(f"<li>{r['name']}: <a href='{n}/wing.stl'>wing.stl</a> (print 1), <a href='{n}/fin.stl'>fin.stl</a> (print 2), <a href='{n}/stick.stl'>stick.stl</a> (print 1). Also <a href='{n}/parts.dxf'>parts.dxf</a>, <a href='{n}/drawing.svg'>drawing.svg</a>, <a href='{n}/geometry.json'>geometry.json</a> (every number with its tag). Wing box {r['sizes']['wing'][0]:.0f} x {r['sizes']['wing'][1]:.0f} x {r['sizes']['wing'][2]:.1f} mm (GEOMETRY"
                 + ("; the height includes the rib" if r["rib"] else "") + f"); fin {r['sizes']['fin'][0]:.0f} x {r['sizes']['fin'][1]:.0f} x {r['sizes']['fin'][2]:.1f} mm; stick {r['sizes']['stick'][0]:.0f} x {r['sizes']['stick'][1]:.0f} x {r['sizes']['stick'][2]:.1f} mm.</li>")
    for r in plainx:
        n = r["folder"].name
        P.append(f"<li><b>Optional:</b> {r['name']} (the F45 wing with no rib, same fins and stick): <a href='{n}/wing.stl'>wing.stl</a>, <a href='{n}/fin.stl'>fin.stl</a> (print 2), <a href='{n}/stick.stl'>stick.stl</a>, <a href='{n}/drawing.svg'>drawing.svg</a>, <a href='{n}/geometry.json'>geometry.json</a>. The published plate result is for a plain plate, so one plain wing shows whether the rib changes the behaviour. It is much floppier (rough tip bend {spec(r)['tip_bend_plain']:.1f} mm at 1 g against {spec(printed[0])['tip_bend']:.1f} mm for the ribbed F45; ROUGH/ASSUMED). Print it only if you want that comparison; it is not part of the main test.</li>")
    P.append(f"</ul><p>Settings: layer height {LAYER_MM:g} mm, 100 % infill, print the wing and fins <b>flat on the bed</b>"
             + (" with the rib on top" if rib_on else "") + f", the stick on its side. Units are millimetres. The wing's front edge at the centre is the origin of the STL. A {thickness:g} mm plate is {thickness / LAYER_MM:.0f} layers; check your slicer really makes it solid, not a thin wall with gaps. Try {', '.join(f'{x:g}' for x in THICKNESS_TABLE_MM)} mm by re-running the generator with <code>--thickness</code>.</p>")
    for r in foam:
        n = r["folder"].name
        P.append(f"<h3>Foamboard wing ({r['name']})</h3><ul><li>Cutting templates, 1:1 on US Letter portrait: <a href='{n}/template_half_wing_letter.svg'>template_half_wing_letter.svg</a> (ONE half wing, with the CG range band and the AVL neutral-point line, centreline marked &quot;flip here for the other half&quot;) and <a href='{n}/template_fins_letter.svg'>template_fins_letter.svg</a> (two fins). Each sheet has a 100 mm scale bar: measure it on your print, it must be exactly 100 mm.</li>"
                 f"<li>Printed stick for this wing: <a href='{n}/stick.stl'>stick.stl</a>, {r['sizes']['stick'][0]:.0f} x {r['sizes']['stick'][1]:.0f} x {r['sizes']['stick'][2]:.1f} mm (GEOMETRY)." + (f" <b>This stick is longer than {STICK_LENGTH_WARN_MM:g} mm ({r['stick_len']:.0f} mm): check it fits your print bed.</b>" if r["stick_len"] > STICK_LENGTH_WARN_MM else f" It is {r['stick_len']:.0f} mm long, within {STICK_LENGTH_WARN_MM:g} mm.") + "</li>"
                 f"<li>Also <a href='{n}/parts.dxf'>parts.dxf</a>, <a href='{n}/drawing.svg'>drawing.svg</a>, <a href='{n}/geometry.json'>geometry.json</a>. The whole wing is {r['g']['span']:g} x {r['g']['tipx'] + r['g']['ct']:.0f} mm, so the board must be at least that big.</li></ul>")
    P.append("</section>")
    P.append("<section><h2>5. Assemble</h2><h3>Printed wings</h3><ol><li>Test-fit the fins: each fin has two slots, the wing has two tabs on each tip edge. Slot width is plate thickness plus 0.1 mm, slot length tab length plus 0.1 mm (GEOMETRY). Adjust if your printer is tight or loose.</li>"
             "<li>Press a fin onto each wing tip so it stands up, centred on the plate mid-plane (the plate, not including the rib). Glue the tabs.</li>"
             "<li>Check the wing lies flat on a table with no twist. Photograph it.</li>"
             + ("<li>Glue the stick onto the <b>same face as the rib</b>, in the gap between the two rib halves, saddle flat to the plate and root front edge on the stick mark labelled 0. In flight this face is the underside: <b>the wing is flown rib-down</b>, rib and stick both underneath.</li>" if rib_on else
                "<li>Glue the stick flat onto the underside of the wing root, root front edge on the stick mark labelled 0.</li>")
             + "<li>Weigh every part and fill in the &quot;measured&quot; column below.</li></ol>")
    if foam:
        P.append("<h3>Foamboard wing: cut and build</h3><ol>"
                 "<li>Measure your board thickness and weigh a piece of known area. Write both in the table below (the numbers on this page are ASSUMED until you do).</li>"
                 "<li>Print the half-wing template at 100 %; check the 100 mm bar. Trace the half onto the board, flip the template on the centreline, trace the other half. Cut with a sharp blade and a straight edge. The wing is one piece, zero dihedral.</li>"
                 "<li>Edge shaping (procedure, JUDGEMENT): round the leading edge to about a half-ellipse two board-thicknesses long; taper the last 20 mm of the trailing edge to about 1 mm; same on both halves. Use a sanding block and check both sides match.</li>"
                 "<li>Cut two fins from the fin template. Butt-glue each to the wing tip edge, centred on the board mid-plane (no tabs). Check they stand square.</li>"
                 "<li>Glue the printed stick under the wing root (saddle flat to the board), root front edge on stick mark 0. The wing is flat, so it stays flat: no rib on this wing.</li>"
                 "<li>Weigh every part. Check the board is not warped.</li></ol>")
    P.append("<div class='scroll'><table><tr><th>Wing</th><th>Part</th><th>Assumed mass, g</th><th>Measured, g</th></tr>")
    for r in results:
        x = spec(r)
        if r["kind"] == "foam":
            parts = ((f"foam wing ({FOAM_AREAL_MASS_G_M2:g} g/m&sup2; ASSUMED, thickness {r['t']:g} mm ASSUMED)", x["wing_mass"]), ("2 foam fins", x["fins_mass"]), ("stick", x["stick_mass"]))
        else:
            parts = ((f"wing plate {thickness:g} mm" + ("" if r["plain"] else " + rib"), x["wing_mass"]), ("2 fins", x["fins_mass"]), ("stick", x["stick_mass"]))
        for lab, v in parts:
            P.append(f"<tr><td>{r['name']}</td><td>{lab}</td><td class='n'>{v:.2f}{tag('ARITHMETIC, material ASSUMED')}</td><td>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;</td></tr>")
        P.append(f"<tr><td>{r['name']}</td><td>glue, tape, ballast, anything else</td><td>not assumed{tag('UNKNOWN')}</td><td></td></tr>")
    P.append("</table></div></section>")
    P.append("<section><h2>6. Balance</h2><p>Find the CG with a knife edge or a pin: lay the stick on it, move until it balances level, read the position off the stick ticks (mm aft of the root front edge) and note to 0.5 mm. "
             "Ballast is M3 screws and nuts in the row of 3.2 mm square holes (5 mm pitch, along the nose and under the wing). Do the fine trim with a washer or coin taped on and slid along the stick. The ballast table gives the grams needed against hole position, but <b>the CG you measure on the knife edge is what counts, not the table</b>.</p>"
             "<p>Nominal masses of things you may have (ASSUMED, weigh yours): " + ", ".join(f"{n} {m:g} g" for n, m in COINS_G) + ". M3 screws, nuts and washers: weigh yours (UNKNOWN).</p>")
    P.append("<div class='box'><h3>Stop rule (JUDGEMENT)</h3><p>The tables below start 15 % of MAC ahead of the AVL neutral point and end at least 8 % of MAC behind it. Do not stop at the neutral point.</p><ol>"
             "<li>Fly the printed table from the first setting aft, 1 mm at a time, with the same release counts.</li>"
             "<li>Continue aft in 1 mm steps, past the end of the printed table if you need to, while any setting still glides or still dives.</li>"
             "<li>Stop when, after at least one gliding setting, two settings in a row give no steady glide; or when you reach the bare-aircraft balance point (the wing, fins and stick with no ballast). Beyond that point ballast must go in the holes under the wing instead of the nose, which is a different build.</li></ol>"
             "<p>As the balance point moves aft the nose ballast gets smaller, so the glider also gets lighter across the sweep. Weigh it at each setting.</p></div>")
    for r in results:
        s = r["settings"]
        if s and r["A"]:
            np_ = r["np_ref"]
            x = spec(r)
            st = r["default_station"]
            P.append(f"<p><b>{r['name']} CG settings</b> (JUDGEMENT: start 15 % of MAC ahead of the AVL neutral point with fins, {CG_STEP_MM:g} mm steps aft to at least 8 % of MAC behind it, then the stop rule above). Neutral point {np_:.1f} mm{tag('AVL, outline only')}. These are places to try, not a prediction.</p>")
            for c0 in range(0, len(s), 14):                       # long tables are split so they fit the page width
                sc = s[c0:c0 + 14]
                cls = ["behind" if c > np_ + 1e-9 else "ahead" for c in sc]
                P.append("<div class='scroll'><table class='cgt'><tr><th>Setting</th>" + "".join(f"<th class='{k}'>{c0 + i + 1}</th>" for i, k in enumerate(cls)) + "</tr>")
                P.append("<tr><td>mm aft of root front edge</td>" + "".join(f"<td class='n {k}'>{c:g}</td>" for c, k in zip(sc, cls)) + "</tr>")
                P.append("<tr><td>ahead (+) / behind (&minus;) neutral point, mm</td>" + "".join(f"<td class='n {k}'>{np_ - c:+.1f}</td>" for c, k in zip(sc, cls)) + "</tr>")
                P.append("<tr><td>side of the neutral point</td>" + "".join(f"<td class='{k}'>{k}</td>" for k in cls) + "</tr>")
                P.append(f"<tr><td>ballast, g (nose hole at {st:.1f} mm)</td>" + "".join(f"<td class='n {k}'>{ballast_g(x['allup_noballast'], x['cg_noballast'], c, st):.1f}</td>" for c, k in zip(sc, cls)) + "</tr></table></div>")
            P.append(f"<p><b>Bare-aircraft balance point</b> (wing, fins and stick, no ballast): <b>{x['cg_noballast']:.1f} mm</b>{tag('ARITHMETIC; masses ASSUMED')}. The printed table ends at {s[-1]:g} mm; the stop rule ends the sweep at the bare-aircraft balance point at the latest. A negative ballast number would mean that setting is behind it, so nose ballast cannot reach it. Full table: <a href='ballast_table.csv'>ballast_table.csv</a> (four nose holes). Shaded blue = ahead of the neutral point, shaded brown = behind it.</p>")
    P.append("</section>")
    P.append("<section><h2>7. Masses, speeds and a rough stiffness check</h2><div class='scroll'><table><tr><th>Wing</th><th>Plate / board, mm</th><th>Plate g</th><th>Rib g</th><th>Wing g</th><th>All-up, no ballast g</th><th>All-up with ballast g</th><th>Loading N/m&sup2; (with ballast)</th><th>Speed at CL 0.3 / 0.5, m/s</th><th>Re tip / mean chord (CL 0.3)</th><th>Re tip / mean chord (CL 0.5)</th><th>Rough tip bend at 1 g, mm: plate only</th><th>with rib</th></tr>")
    for r in results:
        for tt, x in r["thick"].items():
            bp = f"{x['tip_bend_plain']:.1f}" if x["tip_bend_plain"] is not None else "UNKNOWN"
            br = (f"{x['tip_bend']:.1f}" if r["rib"] else "no rib") if x["tip_bend"] is not None else "UNKNOWN"
            P.append(f"<tr><td>{r['name']}</td><td class='n'>{tt:g}</td><td class='n'>{x['plate_mass']:.1f}</td><td class='n'>{x['rib_mass']:.1f}</td><td class='n'>{x['wing_mass']:.1f}</td><td class='n'>{x['allup_noballast']:.1f}</td><td class='n'>{x['allup_ballast']:.1f}</td><td class='n'>{x['ws_ballast']:.1f}</td><td class='n'>{x['V_cl0.3']:.1f} / {x['V_cl0.5']:.1f}</td><td class='n'>{x['Re_tip_cl0.3']:,.0f} / {x['Re_mean_cl0.3']:,.0f}</td><td class='n'>{x['Re_tip_cl0.5']:,.0f} / {x['Re_mean_cl0.5']:,.0f}</td><td class='n'>{bp}</td><td class='n'>{br}</td></tr>")
    P.append(f"</table></div><p class='note'>Masses: printed parts = volume x {PLA_DENSITY_G_CM3} g/cm&sup3; (ARITHMETIC; density ASSUMED, 100 % solid); foamboard = area x {FOAM_AREAL_MASS_G_M2:g} g/m&sup2; (ASSUMED). With-ballast uses the middle CG setting and the default nose hole. Speed: sqrt(2 x weight / (air density x area x CL)), air density {RHO_AIR} (ASSUMED). Reynolds number = speed x chord / {NU_AIR:g} (ASSUMED). <b>Tip bend is ROUGH and ASSUMED</b>: one wing half as a cantilever, stiffness {E_PLA_GPA} GPa (ASSUMED), half the weight spread evenly, ignoring sweep, taper and the wing's own weight. It only shows which thicknesses are floppy. Foamboard stiffness is UNKNOWN, so no bend is given for it. Full table: <a href='scale_table.csv'>scale_table.csv</a>.</p>")
    if rib_on:
        P.append("<h3>Plate only against plate plus rib (second moment of area, ROUGH / ASSUMED)</h3><div class='scroll'><table><tr><th>Wing</th><th>Mean chord c, mm</th><th>Plate t, mm</th><th>Rib w x H, mm</th><th>Neutral axis above plate bottom, mm</th><th>I plate only = c t&sup3;/12, mm&#8308;</th><th>I with rib (T-section), mm&#8308;</th><th>Ratio</th></tr>")
        for r in printed:
            if not r["rib"]:
                continue
            tt = thickness if thickness in r["thick"] else list(r["thick"])[0]
            x = r["thick"][tt]
            yn = second_moment_T(r["g"]["mean_chord"], tt, r["rib"]["width"], r["rib"]["height"])[1]
            P.append(f"<tr><td>{r['name']}</td><td class='n'>{r['g']['mean_chord']:.1f}</td><td class='n'>{tt:g}</td><td class='n'>{r['rib']['width']:g} x {r['rib']['height']:g}</td><td class='n'>{yn:.2f}</td><td class='n'>{x['I_plate']:.2f}</td><td class='n'>{x['I_spec']:.1f}</td><td class='n'>{x['I_spec'] / x['I_plate']:.0f}x</td></tr>")
        P.append("</table></div><p class='note'>ARITHMETIC with ASSUMED inputs. T-section: area A1 = c x t (centre t/2), A2 = w x H (centre t + H/2), neutral axis y = (A1 y1 + A2 y2)/(A1 + A2), I = sum of (own I + A d&sup2;). It treats the rib as running along the whole half span at the mean chord; the real rib is slanted and stops 3 mm short of the tip, so the real gain is smaller. The wing's tip bend scales as 1/I.</p>")
    for r in foam:
        P.append(f"<h3>{r['name']}: this craft can be ballasted over a wide range of wing loadings</h3><p>The printed ones cannot. Wing loading, speed and bob period against all-up mass for this craft (ARITHMETIC; use your measured mass):</p>")
        P.append(drop_table([r], (15, 20, 25, 30, 40)))
    P.append("</section>")
    P.append("<section><h2>8. Launch: drone drop</h2><p>The launch method is a drop from a 5-inch FPV quad at an RC field. This is a procedure, not a prediction. We do not claim either release method below works.</p><ol>"
             "<li><b>Hang the glider level</b> from a thread loop round the stick and over the wing root, at the balance point (the CG you measured). The 5 mm marks on the stick are wider notches that seat the loop, so you can put it back in the same place; use the nearest one, and for finer positions the nose hole and the ballast holes.</li>"
             "<li><b>Two ways to release it.</b> (1) <b>Open wire hook and a jerk</b>: the cheapest, try it first. The quad releases the loop by a sharp acceleration or deceleration. The release attitude and speed will vary from drop to drop, so write down what the quad was doing (in notes). (2) <b>Pin release</b>: pulling a pin frees the loop. More repeatable; for later.</li>"
             "<li><b>Release in forward flight</b> at roughly the glide speed for the wing loading (tables below), <b>not from a hover</b> (the propeller wash would hit the wing).</li>"
             f"<li><b>Same height every time</b>: {DROP_HEIGHT_M:g} m suggested (JUDGEMENT). Calm air. Do a <b>dummy drop</b> first to check the glider clears the props. Use bright tape or colour so you can find it.</li>"
             "<li><b>Why height matters</b> (ARITHMETIC): a glider that is out of trim bobs up and down slowly. The period of that bob (phugoid) is about T = pi x sqrt(2) x V / g, which is about 0.45 x V seconds. A glider at 7 m/s bobs about every 3 seconds. You need several seconds of flight to see whether it settles or not.</li>"
             "<li><b>Record from ground video</b>: time to the ground, distance, heading change (write it in the notes) and the outcome. Also write the release height and the quad's speed at release.</li>"
             f"<li>Start at the first CG setting for the wing and drop {RELEASES_PER_SETTING} times (JUDGEMENT). If it glides at that setting, drop {RELEASES_AT_GLIDING_SETTING} times in total. Move the CG {CG_STEP_MM:g} mm aft. Repeat, following the stop rule in section 6 (the sweep does not end at the neutral point).</li></ol>")
    P.append(drop_table(printed, (10, 12, 14, 16, 18, 20)) if printed else "")
    P.append(f"<p class='note'>ARITHMETIC. Speed = sqrt(2 x weight / (air density x wing area x CL)), air density {RHO_AIR} (ASSUMED). Use <b>your measured mass</b> (weigh the finished glider with its ballast). CL 0.3 is the faster case, CL 0.5 the slower. These speeds say what the quad should do at release. They do not say the glider will fly at that CL. Bob period T = pi x sqrt(2) x V / g (textbook estimate for a glider; ARITHMETIC). The foamboard wing's table is in section 7.</p>")
    P.append("<p><b>Hand launch is for gross trim only.</b> One or two seconds of flight sorts a dive from a pitch-up, so it is fine for finding roughly where to start. It cannot show a repeatable glide. Do not mix hand and drone results: the log has a launch_method column and the results page keeps them apart.</p></section>")
    P.append("<section><h2>9. Record</h2><p>Use <a href='log_template.csv'>log_template.csv</a>. One row per drop. Columns: date, wing (F45, F60, B45 or F45-plain), thickness_mm, config (for example &quot;rib-down, tip fins&quot;), measured_mass_g, measured_cg_mm, launch_method (free text, for example drone drop, open hook; drone drop, pin; hand), release_no, outcome, release_height_m, release_speed_mps, distance_m, time_s, notes. Rows whose wing starts with EXAMPLE are ignored. Heading change from the video goes in notes.</p>"
             "<p>Outcome must be exactly one of: " + ", ".join(f"<b>{o}</b>" for o in OUTCOMES) + ". <b>pitch oscillation</b> = repeated swooping or bounding, nose up and down; <b>tumble</b> = flips end over end. (The old word &quot;oscillation&quot; is not used any more: the report script flags it and asks for &quot;pitch oscillation&quot; or &quot;yaw wander&quot;.) For steady glides also write distance and time to the ground (from the video).</p>"
             "<p>Make the results page with <code>py -B rigid_glider/flat_test_report.py your_log.csv</code>. It writes <code>results/index.html</code> with counts per CG setting for each wing, config and launch method. It does not compare with the model.</p>"
             f"<p><b>Proposed pass rule</b> (PROPOSED, to be agreed before flying): at least {PASS_STEADY_OF_10} of {RELEASES_AT_GLIDING_SETTING} releases are steady glides at one setting, and both settings 1 mm either side also glide (at least half of their releases are steady glides).</p></section>")
    P.append("<section><h2>10. What each outcome would mean</h2><p>These are conditional readings, not predictions.</p><ul>"
             "<li>No setting glides: this counts only after the stop rule has been followed past the AVL neutral point. If it still holds, a clean flat wing of this size does not trim itself on its own at the settings tried. Try a lip next; that is a different test.</li>"
             "<li>Gliding at only one setting: the usable balance range is narrower than the 1 mm steps. Repeat with finer steps (0.5 mm) before concluding.</li>"
             "<li>Glides over several settings in a row: a range exists at this size. Its width and position are measured; compare them with the AVL neutral point.</li>"
             "<li>Going aft you see dive, then glide, then pitch oscillation, then tumble: that is the order reported in the published plate study (Li et al. 2022, unswept plates). Seeing it would be the same pattern; it would not say our wings behave like those plates in other ways.</li>"
             "<li>Pitch-up or stall at settings behind the neutral point: on a swept, tapered wing the tips may let go first as the nose rises, which would show as pitch-up rather than glide. That is unknown. Check twist and flatness too.</li>"
             "<li>Pitch oscillation (repeated bounding, nose up and down): this is about the balance point and the wing, not the fins. Do not blame the fins for it.</li>"
             "<li>Tumble (flips end over end): in the published plate study it appeared with the centre of mass furthest aft. Move the CG forward and see whether it settles.</li>"
             "<li>Dive at every setting you can reach, with the stop rule followed as far as the bare-aircraft balance point: the wing may need more nose-up trim or a different build. Check the aircraft really balances where you think.</li>"
             "<li>Pitch-up or stall at every setting: the CG may be too far aft, or the wing is twisted or bent. Check flatness.</li>"
             "<li>Roll-off to one side every time: check twist, fin alignment and flatness before blaming the shape.</li>"
             "<li>Yaw wander (the nose swings side to side): the fins may be too small or too soft. Compare the thinner and thicker wing.</li>"
             "<li>F45 works and F60 does not: slimmer costs repeatability, at this thickness and size, with this rib.</li>"
             "<li>Both printed wings work: slimmer did not cost anything in this test; the numbers (range width, steady fraction) still matter.</li>"
             "<li>Neither printed wing works: say so, and do not read it as a verdict on all flat wings; thickness, rib, print quality and launch were also fixed.</li>"
             "<li>A printed wing glides, and you want to know whether the rib matters: fly the optional plain wing (F45-plain) the same way. The published plate result is for a plain plate. If the plain wing behaves differently from the ribbed F45, the rib changes the behaviour; until then a difference from published plain-plate tests may partly come from the rib.</li>"
             "<li>The foamboard wing glides (or does not): that says something about foamboard at 400 mm and about 5 mm thick. It does not say anything about the printed wings, because size, thickness and material all differ.</li></ul></section>")
    P.append("<section><h2>11. All assumptions</h2><div class='scroll'><table><tr><th>Item</th><th>Value</th><th>Source tag</th><th>What it means</th><th>How to replace with a measurement</th></tr>")
    for it, val, unit, src, mean, repl in rows_assump:
        P.append(f"<tr><td>{h(str(it))}</td><td>{h(str(val))} {h(unit)}</td><td>{h(src)}</td><td>{h(mean)}</td><td>{h(repl)}</td></tr>")
    P.append("</table></div><p>Same table in <a href='assumptions.csv'>assumptions.csv</a>.</p></section>")
    P.append("<section><h2>12. How to regenerate</h2><p><code>py -B rigid_glider/flat_test.py</code> from the Codes folder. Parameters are at the top of <code>flat_test.py</code>: change span, aspect ratios, thickness, rib size, fin size, clearances, foamboard thickness and areal mass. Options: <code>--span</code>, <code>--ar</code>, <code>--thickness</code>, <code>--plain</code> (no rib on the printed wings), <code>--no-foam</code>, <code>--no-plain-extra</code> (skip the optional plain F45 wing), <code>--out</code>. Running it again overwrites the output folder. The spec is in <code>rigid_glider/FLAT_TEST.md</code>.</p></section>")
    P.append("</main></body></html>")
    (outdir / "index.html").write_text("\n".join(P), encoding="utf-8")


# =============================================================================================== main
def build(outdir: Path = OUT_DEFAULT, span=SPAN_MM, ars=None, thickness=THICKNESS_MM, plain=False, avl=True, foam=True, plain_extra=None):
    ars = ars or AR_LIST
    outdir.mkdir(parents=True, exist_ok=True)
    thick_set = sorted(set(THICKNESS_TABLE_MM) | {thickness})
    saved = list(THICKNESS_TABLE_MM)
    THICKNESS_TABLE_MM[:] = thick_set
    try:
        results = [build_wing(span, ar, thickness, outdir, not plain, avl) for ar in ars]
    finally:
        THICKNESS_TABLE_MM[:] = saved
    if foam:
        results.append(build_wing(FOAM_SPAN_MM, FOAM_AR, FOAM_THICKNESS_MM, outdir, False, avl, foam=True))
    if (PLAIN_EXTRA if plain_extra is None else plain_extra) and not plain:
        THICKNESS_TABLE_MM[:] = thick_set
        try:
            results.append(build_wing(span, PLAIN_EXTRA_AR, thickness, outdir, False, avl, optional=True))
        finally:
            THICKNESS_TABLE_MM[:] = saved
    write_top_csvs(outdir, results, thickness)
    rows = write_assumptions(outdir, results)
    index_html(results, thickness, rows, outdir)
    return results


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--span", type=float, default=SPAN_MM)
    ap.add_argument("--ar", type=float, nargs="+", default=AR_LIST)
    ap.add_argument("--thickness", type=float, default=THICKNESS_MM)
    ap.add_argument("--plain", action="store_true", help="printed wings WITHOUT the spar rib (not the default)")
    ap.add_argument("--no-foam", action="store_true", help="skip the foamboard specimen")
    ap.add_argument("--no-plain-extra", action="store_true", help="skip the OPTIONAL plain (no rib) F45 wing")
    ap.add_argument("--out", type=Path, default=OUT_DEFAULT)
    ap.add_argument("--no-avl", action="store_true", help="skip AVL (neutral point / CG sweep then missing)")
    a = ap.parse_args()
    res = build(a.out, a.span, a.ar, a.thickness, a.plain, not a.no_avl, not a.no_foam, not a.no_plain_extra)
    for r in res:
        print(r["name"], r["folder"], "NP(fins)", r["np_ref"], "settings", r["settings"])


if __name__ == "__main__":
    main()
