"""Sheet builder: a proposed flat-sheet glider outline (DXF / SVG / built-in) + a JSON settings file -> a printable kit
(wing halves with a raised leading-edge bead, a root joint strip carrying jigsaw keys, a rail with printed dihedral faces and a printed tail-bend face,
ballast pocket, optional fin with a trim-tab family) and a foam-build kit from the same geometry.

Conventions (project): mm, x aft, y spanwise (right +), z up.  Tags on numbers: MODEL / ASSUMED / CALCULATED / GEOMETRY / KAI.
Python stdlib only.   py -B rigid_glider/sheet_builder.py rigid_glider/sheet_builder_configs/swiftA_400.json [--out DIR] [--force]
Imports (read only): bird_outline_v4_swift, flat_test, bird_outline_v2.  Writes ONLY into results/runs/sheet_builder/<config name>/ (refuses to overwrite).

Frames
  planform  : x aft, y right, as in the input outline; the RIGHT half is the master, the left half is its mirror image.
  face frame: (X, S, N) on a plate: X along the plate, S across it away from the centreline, N along the face normal (the strip bottom sits on N = 0).
  half-solid: (x, y, h): h = height above the FLAT face of the sheet.  In flight the flat face is the TOP of the wing and h grows downward
              (bead, strip and keys hang under the sheet, toward the rail).  The printed part is the half-solid rotated 180 deg about x
              (flat face on the bed, bead/strip/keys up), so the printed right-wing half has the mirrored planform.
"""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import argparse
import copy
import html
import json
import math
import re
import struct
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))

EPS = 1e-9
SNAP = 1e-6


# =============================================================================================== polygon helpers
def snap(p):
    return (round(p[0] / SNAP) * SNAP + 0.0, round(p[1] / SNAP) * SNAP + 0.0)


def area2(poly):
    return sum(poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1] for i in range(len(poly)))


def parea(poly):
    return 0.5 * area2(poly)


def ccw(poly):
    return list(poly) if area2(poly) > 0 else list(reversed(poly))


def cw(poly):
    return list(poly) if area2(poly) < 0 else list(reversed(poly))


def centroid(poly):
    a = area2(poly)
    cx = sum((poly[i][0] + poly[(i + 1) % len(poly)][0]) * (poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1]) for i in range(len(poly))) / (3 * a)
    cy = sum((poly[i][1] + poly[(i + 1) % len(poly)][1]) * (poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1]) for i in range(len(poly))) / (3 * a)
    return cx, cy


def dedupe(poly, tol=1e-7):
    out = []
    for p in poly:
        if not out or math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > tol:
            out.append(p)
    while len(out) > 1 and math.hypot(out[0][0] - out[-1][0], out[0][1] - out[-1][1]) <= tol:
        out.pop()
    return out


def seg_dist(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 < 1e-18 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy), t


def point_in_poly(p, poly):
    """Even-odd ray test (use on interior points only)."""
    x, y = p
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def on_boundary(p, poly, tol=1e-6):
    """-> (edge index, t) if p lies on the polygon boundary else None."""
    n = len(poly)
    for i in range(n):
        d, t = seg_dist(p, poly[i], poly[(i + 1) % n])
        if d <= tol:
            return i, t
    return None


def bbox2(pts):
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return min(xs), min(ys), max(xs), max(ys)


def polyline_len(pts):
    return sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]) for i in range(len(pts) - 1))


def poly_min_dist(pa, pb):
    """Minimum distance between two closed polylines (vertex-to-segment both ways; exact for non-crossing polylines)."""
    best = 1e30
    for A, B in ((pa, pb), (pb, pa)):
        nb = len(B)
        for p in A:
            for i in range(nb):
                d, _ = seg_dist(p, B[i], B[(i + 1) % nb])
                if d < best:
                    best = d
    return best


# =============================================================================================== triangulation (ear clipping, polygon with holes)
def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _seg_cross_proper(p, q, a, b, eps=1e-9):
    """True if segments pq and ab cross at a point interior to both (touching at ends does not count)."""
    d1 = _cross(p, q, a)
    d2 = _cross(p, q, b)
    d3 = _cross(a, b, p)
    d4 = _cross(a, b, q)
    return ((d1 > eps and d2 < -eps) or (d1 < -eps and d2 > eps)) and ((d3 > eps and d4 < -eps) or (d3 < -eps and d4 > eps))


def _bridge_holes(outer, holes):
    """Merge CW holes into the CCW outer ring by zero-width bridges.  For each hole (rightmost first) the bridge goes from its rightmost vertex to the nearest ring vertex
    whose connecting segment crosses no edge of the ring or of any hole and runs through material."""
    ring = list(outer)
    order = sorted(range(len(holes)), key=lambda i: -max(p[0] for p in holes[i]))
    pending = set(order)
    for hi in order:
        hole = holes[hi]
        pending.discard(hi)
        mi = max(range(len(hole)), key=lambda i: (hole[i][0], -hole[i][1]))
        M = hole[mi]
        others = [holes[j] for j in pending]
        edges = [(ring[i], ring[(i + 1) % len(ring)]) for i in range(len(ring))]
        for hl in [hole] + others:
            edges += [(hl[i], hl[(i + 1) % len(hl)]) for i in range(len(hl))]
        cands = sorted(range(len(ring)), key=lambda k: (ring[k][0] - M[0]) ** 2 + (ring[k][1] - M[1]) ** 2)
        target = None
        for k in cands:
            q = ring[k]
            ok = True
            for a_, b_ in edges:
                if _seg_cross_proper(M, q, a_, b_):
                    ok = False
                    break
            if not ok:
                continue
            mid = ((M[0] + q[0]) / 2, (M[1] + q[1]) / 2)
            if not point_in_poly(mid, ring) or point_in_poly(mid, hole) or any(point_in_poly(mid, o) for o in others):
                continue
            target = k
            break
        if target is None:
            raise ValueError("hole bridge: no visible ring vertex")
        hole_ring = hole[mi:] + hole[:mi]
        ring = ring[:target + 1] + hole_ring + [hole_ring[0]] + [ring[target]] + ring[target + 1:]
    return ring


def triangulate(outer, holes=()):
    """Triangles (point triples, CCW) of a CCW outer ring with CW holes.  Ear clipping; raises ValueError when it cannot finish."""
    outer = ccw(outer)
    holes = [cw(h) for h in holes]
    ring = _bridge_holes(outer, holes) if holes else list(outer)
    idx = list(range(len(ring)))
    tris = []
    while len(idx) > 3:
        n = len(idx)
        found = False
        for relax in (0, 1):
            for k in range(n):
                i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % n]
                a, b, c = ring[i0], ring[i1], ring[i2]
                cr = _cross(a, b, c)
                if cr <= 1e-12:
                    continue
                ok = True
                if not relax:
                    for j in idx:
                        if j in (i0, i1, i2):
                            continue
                        p = ring[j]
                        if p == a or p == b or p == c:
                            continue
                        if _cross(a, b, p) >= -1e-12 and _cross(b, c, p) >= -1e-12 and _cross(c, a, p) >= -1e-12:
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
            if abs(area2([ring[i] for i in idx])) < 1e-9:          # only zero-width bridge leftovers remain
                return tris
            raise ValueError("ear clipping failed (%d vertices left)" % len(idx))
    if len(idx) == 3:
        a, b, c = ring[idx[0]], ring[idx[1]], ring[idx[2]]
        if _cross(a, b, c) > 1e-12:
            tris.append((a, b, c))
    return tris


def rep_point(poly, holes=()):
    """A point strictly inside the polygon (centroid of its largest ear triangle)."""
    tr = triangulate(poly, holes)
    t = max(tr, key=lambda t: abs(_cross(*t)))
    return ((t[0][0] + t[1][0] + t[2][0]) / 3, (t[0][1] + t[1][1] + t[2][1]) / 3)


# =============================================================================================== splitting polygons
def _insert_point(poly, p, tol=1e-6):
    for i, q in enumerate(poly):
        if math.hypot(q[0] - p[0], q[1] - p[1]) <= tol:
            return poly, i
    hit = on_boundary(p, poly, tol)
    if hit is None:
        raise ValueError("point not on polygon boundary: %r" % (p,))
    i, _ = hit
    return poly[:i + 1] + [p] + poly[i + 1:], i + 1


def split_by_polyline(poly, chain):
    """Split a CCW polygon by a polyline whose first and last points lie on the boundary.  Returns [poly1, poly2] (both CCW)."""
    A, B = chain[0], chain[-1]
    poly, _ = _insert_point(list(poly), A)
    poly, _ = _insert_point(poly, B)
    ia = next(i for i, q in enumerate(poly) if math.hypot(q[0] - A[0], q[1] - A[1]) <= 1e-6)
    ib = next(i for i, q in enumerate(poly) if math.hypot(q[0] - B[0], q[1] - B[1]) <= 1e-6)
    n = len(poly)

    def walk(i, j):
        out = [poly[i]]
        while i != j:
            i = (i + 1) % n
            out.append(poly[i])
        return out
    inner = list(chain[1:-1])
    p1 = walk(ia, ib) + list(reversed(inner))
    p2 = walk(ib, ia) + inner
    return [dedupe(ccw(p1)), dedupe(ccw(p2))]


def split_by_line(polys, axis, value):
    """Split every polygon in the list by the line coord[axis] = value.  Pieces keep CCW order."""
    out = []
    for poly in polys:
        poly = list(poly)
        vs = [p[axis] for p in poly]
        if min(vs) >= value - 1e-9 or max(vs) <= value + 1e-9:
            out.append(poly)
            continue
        v = value
        if any(abs(p[axis] - v) < 1e-7 for p in poly):
            v = value + 3e-7                                   # nudge off exact vertex hits (3e-7 mm, far below any print tolerance)
        pts = []
        n = len(poly)
        for i in range(n):
            a, b = poly[i], poly[(i + 1) % n]
            if (a[axis] - v) * (b[axis] - v) < 0:
                t = (v - a[axis]) / (b[axis] - a[axis])
                q = [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]
                q[axis] = v
                pts.append(snap(tuple(q)))
        o = 1 - axis
        pts.sort(key=lambda q: q[o])
        if len(pts) % 2:
            raise ValueError("odd number of line crossings")
        cur = [poly]
        for k in range(0, len(pts), 2):
            A, B = pts[k], pts[k + 1]
            mid = ((A[0] + B[0]) / 2, (A[1] + B[1]) / 2)
            done = False
            for ci, P in enumerate(cur):
                if on_boundary(A, P) is not None and on_boundary(B, P) is not None and point_in_poly(mid, P):
                    cur[ci:ci + 1] = split_by_polyline(P, [A, B])
                    done = True
                    break
            if not done:
                raise ValueError("chord split failed")
        out.extend(cur)
    return out


# =============================================================================================== region mesher (2.5D solids with vertical walls)
def _fn(v):
    return v if callable(v) else (lambda x, y, _v=v: _v)


def mesh_regions(regions):
    """regions: list of dict(outer=[(x,y)], holes=[[...]], zb=number|fn(x,y), zt=number|fn(x,y)).
    Returns a closed, consistently oriented triangle list (outward normals).  Adjacent regions must share boundaries; T-junctions are repaired here.
    Walls are vertical between neighbouring regions wherever their top (or bottom) heights differ."""
    R = []
    for r in regions:
        outer = ccw([snap(p) for p in dedupe(r["outer"])])
        holes = [cw([snap(p) for p in dedupe(h)]) for h in r.get("holes", [])]
        R.append(dict(loops=[outer] + holes, zb=_fn(r["zb"]), zt=_fn(r["zt"])))
    cell = 2.0
    grid = {}
    for r in R:
        for lp in r["loops"]:
            for p in lp:
                grid.setdefault((int(math.floor(p[0] / cell)), int(math.floor(p[1] / cell))), set()).add(p)
    for r in R:
        newloops = []
        for lp in r["loops"]:
            out = []
            n = len(lp)
            for i in range(n):
                a, b = lp[i], lp[(i + 1) % n]
                out.append(a)
                x0, x1 = sorted((a[0], b[0]))
                y0, y1 = sorted((a[1], b[1]))
                ins = []
                for gx in range(int(math.floor((x0 - 1e-4) / cell)), int(math.floor((x1 + 1e-4) / cell)) + 1):
                    for gy in range(int(math.floor((y0 - 1e-4) / cell)), int(math.floor((y1 + 1e-4) / cell)) + 1):
                        for q in grid.get((gx, gy), ()):
                            if q == a or q == b:
                                continue
                            d, t = seg_dist(q, a, b)
                            if d <= 2e-5 and 1e-9 < t < 1 - 1e-9:
                                ins.append((t, q))
                ins.sort()
                last = None
                for t, q in ins:
                    if q != last:
                        out.append(q)
                        last = q
            newloops.append(out)
        r["loops"] = newloops
    emap = {}
    for ri, r in enumerate(R):
        for lp in r["loops"]:
            n = len(lp)
            for i in range(n):
                emap[(lp[i], lp[(i + 1) % n])] = ri
    tris = []
    for ri, r in enumerate(R):
        zt, zb = r["zt"], r["zb"]
        for t in triangulate(r["loops"][0], r["loops"][1:]):
            a, b, c = t
            tris.append(((a[0], a[1], zt(*a)), (b[0], b[1], zt(*b)), (c[0], c[1], zt(*c))))
            tris.append(((a[0], a[1], zb(*a)), (c[0], c[1], zb(*c)), (b[0], b[1], zb(*b))))

    levels = {}
    for r in R:
        for lp in r["loops"]:
            for v in lp:
                s = levels.setdefault(v, set())
                s.add(round(r["zt"](*v), 9))
                s.add(round(r["zb"](*v), 9))

    def lvl(v, lo, hi):
        zs = [lo] + sorted(z for z in levels[v] if lo + 1e-7 < z < hi - 1e-7) + ([hi] if hi - lo > 1e-9 else [])
        return zs

    def wall(a, b, la, lb, ha, hb):
        # outward = right of a->b.  Vertical edges are split at every height present at their vertex so neighbouring walls share them exactly.
        if ha - la < -1e-9 or hb - lb < -1e-9:
            raise ValueError("wall heights cross")
        A = lvl(a, la, ha)
        B = lvl(b, lb, hb)
        fa = [(z - la) / (ha - la) if ha - la > 1e-9 else 0.0 for z in A]
        fb = [(z - lb) / (hb - lb) if hb - lb > 1e-9 else 0.0 for z in B]
        i = j = 0
        while i < len(A) - 1 or j < len(B) - 1:
            adv_a = j >= len(B) - 1 or (i < len(A) - 1 and fa[i + 1] <= fb[j + 1] + 1e-12)
            Ap, Bp = (a[0], a[1], A[i]), (b[0], b[1], B[j])
            if adv_a:
                tris.append((Ap, Bp, (a[0], a[1], A[i + 1])))
                i += 1
            else:
                tris.append((Ap, Bp, (b[0], b[1], B[j + 1])))
                j += 1
    for ri, r in enumerate(R):
        zt, zb = r["zt"], r["zb"]
        for lp in r["loops"]:
            n = len(lp)
            for i in range(n):
                a, b = lp[i], lp[(i + 1) % n]
                rj = emap.get((b, a))
                if rj is None:
                    wall(a, b, zb(*a), zb(*b), zt(*a), zt(*b))
                else:
                    o = R[rj]
                    ta, tb, ua, ub = zt(*a), zt(*b), o["zt"](*a), o["zt"](*b)
                    if ta - ua > 1e-9 or tb - ub > 1e-9:
                        wall(a, b, ua, ub, ta, tb)
                    ba, bb, va, vb = zb(*a), zb(*b), o["zb"](*a), o["zb"](*b)
                    if va - ba > 1e-9 or vb - bb > 1e-9:
                        wall(a, b, ba, bb, va, vb)
    return tris


# =============================================================================================== triangle-mesh tools
def mesh_volume(tris):
    v = 0.0
    for a, b, c in tris:
        v += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6
    return v


def mesh_centroid(tris):
    vol = cx = cy = cz = 0.0
    for a, b, c in tris:
        v = (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6
        vol += v
        cx += v * (a[0] + b[0] + c[0]) / 4
        cy += v * (a[1] + b[1] + c[1]) / 4
        cz += v * (a[2] + b[2] + c[2]) / 4
    return (cx / vol, cy / vol, cz / vol) if vol else (0.0, 0.0, 0.0)


def mesh_check(tris, nd=4):
    """Every undirected edge must be used by exactly two triangles, once in each direction (closed, consistently oriented); volume > 0 means outward normals."""
    key = lambda p: (round(p[0], nd) + 0.0, round(p[1], nd) + 0.0, round(p[2], nd) + 0.0)
    directed = {}
    degenerate = 0
    for t in tris:
        k = [key(p) for p in t]
        if len(set(k)) < 3:
            degenerate += 1
        for i in range(3):
            e = (k[i], k[(i + 1) % 3])
            directed[e] = directed.get(e, 0) + 1
    und = {}
    for (a, b), n in directed.items():
        f = frozenset((a, b))
        und[f] = und.get(f, 0) + n
    bad = sum(1 for n in und.values() if n != 2)
    same = sum(1 for n in directed.values() if n > 1)
    vol = mesh_volume(tris)
    return dict(n_tris=len(tris), n_edges=len(und), shared_not_2=bad, same_direction_twice=same, degenerate=degenerate, volume=vol, ok=(bad == 0 and same == 0 and degenerate == 0 and vol > 0))


def affine(tris, M, t=(0.0, 0.0, 0.0)):
    """p' = M p + t ; winding reversed when det(M) < 0 so normals stay outward."""
    det = (M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]))
    f = lambda p: (M[0][0] * p[0] + M[0][1] * p[1] + M[0][2] * p[2] + t[0], M[1][0] * p[0] + M[1][1] * p[1] + M[1][2] * p[2] + t[1], M[2][0] * p[0] + M[2][1] * p[1] + M[2][2] * p[2] + t[2])
    if det >= 0:
        return [(f(a), f(b), f(c)) for a, b, c in tris]
    return [(f(a), f(c), f(b)) for a, b, c in tris]


def tri_bbox(tris):
    pts = [p for t in tris for p in t]
    return tuple(min(p[i] for p in pts) for i in range(3)), tuple(max(p[i] for p in pts) for i in range(3))


def write_stl(path, tris, name="part"):
    with Path(path).open("wb") as f:
        f.write(name.encode("ascii", "replace")[:80].ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for a, b, c in tris:
            u = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
            v = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
            n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
            m = math.sqrt(sum(x * x for x in n)) or 1.0
            f.write(struct.pack("<12fH", n[0] / m, n[1] / m, n[2] / m, *a, *b, *c, 0))


def read_stl(path):
    data = Path(path).read_bytes()
    n = struct.unpack("<I", data[80:84])[0]
    tris = []
    for i in range(n):
        rec = struct.unpack("<12fH", data[84 + 50 * i:84 + 50 * (i + 1)])
        tris.append((tuple(rec[3:6]), tuple(rec[6:9]), tuple(rec[9:12])))
    return tris



# =============================================================================================== DXF import
def _bulge_points(p0, p1, b):
    if abs(b) < 1e-12:
        return []
    th = 4 * math.atan(b)
    c = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    if c < 1e-12:
        return []
    dx, dy = (p1[0] - p0[0]) / c, (p1[1] - p0[1]) / c
    mx, my = (p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2
    off = (c / 2) / math.tan(th / 2)
    cx, cy = mx - dy * off, my + dx * off
    r = math.hypot(p0[0] - cx, p0[1] - cy)
    a0 = math.atan2(p0[1] - cy, p0[0] - cx)
    n = max(3, int(math.ceil(abs(th) / (2 * math.pi) * 96)))
    return [(cx + r * math.cos(a0 + th * k / n), cy + r * math.sin(a0 + th * k / n)) for k in range(1, n)]


def _bspline_basis(i, p, u, t):
    """Cox-de Boor basis N_{i,p}(u)."""
    if p == 0:
        return 1.0 if (t[i] <= u < t[i + 1]) or (u == t[-1] and t[i] < t[i + 1] == t[-1]) else 0.0
    a = b = 0.0
    d1 = t[i + p] - t[i]
    d2 = t[i + p + 1] - t[i + 1]
    if d1 > 0:
        a = (u - t[i]) / d1 * _bspline_basis(i, p - 1, u, t)
    if d2 > 0:
        b = (t[i + p + 1] - u) / d2 * _bspline_basis(i + 1, p - 1, u, t)
    return a + b


def deboor(k, x, t, c, p):
    """de Boor evaluation of a degree-p B-spline with knots t, control points c at x.  k = knot span index with t[k] <= x < t[k+1]."""
    d = [c[j + k - p] for j in range(p + 1)]
    for r in range(1, p + 1):
        for j in range(p, r - 1, -1):
            den = t[j + 1 + k - r] - t[j + k - p]
            alpha = 0.0 if den == 0 else (x - t[j + k - p]) / den
            d[j] = ((1 - alpha) * d[j - 1][0] + alpha * d[j][0], (1 - alpha) * d[j - 1][1] + alpha * d[j][1])
    return d[p]


def spline_points(ctrl, knots, degree, n=None):
    """Sample a (clamped or open) B-spline from control points + knot vector with de Boor's algorithm."""
    m = len(ctrl)
    if len(knots) != m + degree + 1:
        raise ValueError("SPLINE: knot count %d != ctrl %d + degree %d + 1" % (len(knots), m, degree))
    lo, hi = knots[degree], knots[m]
    n = n or max(24, 8 * m)
    out = []
    for s in range(n + 1):
        x = lo + (hi - lo) * s / n
        if x >= hi:
            x = hi - 1e-12 * max(1.0, abs(hi))
        k = degree
        while k < m - 1 and knots[k + 1] <= x:
            k += 1
        out.append(deboor(k, x, knots, ctrl, degree))
    return out


def fit_spline_points(fit, degree=3, n=None):
    """Interpolating B-spline through fit points (chord-length parameters, averaged knots), sampled with de Boor."""
    N = len(fit)
    if N < 3:
        return list(fit)
    p = min(degree, N - 1)
    d = [0.0]
    for i in range(1, N):
        d.append(d[-1] + math.hypot(fit[i][0] - fit[i - 1][0], fit[i][1] - fit[i - 1][1]))
    L = d[-1] or 1.0
    u = [v / L for v in d]
    knots = [0.0] * (p + 1)
    for j in range(1, N - p):
        knots.append(sum(u[j:j + p]) / p)
    knots += [1.0] * (p + 1)
    A = [[_bspline_basis(j, p, u[i], knots) for j in range(N)] for i in range(N)]
    for ax in (0, 1):
        pass
    # solve A c = P for both coordinates (Gaussian elimination with partial pivoting)
    M = [A[i][:] + [fit[i][0], fit[i][1]] for i in range(N)]
    for col in range(N):
        piv = max(range(col, N), key=lambda r: abs(M[r][col]))
        if abs(M[piv][col]) < 1e-14:
            raise ValueError("SPLINE fit: singular system")
        M[col], M[piv] = M[piv], M[col]
        pv = M[col][col]
        for r in range(col + 1, N):
            f = M[r][col] / pv
            if f:
                for cc in range(col, N + 2):
                    M[r][cc] -= f * M[col][cc]
    cx = [0.0] * N
    cy = [0.0] * N
    for r in range(N - 1, -1, -1):
        sx = M[r][N] - sum(M[r][c] * cx[c] for c in range(r + 1, N))
        sy = M[r][N + 1] - sum(M[r][c] * cy[c] for c in range(r + 1, N))
        cx[r], cy[r] = sx / M[r][r], sy / M[r][r]
    return spline_points(list(zip(cx, cy)), knots, p, n or max(40, 6 * N))


def parse_dxf(text, layers=None):
    """Return dict(chains=[(points, closed)], counts={entity: n}, layers=set).  Entities: LINE ARC LWPOLYLINE POLYLINE(+VERTEX) SPLINE (CIRCLE is reported, not used)."""
    raw = text.splitlines()
    pairs = []
    for i in range(0, len(raw) - 1, 2):
        try:
            pairs.append((int(raw[i].strip()), raw[i + 1].strip()))
        except ValueError:
            continue
    ents = []
    cur = None
    in_ent = False
    for code, val in pairs:
        if code == 0 and val == "SECTION":
            cur = None
            continue
        if code == 2 and cur is None and val == "ENTITIES":
            in_ent = True
            continue
        if code == 0 and val == "ENDSEC":
            in_ent = False
            cur = None
            continue
        if not in_ent:
            continue
        if code == 0:
            cur = dict(type=val, g=[])
            ents.append(cur)
        elif cur is not None:
            cur["g"].append((code, val))
    chains, counts, seen_layers = [], {}, set()

    def get(e, c, default=None, cast=float):
        for cc, v in e["g"]:
            if cc == c:
                return cast(v)
        return default

    def getall(e, c):
        return [float(v) for cc, v in e["g"] if cc == c]
    i = 0
    while i < len(ents):
        e = ents[i]
        t = e["type"]
        lay = get(e, 8, "0", str)
        i += 1
        if t in ("VERTEX", "SEQEND"):
            continue
        if layers is not None and t != "POLYLINE" and lay not in layers:
            continue
        counts[t] = counts.get(t, 0) + 1
        seen_layers.add(lay)
        if t == "LINE":
            chains.append(([(get(e, 10), get(e, 20)), (get(e, 11), get(e, 21))], False))
        elif t == "ARC":
            cx, cy, r, a0, a1 = get(e, 10), get(e, 20), get(e, 40), get(e, 50), get(e, 51)
            sweep = (a1 - a0) % 360.0 or 360.0
            n = max(4, int(math.ceil(sweep / 360.0 * 96)))
            chains.append(([(cx + r * math.cos(math.radians(a0 + sweep * k / n)), cy + r * math.sin(math.radians(a0 + sweep * k / n))) for k in range(n + 1)], False))
        elif t == "LWPOLYLINE":
            flag = get(e, 70, 0, lambda v: int(float(v)))
            verts = []
            for cc, v in e["g"]:
                if cc == 10:
                    verts.append([float(v), 0.0, 0.0])
                elif cc == 20 and verts:
                    verts[-1][1] = float(v)
                elif cc == 42 and verts:
                    verts[-1][2] = float(v)
            closed = bool(flag & 1)
            pts = []
            nv = len(verts)
            for k in range(nv if closed else nv - 1):
                a, b = verts[k], verts[(k + 1) % nv]
                pts.append((a[0], a[1]))
                pts += _bulge_points((a[0], a[1]), (b[0], b[1]), a[2])
            if not closed and nv:
                pts.append((verts[-1][0], verts[-1][1]))
            chains.append((pts, closed))
        elif t == "POLYLINE":
            if layers is not None and lay not in layers:
                while i < len(ents) and ents[i]["type"] != "SEQEND":
                    i += 1
                continue
            flag = get(e, 70, 0, lambda v: int(float(v)))
            verts = []
            while i < len(ents) and ents[i]["type"] == "VERTEX":
                v = ents[i]
                verts.append([get(v, 10), get(v, 20), get(v, 42, 0.0)])
                i += 1
            closed = bool(flag & 1)
            nv = len(verts)
            pts = []
            for k in range(nv if closed else nv - 1):
                a, b = verts[k], verts[(k + 1) % nv]
                pts.append((a[0], a[1]))
                pts += _bulge_points((a[0], a[1]), (b[0], b[1]), a[2])
            if not closed and nv:
                pts.append((verts[-1][0], verts[-1][1]))
            chains.append((pts, closed))
        elif t == "SPLINE":
            deg = get(e, 71, 3, lambda v: int(float(v)))
            knots = getall(e, 40)
            xs, ys = getall(e, 10), getall(e, 20)
            fx, fy = getall(e, 11), getall(e, 21)
            flag = get(e, 70, 0, lambda v: int(float(v)))
            if xs and len(knots) == len(xs) + deg + 1:
                pts = spline_points(list(zip(xs, ys)), knots, deg)
            elif fx:
                pts = fit_spline_points(list(zip(fx, fy)), deg)
            elif xs:
                pts = list(zip(xs, ys))
            else:
                raise ValueError("SPLINE without points")
            chains.append((pts, bool(flag & 1)))
    return dict(chains=chains, counts=counts, layers=seen_layers)


def assemble_loops(chains, tol=0.05):
    """Join chains end to end (within tol) into closed loops.  Returns (loops, leftovers, max_join_gap)."""
    pool = [(list(p), c) for p, c in chains if len(p) >= 2 or c]
    loops, left, maxgap = [], [], 0.0
    pool.sort(key=lambda pc: -len(pc[0]))
    used = [False] * len(pool)
    for si in range(len(pool)):
        if used[si]:
            continue
        used[si] = True
        pts, closed = pool[si]
        if closed:
            loops.append(dedupe(pts, 1e-9))
            continue
        pts = list(pts)
        while True:
            if math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]) <= tol and len(pts) > 2:
                break
            best = None
            for j in range(len(pool)):
                if used[j] or pool[j][1]:
                    continue
                q = pool[j][0]
                for rev in (False, True):
                    a = q[-1] if rev else q[0]
                    d = math.hypot(a[0] - pts[-1][0], a[1] - pts[-1][1])
                    if d <= tol and (best is None or d < best[0]):
                        best = (d, j, rev)
            if best is None:
                break
            d, j, rev = best
            used[j] = True
            q = list(reversed(pool[j][0])) if rev else pool[j][0]
            maxgap = max(maxgap, d)
            pts += q[1:]
        g = math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1])
        if g <= tol and len(pts) > 2:
            maxgap = max(maxgap, g)
            loops.append(dedupe(pts, 1e-9))
        else:
            left.append(pts)
    return loops, left, maxgap


# =============================================================================================== SVG import
def _mat_mul(a, b):
    return (a[0] * b[0] + a[2] * b[1], a[1] * b[0] + a[3] * b[1], a[0] * b[2] + a[2] * b[3], a[1] * b[2] + a[3] * b[3], a[0] * b[4] + a[2] * b[5] + a[4], a[1] * b[4] + a[3] * b[5] + a[5])


def parse_transform(s):
    """SVG transform list -> matrix (a b c d e f).  translate, scale, rotate, matrix (skew not supported)."""
    m = (1.0, 0.0, 0.0, 1.0, 0.0, 0.0)
    for name, args in re.findall(r"([a-zA-Z]+)\s*\(([^)]*)\)", s or ""):
        v = [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", args)]
        if name == "translate":
            t = (1, 0, 0, 1, v[0], v[1] if len(v) > 1 else 0.0)
        elif name == "scale":
            t = (v[0], 0, 0, v[1] if len(v) > 1 else v[0], 0, 0)
        elif name == "rotate":
            a = math.radians(v[0])
            c, sn = math.cos(a), math.sin(a)
            t = (c, sn, -sn, c, 0, 0)
            if len(v) == 3:
                t = _mat_mul(_mat_mul((1, 0, 0, 1, v[1], v[2]), t), (1, 0, 0, 1, -v[1], -v[2]))
        elif name == "matrix":
            t = tuple(v[:6])
        else:
            raise ValueError("SVG transform '%s' not supported" % name)
        m = _mat_mul(m, t)
    return m


def _flatten_path(d):
    toks = re.findall(r"[MmLlHhVvCcQqZzSsTtAa]|[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", d)
    i = 0
    subs = []
    cur = None
    cx = cy = sx = sy = 0.0
    cmd = None
    nargs = {"M": 2, "L": 2, "H": 1, "V": 1, "C": 6, "Q": 4, "Z": 0}

    def num():
        nonlocal i
        v = float(toks[i])
        i += 1
        return v
    while i < len(toks):
        if re.match(r"[A-Za-z]", toks[i]):
            cmd = toks[i]
            i += 1
            if cmd in "SsTtAa":
                raise ValueError("SVG path command %s not supported (M L H V C Q Z only)" % cmd)
            if cmd in "Zz":
                if cur is not None:
                    subs.append((cur, True))
                    cur = None
                cx, cy = sx, sy
                continue
        elif cmd is None:
            raise ValueError("bad path data")
        up = cmd.upper()
        rel = cmd.islower()
        if up == "M":
            x, y = num(), num()
            if rel:
                x, y = cx + x, cy + y
            if cur is not None:
                subs.append((cur, False))
            cur = [(x, y)]
            cx, cy, sx, sy = x, y, x, y
            cmd = "l" if rel else "L"
        elif up == "L":
            x, y = num(), num()
            if rel:
                x, y = cx + x, cy + y
            cur.append((x, y))
            cx, cy = x, y
        elif up == "H":
            x = num()
            x = cx + x if rel else x
            cur.append((x, cy))
            cx = x
        elif up == "V":
            y = num()
            y = cy + y if rel else y
            cur.append((cx, y))
            cy = y
        elif up == "C":
            v = [num() for _ in range(6)]
            if rel:
                v = [cx + v[0], cy + v[1], cx + v[2], cy + v[3], cx + v[4], cy + v[5]]
            L = math.hypot(v[0] - cx, v[1] - cy) + math.hypot(v[2] - v[0], v[3] - v[1]) + math.hypot(v[4] - v[2], v[5] - v[3])
            n = max(8, min(96, int(L / 0.7)))
            for k in range(1, n + 1):
                t = k / n
                a, b, c, dd = (1 - t) ** 3, 3 * (1 - t) ** 2 * t, 3 * (1 - t) * t * t, t ** 3
                cur.append((a * cx + b * v[0] + c * v[2] + dd * v[4], a * cy + b * v[1] + c * v[3] + dd * v[5]))
            cx, cy = v[4], v[5]
        elif up == "Q":
            v = [num() for _ in range(4)]
            if rel:
                v = [cx + v[0], cy + v[1], cx + v[2], cy + v[3]]
            L = math.hypot(v[0] - cx, v[1] - cy) + math.hypot(v[2] - v[0], v[3] - v[1])
            n = max(8, min(96, int(L / 0.7)))
            for k in range(1, n + 1):
                t = k / n
                a, b, c = (1 - t) ** 2, 2 * (1 - t) * t, t * t
                cur.append((a * cx + b * v[0] + c * v[2], a * cy + b * v[1] + c * v[3]))
            cx, cy = v[2], v[3]
        else:
            raise ValueError("path command %s" % cmd)
    if cur is not None:
        subs.append((cur, False))
    return subs


def parse_svg(text, mm_per_unit=None):
    """-> dict(chains=[(pts, closed)] in mm (SVG y still pointing DOWN), counts).  Paths (M L H V C Q Z abs+rel), polygon, polyline; translate/scale/rotate/matrix transforms."""
    import xml.etree.ElementTree as ET
    root = ET.fromstring(text)
    strip = lambda t: t.split("}")[-1]
    scale = mm_per_unit
    if scale is None:
        scale = 1.0
        w = root.get("width")
        vb = root.get("viewBox")
        if w and vb:
            mu = re.match(r"\s*([-+]?\d*\.?\d+)\s*(mm|cm|in|px|pt)?", w)
            if mu:
                k = {"mm": 1.0, "cm": 10.0, "in": 25.4, "px": 25.4 / 96, "pt": 25.4 / 72, None: 1.0}[mu.group(2)]
                vbw = float(re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", vb)[2])
                scale = float(mu.group(1)) * k / vbw
    chains, counts = [], {}

    def walk(el, mat):
        m = _mat_mul(mat, parse_transform(el.get("transform")))
        tag = strip(el.tag)
        subs = []
        if tag == "path" and el.get("d"):
            subs = _flatten_path(el.get("d"))
        elif tag in ("polygon", "polyline") and el.get("points"):
            v = [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", el.get("points"))]
            subs = [(list(zip(v[0::2], v[1::2])), tag == "polygon")]
        if subs:
            counts[tag] = counts.get(tag, 0) + 1
        for pts, closed in subs:
            tp = [((m[0] * x + m[2] * y + m[4]) * scale, (m[1] * x + m[3] * y + m[5]) * scale) for x, y in pts]
            if len(tp) > 2 and math.hypot(tp[0][0] - tp[-1][0], tp[0][1] - tp[-1][1]) < 1e-9:
                closed = True
            chains.append((tp, closed))
        for ch in el:
            walk(ch, m)
    walk(root, (1.0, 0.0, 0.0, 1.0, 0.0, 0.0))
    return dict(chains=chains, counts=counts, scale_mm_per_unit=scale)


# =============================================================================================== DXF / SVG export
def dxf_text(polylines, layers=None, closed_default=True):
    """polylines: list of (layer, points, closed).  LINE entities only (what the project's existing parts.dxf uses; every CAD importer reads them)."""
    layers = layers or sorted({l for l, _, _ in polylines})
    ent = []
    for layer, pts, closed in polylines:
        n = len(pts)
        rng = range(n) if closed else range(n - 1)
        for i in rng:
            a, b = pts[i], pts[(i + 1) % n]
            ent.append("0\nLINE\n8\n%s\n10\n%.5f\n20\n%.5f\n30\n0.0\n11\n%.5f\n21\n%.5f\n31\n0.0\n" % (layer, a[0], a[1], b[0], b[1]))
    cols = {"OUTLINE": 7, "CUT": 7}
    lay = "".join("0\nLAYER\n2\n%s\n70\n0\n62\n%d\n6\nCONTINUOUS\n" % (l, cols.get(l, 3 + (hash(l) % 5))) for l in layers)
    return ("0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n0\nSECTION\n2\nTABLES\n0\nTABLE\n2\nLAYER\n70\n%d\n" % len(layers)
            + lay + "0\nENDTAB\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n" + "".join(ent) + "0\nENDSEC\n0\nEOF\n")


def svg_outline_text(pts, w=None, h=None, flip_y=True):
    """Single closed outline as an SVG path in mm (y flipped by default, like every SVG editor)."""
    xs = [p[0] for p in pts]
    ys = [(-p[1] if flip_y else p[1]) for p in pts]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    d = "M " + " L ".join("%.4f,%.4f" % (x, y) for x, y in zip(xs, ys)) + " Z"
    return ('<?xml version="1.0" encoding="UTF-8"?>\n<svg xmlns="http://www.w3.org/2000/svg" width="%.3fmm" height="%.3fmm" viewBox="%.3f %.3f %.3f %.3f">\n'
            '<path d="%s" fill="none" stroke="#000" stroke-width="0.2"/>\n</svg>\n' % (x1 - x0, y1 - y0, x0, y0, x1 - x0, y1 - y0, d))


# =============================================================================================== outline loading
def _rot_flip(pts, rotate_deg=0.0, flip_x=False, flip_y=False):
    a = math.radians(rotate_deg)
    c, s = math.cos(a), math.sin(a)
    out = []
    for x, y in pts:
        if flip_x:
            x = -x
        if flip_y:
            y = -y
        out.append((x * c - y * s, x * s + y * c))
    return out


def resample(poly, max_seg):
    out = []
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        k = max(1, int(math.ceil(L / max_seg)))
        for j in range(k):
            out.append((a[0] + (b[0] - a[0]) * j / k, a[1] + (b[1] - a[1]) * j / k))
    return out


def asymmetry_mm(poly_right, poly_left_mirrored, step=0.25):
    """Max distance (both ways) between two boundaries, ignoring the y = 0 chord.  Returns (max, mean)."""
    def samples(poly):
        pts = []
        n = len(poly)
        for i in range(n):
            a, b = poly[i], poly[(i + 1) % n]
            if abs(a[1]) < 1e-6 and abs(b[1]) < 1e-6:
                continue
            L = math.hypot(b[0] - a[0], b[1] - a[1])
            k = max(1, int(math.ceil(L / step)))
            for j in range(k + 1):
                pts.append((a[0] + (b[0] - a[0]) * j / k, a[1] + (b[1] - a[1]) * j / k))
        return pts

    def dist_to(poly, p):
        best = 1e30
        n = len(poly)
        for i in range(n):
            a, b = poly[i], poly[(i + 1) % n]
            if abs(a[1]) < 1e-6 and abs(b[1]) < 1e-6:
                continue
            d, _ = seg_dist(p, a, b)
            best = min(best, d)
        return best
    ds = [dist_to(poly_left_mirrored, p) for p in samples(poly_right)] + [dist_to(poly_right, p) for p in samples(poly_left_mirrored)]
    return max(ds), sum(ds) / len(ds)


def load_outline(src, span_mm, base_dir=None):
    """src: config 'source' block.  Returns dict(half=right-half polygon CCW with the root edge on y=0, meta..., asymmetry_mm, notes)."""
    typ = src.get("type", "builtin")
    notes = []
    out = dict(source=typ, asymmetry_max_mm=None, asymmetry_mean_mm=None, notes=notes)
    if typ == "builtin":
        name = src["builtin"]
        if name == "swiftA":
            import bird_outline_v4_swift as sw
            g = sw.build_geometry4(sw.GLIDE)
            k = float(span_mm)
            half = [(x * k, y * k) for x, y in g["half_outline"]]
            out.update(x_hinge=g["x_h"] * k, tail_end_x=g["tail_end_x"] * k, nose_x=g["nose_x"] * k, neck_half_width=g["wt0"] * k / 2, body_half=g["y_body"] * k,
                       builtin="bird_outline_v4_swift.build_geometry4(GLIDE) at span %g" % k)
        elif name == "b45":
            import flat_test as ft
            g = ft.wing_geometry(float(span_mm), float(src.get("ar", 4.5)))
            b2 = g["span"] / 2
            half = [(0.0, 0.0), (g["tipx"], b2), (g["tipx"] + g["ct"], b2), (g["cr"], 0.0)]
            out.update(x_hinge=None, tail_end_x=g["cr"], nose_x=0.0, builtin="flat_test.wing_geometry(%g, %g)" % (span_mm, src.get("ar", 4.5)))
        else:
            raise ValueError("unknown builtin outline %r" % name)
        half = ccw(dedupe(half))
        out["half"] = half
        out["half_span"] = max(p[1] for p in half)
        return out
    path = Path(src["path"])
    if not path.is_absolute():
        path = (Path(base_dir) if base_dir else Path.cwd()) / path
    text = path.read_text(encoding="utf-8", errors="replace")
    if typ == "dxf" or path.suffix.lower() == ".dxf":
        lay = src.get("layers")
        parsed = parse_dxf(text, set(lay) if lay else None)
        if not parsed["chains"] and lay:
            raise ValueError("no entities on layers %s (found layers %s)" % (lay, sorted(parse_dxf(text)["layers"])))
        flipy = bool(src.get("flip_y", False))
    else:
        parsed = parse_svg(text, src.get("mm_per_unit"))
        flipy = bool(src.get("flip_y", True))            # SVG y points down
    out["entity_counts"] = parsed["counts"]
    loops, left, gap = assemble_loops(parsed["chains"], float(src.get("join_tol_mm", 0.05)))
    if not loops:
        raise ValueError("no closed outline found (open chains: %d, join tolerance %.3g mm)" % (len(left), float(src.get("join_tol_mm", 0.05))))
    out["max_join_gap_mm"] = gap
    out["open_chains_ignored"] = len(left)
    loop = max(loops, key=lambda p: abs(area2(p)))
    pts = _rot_flip(loop, float(src.get("rotate_deg", 0.0)), bool(src.get("flip_x", False)), flipy)
    kind = src.get("kind", "auto")
    ys = [p[1] for p in pts]
    if kind == "auto":
        kind = "full" if (min(ys) < -0.02 * max(max(ys), -min(ys)) and max(ys) > 0.02 * max(max(ys), -min(ys))) else "half"
    out["kind"] = kind
    pts = ccw(dedupe(pts))
    if kind == "full":
        pieces = split_by_line([pts], 1, 0.0)
        right = [p for p in pieces if sum(q[1] for q in p) > 0]
        left_p = [p for p in pieces if sum(q[1] for q in p) < 0]
        if len(right) != 1 or len(left_p) != 1:
            raise ValueError("full outline does not split into one right and one left half at y = 0 (%d / %d pieces)" % (len(right), len(left_p)))
        R_, L_ = right[0], left_p[0]
        Lm = ccw([(x, -y) for x, y in L_])
        amax, amean = asymmetry_mm(R_, Lm)
        out["asymmetry_max_mm"], out["asymmetry_mean_mm"] = amax, amean
        sym = src.get("symmetrize", "right")
        half = R_ if sym == "right" else (Lm if sym == "left" else None)
        if half is None:
            raise ValueError("symmetrize must be 'right' or 'left'")
        notes.append("full outline: right/left halves differ by up to %.3f mm (mean %.3f); '%s' half used as the master" % (amax, amean, sym))
    else:
        if max(ys) <= 1e-9 and min(ys) < 0:
            pts = ccw([(x, -y) for x, y in pts])
        y0 = min(p[1] for p in pts)
        if abs(y0) > 1e-6:
            notes.append("half outline shifted by %.4f mm in y so that the root edge lies on y = 0" % (-y0))
            pts = [(x, y - y0) for x, y in pts]
        half = pts
    if src.get("scale") not in (None, 1, 1.0):
        half = [(x * float(src["scale"]), y * float(src["scale"])) for x, y in half]
    hs = max(p[1] for p in half)
    if src.get("scale_to_span", True) and span_mm:
        f = span_mm / (2 * hs)
        if abs(f - 1.0) > 1e-9:
            notes.append("scaled by %.6f so the full span is %.2f mm (input span %.2f mm)" % (f, span_mm, 2 * hs))
            half = [(x * f, y * f) for x, y in half]
    half = [snap(p) for p in dedupe(half)]
    if src.get("x_origin") == "root_le":
        xr = min(p[0] for p in half if abs(p[1]) < 1e-6)
        half = [(x - xr, y) for x, y in half]
        notes.append("x shifted so the root leading edge is x = 0")
    out["half"] = ccw(half)
    out["half_span"] = max(p[1] for p in out["half"])
    out["x_hinge"] = src.get("hinge_x_mm")
    out["tail_end_x"] = max(p[0] for p in out["half"])
    out["nose_x"] = min(p[0] for p in out["half"])
    return out


# =============================================================================================== settings
DEFAULTS = {
    "name": "unnamed",
    "span_mm": 400.0,
    "source": {"type": "builtin", "builtin": "swiftA"},
    "dihedral_deg": 0.0,                 # per half, set by the rail saddle faces
    "root_gap_mm": 0.4,                  # gap between the two halves' root edges (raised automatically if dihedral + tail bend would make them touch)
    "tail": {"bend_deg": 0.0, "hinge_x_mm": None, "hinge_zone_mm": 4.0},
    "sheet": {"layer_height_mm": 0.1, "n_layers": 4},
    "bead": {"enabled": True, "width_root_mm": 3.0, "width_tip_mm": 1.2, "height_root_mm": 1.5, "height_tip_mm": 0.6, "stop_frac": 0.9},
    "root_strip": {"width_mm": 7.0, "thickness_mm": 1.0},
    "keys": {"head_r_mm": 1.6, "neck_w_mm": 1.2, "neck_len_mm": 1.6, "height_mm": 0.8, "clearance_mm": 0.15, "pitch_mm": 28.0, "margin_mm": 1.2, "n_min": 2},
    "split": {"joint_width_mm": 8.0, "joint_thickness_mm": 1.0, "key_head_r_mm": 3.0, "key_neck_w_mm": 2.4, "key_neck_len_mm": 2.4, "key_fracs": [0.3, 0.55, 0.8], "max_pieces": 8, "scan_step_mm": 1.0},
    "bed": {"size_mm": 250.0, "rotation_step_deg": 0.5},
    "rail": {"web_thickness_mm": 2.0, "ballast_block_thickness_mm": 4.0, "plate_thickness_mm": 1.3, "plate_overlap_mm": 0.5, "wall_mm": 1.0, "window_height_mm": 6.0,
             "plate_margin_mm": 1.0, "rib_mm": 1.2, "lightening": True, "web_depth_min_mm": 3.0, "front_x_mm": None, "ballast_station_x_mm": None,
             "print_orientation": "side", "nose_extension_mm": 0.0, "fin_slot_len_mm": 6.0},
    "ballast": {"density_g_cm3": 11.3, "fill_margin": 0.25},
    "fin": {"type": "none", "height_mm": 20.95, "chord_mm": 16.76, "taper": 1.0, "thickness_mm": 0.6, "tab_chord_frac": 0.35, "tab_span_frac": 0.7,
            "tab_angles_deg": [0.0, 2.0, -2.0, 4.0, -4.0], "tab_on": "right", "tab_flat_mm": 4.0, "key_head_r_mm": 1.0, "key_neck_w_mm": 0.9, "key_neck_len_mm": 0.8,
            "slot_clearance_mm": 0.1, "x_te_mm": None, "wing_tab_fracs": [0.25, 0.75], "wing_tab_len_frac": 0.2},
    "mass": {"pla_density_g_cm3": 1.24, "glue_root_g": 0.3, "glue_per_split_g": 0.05, "glue_per_fin_g": 0.05, "target_cg_mm": None, "cg_reference": "nose_tip", "wing_loading_area": "wing"},
    "foam": {"paper": "A4", "tile_overlap_mm": 10.0, "margin_mm": 8.0},
}


def deep_merge(base, over):
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def load_config(path_or_dict, base_dir=None):
    if isinstance(path_or_dict, dict):
        return deep_merge(DEFAULTS, path_or_dict)
    p = Path(path_or_dict)
    raw = json.loads(p.read_text(encoding="utf-8"))
    raw = {k: v for k, v in raw.items() if not k.startswith("_")}
    return deep_merge(DEFAULTS, raw)


# =============================================================================================== bed fit
def convex_hull(pts):
    pts = sorted(set((round(p[0], 6), round(p[1], 6)) for p in pts))
    if len(pts) <= 2:
        return pts

    def half(seq):
        h = []
        for p in seq:
            while len(h) >= 2 and _cross(h[-2], h[-1], p) <= 0:
                h.pop()
            h.append(p)
        return h
    lo, up = half(pts), half(reversed(pts))
    return lo[:-1] + up[:-1]


def fit_search(points, bed=250.0, step=0.5):
    """Rotate the footprint in `step` degree steps; return the angle (smallest bounding-box area among fitting angles, 0 preferred) or the least-bad angle."""
    hull = convex_hull(points)
    best_fit = None
    least = None
    n = int(round(180.0 / step))
    for i in range(n):
        a = i * step
        c, s = math.cos(math.radians(a)), math.sin(math.radians(a))
        us = [x * c + y * s for x, y in hull]
        vs = [-x * s + y * c for x, y in hull]
        w, h = max(us) - min(us), max(vs) - min(vs)
        if w <= bed + 1e-9 and h <= bed + 1e-9:
            key = (0 if i == 0 else 1, w * h)
            if best_fit is None or key < best_fit[0]:
                best_fit = (key, a, w, h)
        m = max(w, h)
        if least is None or m < least[0]:
            least = (m, a, w, h)
    if best_fit:
        return dict(fits=True, angle_deg=best_fit[1], width=best_fit[2], height=best_fit[3])
    return dict(fits=False, angle_deg=least[1], width=least[2], height=least[3])


def place_on_bed(tris, angle_deg, centre=128.0):
    """Rotate about z by angle, put the bounding-box centre at (centre, centre) (middle of a 256 mm bed) and z >= 0."""
    c, s = math.cos(math.radians(angle_deg)), math.sin(math.radians(angle_deg))
    M = [[c, s, 0], [-s, c, 0], [0, 0, 1]]
    out = affine(tris, M)
    lo, hi = tri_bbox(out)
    return affine(out, [[1, 0, 0], [0, 1, 0], [0, 0, 1]], (centre - (lo[0] + hi[0]) / 2, centre - (lo[1] + hi[1]) / 2, -lo[2]))


# =============================================================================================== jigsaw key outline
def key_polygon(cx, cy, r, wn, ln, sign=1, clr=0.0, n_arc=64):
    """Jigsaw knob (circular head + straight neck) centred on the head centre; neck points toward sign*x.  clr offsets every edge outward (socket = key with clr).
    The neck END is at r + ln from the head centre (a free-standing post: the neck only gives the shape its jigsaw character)."""
    R, nw, Lx = r + clr, wn / 2 + clr, r + ln + clr
    th0 = math.asin(min(nw / R, 0.999))
    pts = [(Lx, nw)]
    N = 96
    angs = [th0] + [2 * math.pi * k / N for k in range(1, N) if th0 + 1e-9 < 2 * math.pi * k / N < 2 * math.pi - th0 - 1e-9] + [2 * math.pi - th0]
    for a in angs:
        pts.append((R * math.cos(a), R * math.sin(a)))
    pts.append((Lx, -nw))
    pts = [(cx + sign * x, cy + y) for x, y in pts]
    return ccw(pts)




def tab_path(xc, yc, r, wn, ln, clr=0.0, n_arc=48):
    """Open boundary path of a jigsaw TAB on a cut line y = yc, protruding toward +y: starts on the +x side of the base, up the neck, over the head, down the -x side
    (the order in which it is inserted into the boundary of the INBOARD piece, whose top edge runs +x -> -x).  clr > 0 gives the matching NOTCH outline
    (every edge moved outward by clr; head centre unchanged); reversed it is the boundary of the OUTBOARD piece (its bottom edge runs -x -> +x)."""
    R, nw = r + clr, wn / 2 + clr
    yh = yc + ln + r
    ac = math.acos(min(nw / R, 0.999999))
    a0, a1 = -ac, math.pi + ac
    N = 96
    angs = [a0] + [2 * math.pi * k / N - 2 * math.pi * (1 if 2 * math.pi * k / N > a1 else 0) for k in range(-N, N)]
    angs = [a0] + sorted(a for a in set(round(2 * math.pi * k / N, 12) for k in range(-N, 2 * N)) if a0 + 1e-9 < a < a1 - 1e-9) + [a1]
    pts = [(xc + nw, yc)]
    for a in angs:
        pts.append((xc + R * math.cos(a), yh + R * math.sin(a)))
    pts.append((xc - nw, yc))
    return [snap(p) for p in pts]


def tab_extent(r, ln):
    return ln + 2 * r


# =============================================================================================== wing: derived parameters
def derive(cfg, outline):
    sh, bd, rs, ky, sp = cfg["sheet"], cfg["bead"], cfg["root_strip"], cfg["keys"], cfg["split"]
    P = {}
    P["layer"], P["n_layers"] = float(sh["layer_height_mm"]), int(sh["n_layers"])
    if P["n_layers"] < 2:
        raise ValueError("sheet.n_layers must be >= 2 (Kai: at least 2 layers)")
    P["t_m"] = P["layer"] * P["n_layers"]
    P["dihedral"] = float(cfg["dihedral_deg"])
    P["bend"] = float(cfg["tail"]["bend_deg"])
    xh = cfg["tail"].get("hinge_x_mm")
    xh = outline.get("x_hinge") if xh is None else xh
    P["x_hinge"] = xh
    P["hz"] = float(cfg["tail"]["hinge_zone_mm"])
    P["hinge_active"] = bool(xh is not None and abs(P["bend"]) > 1e-9)
    half = outline["half"]
    P["half_span"] = max(p[1] for p in half)
    P["bead_on"] = bool(bd["enabled"])
    P["bead_w0"], P["bead_w1"] = float(bd["width_root_mm"]), float(bd["width_tip_mm"])
    P["bead_h0"], P["bead_h1"] = float(bd["height_root_mm"]), float(bd["height_tip_mm"])
    P["bead_stop"] = float(bd["stop_frac"]) * P["half_span"]
    if P["bead_on"] and min(P["bead_h0"], P["bead_h1"]) < P["layer"] - 1e-9:
        raise ValueError("bead heights must be at least one layer")
    P["sw"] = float(rs["width_mm"])
    hs = float(rs["thickness_mm"])
    need = P["t_m"] + (max(P["bead_h0"], P["bead_h1"]) if P["bead_on"] else 0.0)
    P["Hs_requested"] = hs
    P["Hs"] = max(hs, need, P["t_m"] + P["layer"])
    P["kh"] = float(ky["height_mm"])
    P["key_r"], P["key_wn"], P["key_ln"] = float(ky["head_r_mm"]), float(ky["neck_w_mm"]), float(ky["neck_len_mm"])
    P["key_clr"] = float(ky["clearance_mm"])
    P["key_pitch"], P["key_margin"], P["key_nmin"] = float(ky["pitch_mm"]), float(ky["margin_mm"]), int(ky["n_min"])
    P["wj"] = float(sp["joint_width_mm"])
    P["Hj"] = max(float(sp["joint_thickness_mm"]), need)
    P["skey_r"], P["skey_wn"], P["skey_ln"] = float(sp["key_head_r_mm"]), float(sp["key_neck_w_mm"]), float(sp["key_neck_len_mm"])
    P["skey_fracs"] = list(sp["key_fracs"])
    P["bed"] = float(cfg["bed"]["size_mm"])
    P["rot_step"] = float(cfg["bed"]["rotation_step_deg"])
    # the root gap must survive the rigid bend of the tail about the slanted fold line: root edges converge by X sin(bend) sin(dihedral)
    P["root_gap_req"] = float(cfg["root_gap_mm"])
    if P["hinge_active"] and abs(P["dihedral"]) > 1e-9:
        xe = outline["tail_end_x"] - xh
        conv = 2 * xe * math.sin(math.radians(abs(P["bend"]))) * math.sin(math.radians(abs(P["dihedral"])))
        P["root_gap_conv"] = conv
        P["root_gap"] = max(P["root_gap_req"], conv + 0.2)
    else:
        P["root_gap_conv"] = 0.0
        P["root_gap"] = P["root_gap_req"]
    P["S0"] = P["root_gap"] / 2.0
    return P


# =============================================================================================== wing: leading-edge chain and bead curves
def le_chain(half):
    """Vertices from the nose (root edge, foremost) along the leading edge to the tip (first vertex at max y)."""
    n = len(half)
    root = [i for i, p in enumerate(half) if abs(p[1]) < 1e-6]
    i0 = min(root, key=lambda i: half[i][0])
    ymax = max(p[1] for p in half)
    chain = [half[i0]]
    i = i0
    for _ in range(n):
        i = (i - 1) % n
        chain.append(half[i])
        if half[i][1] >= ymax - 1e-6:
            break
    return chain


def chain_at_y(chain, y):
    for i in range(len(chain) - 1):
        a, b = chain[i], chain[i + 1]
        if (a[1] - y) * (b[1] - y) <= 0 and a[1] != b[1]:
            t = (y - a[1]) / (b[1] - a[1])
            return (a[0] + t * (b[0] - a[0]), y)
    raise ValueError("chain does not reach y = %g" % y)


def bead_curves(P, half):
    """-> dict(le=[(x,y)] resampled LE chain over the bead range, inner=[(x,y)], w(y), h(y), y0, y1).  The bead is a band between the leading edge and the same chain
    moved inward by w(y) along the local normal; width and height taper linearly with y between y0 (strip edge) and y1 (stop)."""
    y0 = P["sw"]
    y1 = P["bead_stop"]
    ch = le_chain(half)
    if not P["bead_on"] or y1 <= y0 + 1.0:
        return None
    # keep the monotone-in-y prefix
    mono = [ch[0]]
    for p in ch[1:]:
        if p[1] > mono[-1][1] + 1e-9:
            mono.append(p)
    # resample to <= 6 mm in y
    pts = []
    for i in range(len(mono) - 1):
        a, b = mono[i], mono[i + 1]
        k = max(1, int(math.ceil((b[1] - a[1]) / 6.0)))
        for j in range(k):
            pts.append((a[0] + (b[0] - a[0]) * j / k, a[1] + (b[1] - a[1]) * j / k))
    pts.append(mono[-1])
    w = lambda y: P["bead_w0"] + (P["bead_w1"] - P["bead_w0"]) * (min(max(y, y0), y1) - y0) / (y1 - y0)
    h = lambda y: P["bead_h0"] + (P["bead_h1"] - P["bead_h0"]) * (min(max(y, y0), y1) - y0) / (y1 - y0)
    inner = []
    for i, p in enumerate(pts):
        a = pts[max(i - 1, 0)]
        b = pts[min(i + 1, len(pts) - 1)]
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dx, dy) or 1.0
        nx, ny = dy / L, -dx / L
        wi = w(p[1])
        inner.append((p[0] + wi * nx, p[1] + wi * ny))
    # inner chain must be monotone in y over the bead range
    for i in range(1, len(inner)):
        if inner[i][1] <= inner[i - 1][1] and pts[i][1] >= y0 - 1e-6:
            raise ValueError("bead inner curve folds back near y=%.1f (bead too wide for the leading-edge curvature)" % pts[i][1])
    return dict(le=pts, inner=inner, w=w, h=h, y0=y0, y1=y1)


# =============================================================================================== wing: pieces (spanwise split with jigsaw joints)
def _chord_at(half, y):
    xs = []
    n = len(half)
    for i in range(n):
        a, b = half[i], half[(i + 1) % n]
        if (a[1] - y) * (b[1] - y) < 0:
            t = (y - a[1]) / (b[1] - a[1])
            xs.append(a[0] + t * (b[0] - a[0]))
    return sorted(xs)


def _slab_points(half, ya, yb):
    polys = split_by_line([half], 1, ya) if ya > 1e-9 else [half]
    polys = [p for p in polys if min(q[1] for q in p) >= ya - 1e-6]
    polys = split_by_line(polys, 1, yb)
    polys = [p for p in polys if max(q[1] for q in p) <= yb + 1e-6]
    return [q for p in polys for q in p]


def plan_cuts(P, half, notes):
    """Greedy spanwise cuts from the root: the largest slab that fits the bed (rotation search, jigsaw tab allowance included) each time."""
    ymax = P["half_span"]
    bed, step = P["bed"], P["rot_step"]
    tabext = tab_extent(P["skey_r"], P["skey_ln"])
    # tail extent: cuts must lie outboard of the tail fan and the strip
    ytail = max([p[1] for p in half if P["x_hinge"] is not None and p[0] > P["x_hinge"] - 1e-6] + [0.0])
    ymin_cut = max(P["sw"] + 2 * P["wj"], ytail + P["wj"] + 4.0)
    fit0 = fit_search(half, bed, step)
    if fit0["fits"]:
        return [], fit0
    cuts = []
    ya = 0.0
    sstep = float(P.get("scan_step", 1.0))
    for _ in range(int(P.get("max_pieces", 8))):
        found = None
        yb = ymax
        # does the whole remainder fit?
        pts = _slab_points(half, ya, ymax) if ya > 0 else half
        if fit_search(pts, bed, step)["fits"]:
            return cuts, fit0
        yb = ymax - sstep
        while yb > ya + max(3 * P["wj"], 10.0):
            if yb >= ymin_cut and len(_chord_at(half, yb)) == 2:
                pts = _slab_points(half, ya, yb)
                xs = _chord_at(half, yb)
                pts = pts + [(xs[0], yb + tabext), (xs[1], yb + tabext)]
                if fit_search(pts, bed, step)["fits"]:
                    found = yb
                    break
            yb -= sstep
        if found is None:
            notes.append("FLAG: no spanwise cut found that makes the piece starting at y=%.1f fit the %g mm bed" % (ya, bed))
            return cuts, fit0
        cuts.append(found)
        ya = found
    notes.append("FLAG: max_pieces reached with the remainder still too large")
    return cuts, fit0


def make_pieces(P, half, notes):
    cuts, fit0 = plan_cuts(P, half, notes)
    pieces = []
    rest = half
    y_lo = 0.0
    for c in cuts:
        parts = split_by_line([rest], 1, c)
        low = [p for p in parts if max(q[1] for q in p) <= c + 1e-6]
        up = [p for p in parts if min(q[1] for q in p) >= c - 1e-6]
        if len(low) != 1 or len(up) != 1:
            raise ValueError("cut at y=%.2f does not give one inboard and one outboard piece" % c)
        low, up = list(low[0]), list(up[0])
        xs = _chord_at(rest, c)
        xa, xb = xs[0], xs[-1]
        r = P["skey_r"]
        margin = r + 1.5
        keys = [xa + f * (xb - xa) for f in P["skey_fracs"] if xa + margin <= xa + f * (xb - xa) <= xb - margin]
        if not keys:
            keys = [(xa + xb) / 2]
            notes.append("FLAG: joint at y=%.1f is too short for the requested key fractions; one centred key used" % c)
        keys = [round(k, 6) for k in sorted(keys)]
        # inboard: replace the chord edge (xb,c) -> (xa,c) by tabs, descending x
        n = len(low)
        ia = next(i for i in range(n) if abs(low[i][1] - c) < 1e-6 and abs(low[i][0] - xb) < 1e-5 and abs(low[(i + 1) % n][1] - c) < 1e-6 and abs(low[(i + 1) % n][0] - xa) < 1e-5)
        path = []
        for xc in reversed(keys):
            path += tab_path(xc, c, P["skey_r"], P["skey_wn"], P["skey_ln"], 0.0)
        low = low[:ia + 1] + path + low[ia + 1:]
        n = len(up)
        ib = next(i for i in range(n) if abs(up[i][1] - c) < 1e-6 and abs(up[i][0] - xa) < 1e-5 and abs(up[(i + 1) % n][1] - c) < 1e-6 and abs(up[(i + 1) % n][0] - xb) < 1e-5)
        path = []
        for xc in keys:
            path += list(reversed(tab_path(xc, c, P["skey_r"], P["skey_wn"], P["skey_ln"], P["key_clr"])))
        up = up[:ib + 1] + path + up[ib + 1:]
        pieces.append(dict(poly=dedupe(low), y_lo=y_lo, y_hi=c, cut_lo=(cuts[len(pieces) - 1] if pieces else None), cut_hi=c, cut_keys_hi=keys))
        rest = dedupe(up)
        y_lo = c
    pieces.append(dict(poly=dedupe(rest), y_lo=y_lo, y_hi=P["half_span"], cut_lo=(cuts[-1] if cuts else None), cut_hi=None, cut_keys_hi=[]))
    for i, pc in enumerate(pieces):
        pc["index"] = i
        pc["is_root"] = (i == 0)
        pc["cut_keys_lo"] = pieces[i - 1]["cut_keys_hi"] if i else []
    return pieces, fit0


# =============================================================================================== wing: regions of one piece
def place_keys(P, strip_polys, notes):
    """Jigsaw knobs on the root strip (centre line of the strip, y = sw/2).  Returns list of dict(cx, cy, sign, poly, poly_socket)."""
    yc = P["sw"] / 2.0
    r, wn, ln, clr = P["key_r"], P["key_wn"], P["key_ln"], P["key_clr"]
    keys = []
    m = r + ln + clr + P["key_margin"]
    for poly in strip_polys:
        xs = []
        n = len(poly)
        for i in range(n):
            a, b = poly[i], poly[(i + 1) % n]
            if (a[1] - yc) * (b[1] - yc) < 0:
                t = (yc - a[1]) / (b[1] - a[1])
                xs.append(a[0] + t * (b[0] - a[0]))
        xs.sort()
        for k in range(0, len(xs) - 1, 2):
            xa, xb = xs[k], xs[k + 1]
            L = (xb - xa) - 2 * m
            if L < 0:
                continue
            cnt = max(P["key_nmin"], int(math.ceil(L / P["key_pitch"])) + 1)
            while cnt > 1 and L / (cnt - 1) < 2 * (r + ln) + 2.0:
                cnt -= 1
            cands = [(xa + m + (L * i / (cnt - 1) if cnt > 1 else L / 2)) for i in range(cnt)]
            for i, cx in enumerate(cands):
                sign = 1 if (len(keys) % 2 == 0) else -1
                kp = key_polygon(cx, yc, r, wn, ln, sign, 0.0)
                sk = key_polygon(cx, yc, r, wn, ln, sign, clr)
                ok = all(point_in_poly(q, poly) for q in sk) and min(seg_dist(q, poly[j], poly[(j + 1) % len(poly)])[0] for q in sk for j in range(len(poly))) > 0.5
                if ok:
                    keys.append(dict(cx=cx, cy=yc, sign=sign, poly=[snap(q) for q in kp], socket=sk, strip_poly=poly))
                else:
                    notes.append("FLAG: root key at x=%.1f does not fit inside the strip and was skipped" % cx)
    return keys


def piece_regions(P, piece, bead, notes):
    """Regions (outer, holes, class) of one piece, and the keys on its root strip."""
    polys = [list(piece["poly"])]
    y_lo, y_hi = piece["y_lo"], piece["y_hi"]
    wj = P["wj"]
    breaks = []
    if piece["is_root"]:
        breaks.append(P["sw"])
    if piece["cut_lo"] is not None:
        breaks.append(piece["cut_lo"] + wj)
    if piece["cut_hi"] is not None:
        breaks.append(piece["cut_hi"] - wj)
    b0 = b1 = None
    if bead:
        b0 = max(bead["y0"], (piece["cut_lo"] + wj) if piece["cut_lo"] is not None else 0.0)
        b1 = min(bead["y1"], (piece["cut_hi"] - wj) if piece["cut_hi"] is not None else 1e9)
        if b1 - b0 > 1.0 and b0 < y_hi and b1 > y_lo:
            breaks += [b0, b1]
        else:
            b0 = b1 = None
    for y in sorted(set(round(b, 6) for b in breaks)):
        if y_lo + 1e-6 < y < y_hi - 1e-6:
            polys = split_by_line(polys, 1, y)
    if piece["is_root"] and P["hinge_active"]:
        xa, xb = P["x_hinge"] - P["hz"] / 2, P["x_hinge"] + P["hz"] / 2
        new = []
        for p in polys:
            if max(q[1] for q in p) <= P["sw"] + 1e-6:
                new += split_by_line([p], 0, xa)
            else:
                new.append(p)
        polys = new
        new = []
        for p in polys:
            if max(q[1] for q in p) <= P["sw"] + 1e-6:
                new += split_by_line([p], 0, xb)
            else:
                new.append(p)
        polys = new
    if bead and b0 is not None:
        ym = (b0 + b1) / 2
        xle = chain_at_y(bead["le"], ym)
        probe = (xle[0] + 0.25 * bead["w"](ym), ym)
        target = None
        for i, p in enumerate(polys):
            if point_in_poly(probe, p):
                target = i
                break
        if target is not None:
            I0 = chain_at_y(bead["inner"], b0)
            I1 = chain_at_y(bead["inner"], b1)
            mid = [q for q in bead["inner"] if b0 + 1e-6 < q[1] < b1 - 1e-6]
            chain = [snap(I0)] + [snap(q) for q in mid] + [snap(I1)]
            polys[target:target + 1] = split_by_polyline(polys[target], chain)
        else:
            notes.append("FLAG: bead probe point outside the piece outline")
    xin = None
    if bead:
        def xin(y):
            return chain_at_y(bead["inner"], min(max(y, bead["inner"][0][1]), bead["inner"][-1][1]))[0]
    regions = []
    for p in polys:
        p = dedupe([snap(q) for q in p])
        if abs(parea(p)) < 1e-6:
            continue
        rp = rep_point(p)
        cls = "R"
        if piece["cut_lo"] is not None and rp[1] <= piece["cut_lo"] + wj:
            cls = "J"
        elif piece["cut_hi"] is not None and rp[1] >= piece["cut_hi"] - wj:
            cls = "J"
        elif piece["is_root"] and rp[1] <= P["sw"] and not (P["hinge_active"] and abs(rp[0] - P["x_hinge"]) <= P["hz"] / 2):
            cls = "S"
        elif bead and b0 is not None and b0 <= rp[1] <= b1 and rp[0] < xin(rp[1]):
            cls = "B"
        regions.append(dict(outer=ccw(p), holes=[], cls=cls))
    keys = []
    if piece["is_root"]:
        keys = place_keys(P, [r["outer"] for r in regions if r["cls"] == "S"], notes)
        for k in keys:
            for r in regions:
                if r["cls"] == "S" and r["outer"] == k["strip_poly"]:
                    r["holes"].append(cw(k["poly"]))
    return regions, keys


def region_heights(P, bead):
    t_m, Hs, Hj, kh = P["t_m"], P["Hs"], P["Hj"], P["kh"]
    hb = (lambda x, y: t_m + bead["h"](y)) if bead else (lambda x, y: t_m)
    return {"R": t_m, "S": Hs, "J": Hj, "B": hb, "K": Hs + kh}


def build_piece_solid(P, regions, keys, bead):
    H = region_heights(P, bead)
    mr = []
    for r in regions:
        mr.append(dict(outer=r["outer"], holes=r["holes"], zb=0.0, zt=H[r["cls"]]))
    for k in keys:
        mr.append(dict(outer=k["poly"], holes=[], zb=0.0, zt=H["K"]))
    return mesh_regions(mr)


def build_wing(cfg, outline, notes):
    half = outline["half"]
    P = derive(cfg, outline)
    P["scan_step"], P["max_pieces"] = float(cfg["split"]["scan_step_mm"]), int(cfg["split"]["max_pieces"])
    if P["Hs"] > P["Hs_requested"] + 1e-9:
        notes.append("root strip thickness raised from %.2f to %.2f mm so it is not lower than the tallest bead (+ membrane)" % (P["Hs_requested"], P["Hs"]))
    tabs_added = None
    fin = cfg["fin"]
    if fin["type"] == "tip":
        half, tabs_added = add_tip_tabs(half, P, fin, notes)
    bead = bead_curves(P, half)
    pieces, fit0 = make_pieces(P, half, notes)
    solids = []
    allkeys = []
    for pc in pieces:
        regions, keys = piece_regions(P, pc, bead, notes)
        tris = build_piece_solid(P, regions, keys, bead)
        pc["regions"], pc["keys"], pc["tris_halfsolid"] = regions, keys, tris
        pc["area"] = abs(parea(pc["poly"]))
        allkeys += keys
    return dict(P=P, half=half, bead=bead, pieces=pieces, fit_unsplit=fit0, tabs=tabs_added)


def add_tip_tabs(half, P, fin, notes):
    """Two tabs on the tip edge (fin slots take them), like flat_test: tab length 20 % of the tip chord, centres at 25 % and 75 %, protruding by the fin thickness."""
    ymax = max(p[1] for p in half)
    idx = [i for i, p in enumerate(half) if p[1] >= ymax - 1e-6]
    if len(idx) < 2:
        notes.append("FLAG: no tip edge for tip-fin tabs")
        return half, None
    n = len(half)
    # CCW: tip edge runs from larger x (TE end) to smaller x (LE end)
    i = next(i for i in idx if half[(i + 1) % n][1] >= ymax - 1e-6 and half[i][0] > half[(i + 1) % n][0])
    a, b = half[i], half[(i + 1) % n]
    chord = a[0] - b[0]
    tl = float(fin.get("wing_tab_len_frac", 0.2)) * chord
    tt = float(fin["thickness_mm"])
    path = []
    tabs = []
    for f in sorted(fin.get("wing_tab_fracs", [0.25, 0.75]), reverse=True):      # descending x
        xc = b[0] + f * chord
        path += [(xc + tl / 2, ymax), (xc + tl / 2, ymax + tt), (xc - tl / 2, ymax + tt), (xc - tl / 2, ymax)]
        tabs.append(dict(x_centre=xc, length=tl, out=tt))
    newhalf = half[:i + 1] + [snap(q) for q in path] + half[i + 1:]
    if tl < 2.0:
        notes.append("FLAG: tip-fin tabs are only %.1f mm long (tip chord %.1f mm)" % (tl, chord))
    P["tip_chord"] = chord
    return newhalf, tabs


# =============================================================================================== frames: sheet <-> rail faces
def wing_frame(P):
    """Rail-face frames in the flight frame for the RIGHT side: wing plate (X,S,N) = (ex, a, nw), tail plate rotated by the bend about the fold line x = x_hinge."""
    d, b = math.radians(P["dihedral"]), math.radians(P["bend"] if P["hinge_active"] else 0.0)
    c, s = math.cos(d), math.sin(d)
    ex = (1.0, 0.0, 0.0)
    a = (0.0, c, s)                                  # S axis (across the face, away from the centreline)
    nw = (0.0, -s, c)                                # face normal of the wing plate (up, away from the rail)
    ut = tuple(math.cos(b) * ex[i] + math.sin(b) * nw[i] for i in range(3))
    nt = tuple(-math.sin(b) * ex[i] + math.cos(b) * nw[i] for i in range(3))
    return dict(ex=ex, a=a, nw=nw, ut=ut, nt=nt, c=c, s=s, beta=b, delta=d)


def half_to_flight(P, p, side=1):
    """(x, y, h) of the RIGHT half-solid -> flight coordinates.  side = -1 gives the left half (mirror).  The strip bottom (h = Hs) lies on the rail face."""
    F = wing_frame(P)
    x, y, h = p
    S = y + P["S0"]
    N = P["Hs"] - h
    if P["hinge_active"] and x >= P["x_hinge"]:
        X = x - P["x_hinge"]
        q = tuple(P["x_hinge"] * F["ex"][i] + X * F["ut"][i] + S * F["a"][i] + N * F["nt"][i] for i in range(3))
    else:
        q = tuple(x * F["ex"][i] + S * F["a"][i] + N * F["nw"][i] for i in range(3))
    return (q[0], side * q[1], q[2])


def plate_matrix(F, tail):
    cols = (F["ut"], F["a"], F["nt"]) if tail else (F["ex"], F["a"], F["nw"])
    return [[cols[j][i] for j in range(3)] for i in range(3)]


def weld_cancel(a, b, nd=5):
    """Union of two closed meshes that share coincident, opposite faces: those faces are removed."""
    key = lambda t: tuple(sorted((round(p[0], nd) + 0.0, round(p[1], nd) + 0.0, round(p[2], nd) + 0.0) for p in t))
    ka = {}
    for i, t in enumerate(a):
        ka.setdefault(key(t), []).append(i)
    drop_a = set()
    keep_b = []
    for t in b:
        k = key(t)
        if k in ka and ka[k]:
            drop_a.add(ka[k].pop())
        else:
            keep_b.append(t)
    return [t for i, t in enumerate(a) if i not in drop_a] + keep_b


def _poly_z(poly, x):
    if x <= poly[0][0]:
        return poly[0][1]
    for i in range(len(poly) - 1):
        a, b = poly[i], poly[i + 1]
        if a[0] <= x <= b[0]:
            if b[0] == a[0]:
                return b[1]
            return a[1] + (b[1] - a[1]) * (x - a[0]) / (b[0] - a[0])
    return poly[-1][1]


# =============================================================================================== rail
def _lighten(X0, X1, S0, S1, sockets, rib, side=1.0, min_len=5.0):
    """Rectangular lightening holes between the sockets (and the plate ends); a rib of `rib` mm is left around every socket and at the ends."""
    ext = sorted((min(p[0] for p in sk), max(p[0] for p in sk)) for sk in sockets)
    gaps, x = [], X0 + rib
    for lo, hi in ext:
        gaps.append((x, lo - rib))
        x = hi + rib
    gaps.append((x, X1 - rib))
    out = []
    for a, b in gaps:
        if b - a >= min_len:
            out.append([(a, S0 + side), (b, S0 + side), (b, S1 - side), (a, S1 - side)])
    return out


def build_rail(cfg, P, wing, outline, win_len, win_station):
    """Rail = web (extruded side profile: through-window for the ballast in a locally thicker block, optional fin slot) + four face plates (wing/tail x right/left) with
    jigsaw sockets and lightening holes.  Overlapping closed shells (the web top sits `plate_overlap` inside the plates); union volume by sampling.  Flight coordinates."""
    rc = cfg["rail"]
    F = wing_frame(P)
    c, s, beta = F["c"], F["s"], F["beta"]
    tw, twb = float(rc["web_thickness_mm"]), float(rc["ballast_block_thickness_mm"])
    hh, ov, wall = float(rc["plate_thickness_mm"]), float(rc["plate_overlap_mm"]), float(rc["wall_mm"])
    win_h, pm, rib = float(rc["window_height_mm"]), float(rc["plate_margin_mm"]), float(rc["rib_mm"])
    depth = P["kh"] + P["key_clr"]
    if hh - depth < 0.5:
        hh = depth + 0.55
    S0, sw = P["S0"], P["sw"]
    S1 = S0 + sw + pm
    yc = sw / 2.0
    rootp = wing["pieces"][0]
    keys = rootp["keys"]
    xle = chain_at_y(le_chain(wing["half"]), yc)[0]
    xp0 = xle + 0.5
    hinge = P["hinge_active"]
    xh, hz = P["x_hinge"], P["hz"]
    chord = _chord_at(outline["half"], yc)
    if hinge:
        xp1w = xh - hz / 2
        Xend = chord[-1] - 0.5 - xh
    else:
        xp1w = chord[-1] - 0.5
        Xend = None
    sock_w, sock_t = [], []
    for k in keys:
        sk = key_polygon(k["cx"], k["cy"] + S0, P["key_r"], P["key_wn"], P["key_ln"], k["sign"], P["key_clr"])
        if hinge and k["cx"] > xh:
            sock_t.append([(x - xh, y) for x, y in sk])
        else:
            sock_w.append(sk)

    def plate_regions(X0, X1, sockets):
        outer = [(X0, S0), (X1, S0), (X1, S1), (X0, S1)]
        lh = _lighten(X0, X1, S0, S1, sockets, rib) if rc.get("lightening", True) else []
        reg = [dict(outer=outer, holes=[sk for sk in sockets] + lh, zb=-hh, zt=0.0)]
        for sk in sockets:
            reg.append(dict(outer=sk, holes=[], zb=-hh, zt=-depth))
        return reg, lh
    shells = {}
    reg_w, lh_w = plate_regions(xp0, xp1w, sock_w)
    shells["plate_wing_R"] = affine(mesh_regions(reg_w), plate_matrix(F, False))
    lh_t = []
    if hinge:
        reg_t, lh_t = plate_regions(hz / 2, Xend, sock_t)
        shells["plate_tail_R"] = affine(mesh_regions(reg_t), plate_matrix(F, True), (xh, 0.0, 0.0))
    # --- web profile (x, z): the top follows the plate bottoms + ov
    Smid = S0 + 0.75
    z_w = Smid * s - hh * c + ov
    if hinge:
        def tailpt(X):
            return (xh + X * math.cos(beta) + hh * math.sin(beta), X * math.sin(beta) * c + Smid * s - hh * math.cos(beta) * c + ov)
        pb, pe = tailpt(hz / 2), tailpt(Xend)
        after = [(xp1w, z_w), pb, pe]
        x_web1 = pe[0]
    else:
        after = [(xp1w, z_w)]
        x_web1 = xp1w
    win_x0 = win_station - win_len / 2 if win_len > 0 else None
    fin = cfg["fin"]
    centre_fin = fin["type"] == "centre"
    x_web0 = min(xp0 - 1.0 - float(rc.get("nose_extension_mm", 0.0)), (win_x0 - wall) if win_x0 is not None else 1e9)
    if rc.get("front_x_mm") is not None:
        x_web0 = float(rc["front_x_mm"])
    top_poly = [(x_web0, z_w)] + after
    Dvis = win_h + 2 * wall
    dmin = max(float(rc["web_depth_min_mm"]), 5.0 if centre_fin else 0.0)
    bx0 = bx1 = None
    ramp = 0.0
    if win_len > 0:
        bx0, bx1 = win_x0 - wall, win_x0 + win_len + wall
        ramp = max(Dvis - dmin, 0.1)

    def depth_at(x):
        if win_len <= 0 or Dvis <= dmin:
            return dmin
        if x <= bx1:
            return Dvis
        if x >= bx1 + ramp:
            return dmin
        return Dvis + (dmin - Dvis) * (x - bx1) / ramp
    xs = sorted(set([p[0] for p in top_poly] + ([bx1, bx1 + ramp] if win_len > 0 and bx1 + ramp < x_web1 else [])))
    xs = [x for x in xs if top_poly[0][0] - 1e-9 <= x <= top_poly[-1][0] + 1e-9]
    bot_poly = [(x, _poly_z(top_poly, x) - ov - depth_at(x)) for x in xs]
    outer = list(bot_poly) + list(reversed(top_poly))
    holes = []
    if win_len > 0:
        wz0 = _poly_z(bot_poly, win_x0 + win_len / 2) + wall
        holes.append([(win_x0, wz0), (win_x0 + win_len, wz0), (win_x0 + win_len, wz0 + win_h), (win_x0, wz0 + win_h)])
    slot_ts = slot_x0 = None
    cuts = []
    if win_len > 0:
        cuts += [bx0, bx1]
    if centre_fin:
        slot_ts = float(fin["thickness_mm"]) + float(fin["slot_clearance_mm"])
        slot_x0 = x_web1 - float(rc["fin_slot_len_mm"])
        cuts.append(slot_x0)
    polys = [ccw(outer)]
    for xv in sorted(set(cuts)):
        polys = split_by_line(polys, 0, xv)
    regions_web = []
    for p in polys:
        rx = rep_point(p)[0]
        in_block = win_len > 0 and bx0 - 1e-6 <= rx <= bx1 + 1e-6
        in_slot = centre_fin and rx >= slot_x0
        hl = [h for h in holes if point_in_poly(((h[0][0] + h[2][0]) / 2, (h[0][1] + h[2][1]) / 2), p)]
        regions_web.append(dict(outer=p, holes=hl, zb=(slot_ts / 2 if in_slot else 0.0), zt=(twb / 2 if in_block else tw / 2)))
    half_web = mesh_regions(regions_web)                          # coordinates (x, z, w = y)
    web_R = affine(half_web, [[1, 0, 0], [0, 0, 1], [0, 1, 0]])
    mir = [[1, 0, 0], [0, -1, 0], [0, 0, 1]]
    web_L = affine(web_R, mir)
    shells["web"] = weld_cancel(web_R, web_L)
    shells["plate_wing_L"] = affine(shells["plate_wing_R"], mir)
    if hinge:
        shells["plate_tail_L"] = affine(shells["plate_tail_R"], mir)

    def inside_web(p):
        x, y, z = p
        if x < top_poly[0][0] or x > top_poly[-1][0]:
            return False
        th = twb if (win_len > 0 and bx0 <= x <= bx1) else tw
        if abs(y) > th / 2:
            return False
        if z > _poly_z(top_poly, x) or z < _poly_z(bot_poly, x):
            return False
        for h in holes:
            if h[0][0] < x < h[1][0] and h[0][1] < z < h[2][1]:
                return False
        if centre_fin and x > slot_x0 and abs(y) < slot_ts / 2:
            return False
        return True
    acc = [0.0, 0.0]

    def sample_plate(X0, X1, sockets, lights, tail):
        M = plate_matrix(F, tail)
        t0 = (xh, 0.0, 0.0) if tail else (0.0, 0.0, 0.0)
        nX = max(4, int((X1 - X0) / 0.25))
        dX = (X1 - X0) / nX
        sw_hi = S0 + (max(tw, twb) / 2) / c + 0.3
        nS = max(4, int((sw_hi - S0) / 0.1))
        dS = (sw_hi - S0) / nS
        nN = max(4, int(hh / 0.1))
        dN = hh / nN
        cell = dX * dS * dN
        for i in range(nX):
            X = X0 + (i + 0.5) * dX
            for j in range(nS):
                S = S0 + (j + 0.5) * dS
                if any(l[0][0] < X < l[1][0] and l[0][1] < S < l[2][1] for l in lights):
                    continue
                sk_here = any(point_in_poly((X, S), sk) for sk in sockets)
                for k in range(nN):
                    N = -hh + (k + 0.5) * dN
                    if sk_here and N > -depth:
                        continue
                    p = tuple(M[r][0] * X + M[r][1] * S + M[r][2] * N + t0[r] for r in range(3))
                    if inside_web(p):
                        acc[0] += cell
                        acc[1] += cell * p[0]
    sample_plate(xp0, xp1w, sock_w, lh_w, False)
    if hinge:
        sample_plate(hz / 2, Xend, sock_t, lh_t, True)
    ov_vol, ov_mom = 2 * acc[0], 2 * acc[1]
    vol = {k: mesh_volume(v) for k, v in shells.items()}
    cen = {k: mesh_centroid(v) for k, v in shells.items()}
    V = sum(vol.values()) - ov_vol
    xmom = sum(vol[k] * cen[k][0] for k in vol) - ov_mom
    info = dict(volume_mm3=V, cx=xmom / V, overlap_mm3=ov_vol, shell_volumes=vol, x_min=x_web0, x_max=x_web1, z_web_top=z_w, web_depth_visible=Dvis,
                plate_x=(xp0, xp1w), tail_plate_X=(hz / 2, Xend) if hinge else None, S_range=(S0, S1), plate_thickness=hh, pocket_depth=depth, web_thickness=tw,
                block_thickness=twb, window=dict(x0=win_x0, len=win_len, height=win_h, thickness=twb, area=win_len * win_h) if win_len > 0 else None,
                fin_slot=(dict(thickness=slot_ts, length=float(rc["fin_slot_len_mm"]), x0=slot_x0) if centre_fin else None),
                top_poly=top_poly, bot_poly=bot_poly, holes=holes, n_sockets=len(sock_w) + len(sock_t), sockets_wing=sock_w, sockets_tail=sock_t,
                lightening_wing=lh_w, lightening_tail=lh_t, frame=F, S0=S0, hh=hh, ov=ov, depth=depth, xh=xh, hinge=hinge, Xend=Xend, win_station=win_station,
                x_cut_block=(bx0, bx1) if win_len > 0 else None)
    return shells, info


def rail_print_mesh(shells, orientation):
    """Print orientation of the rail: 'side' = lying on its side (profile plane on the bed, flight y becomes height), 'upright' = as flown."""
    tris = [t for k in sorted(shells) for t in shells[k]]
    if orientation == "side":
        return affine(tris, [[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    return tris


# =============================================================================================== fin and trim-tab family
def fin_geometry(cfg, P, wing, rail_info):
    """Fin plan in (u, v) = (height from the base, chord from the leading edge).  Returns dict with the body regions, the tab family and feasibility flags.
    Trim tab mechanism (chosen here): the tab is a SEPARATE flat part; its forward edge is a jigsaw edge matching the fin body's trailing-edge jigsaw (keys symmetric about
    the tab mid-span), and beyond a flat landing it is tilted by a printed angle (a sheared plate).  The family 0, 2, 4 deg is printed; the negative angles are the same
    parts turned over.  Swapping the tab changes the trim to a fixed, printed value; no moving parts, no pins."""
    fc = cfg["fin"]
    out = dict(type=fc["type"], flags=[])
    if fc["type"] == "none":
        return out
    h, c0, tp, tf = float(fc["height_mm"]), float(fc["chord_mm"]), float(fc["taper"]), float(fc["thickness_mm"])
    ct = tp * c0
    cf = lambda u: c0 + (ct - c0) * u / h
    frac, span_frac = float(fc["tab_chord_frac"]), float(fc["tab_span_frac"])
    v_cut = c0 * (1 - frac)
    # keep the cut line straight at v_cut; the TE is c(u)
    ua, ub = h * (1 - span_frac) / 2, h * (1 + span_frac) / 2
    r, wn, ln = float(fc["key_head_r_mm"]), float(fc["key_neck_w_mm"]), float(fc["key_neck_len_mm"])
    flat = float(fc["tab_flat_mm"])
    clr = P["key_clr"]
    tab_chord = frac * c0
    span = ub - ua
    um = (ua + ub) / 2
    nkeys = 3 if span >= 4 * (2 * r + 2.0) else (2 if span >= 2 * (2 * r + 2.0) else 1)
    pitch = (span - 2 * (r + 1.2)) / max(nkeys - 1, 1) if nkeys > 1 else 0.0
    ukeys = [um + (i - (nkeys - 1) / 2) * pitch for i in range(nkeys)]
    tilt_len = tab_chord - flat
    out.update(h=h, c0=c0, ct=ct, tf=tf, v_cut=v_cut, ua=ua, ub=ub, ukeys=ukeys, tab_chord=tab_chord, tab_span=span, flat=flat, tilt_len=tilt_len, key=(r, wn, ln))
    ext = tab_extent(r, ln)
    if flat < ext + 1.0:
        flat = ext + 1.0
        out["flat"] = flat
        tilt_len = tab_chord - flat
        out["tilt_len"] = tilt_len
        out["flags"].append("tab flat landing raised to %.1f mm so it holds the keys" % flat)
    if tilt_len < 3.0:
        out["flags"].append("FLAG: trim tab cannot be made at this size: tab chord %.1f mm leaves only %.1f mm of tilted tab after the %.1f mm key landing (need >= 3 mm); no tab family written" % (tab_chord, tilt_len, flat))
        out["tab_feasible"] = False
    elif min(wn, 2 * r) < 0.8:
        out["flags"].append("FLAG: trim-tab key neck %.2f mm is below a printable wall" % wn)
        out["tab_feasible"] = False
    else:
        out["tab_feasible"] = True
    out["tab_angle_step_mm"] = tilt_len * math.tan(math.radians(2.0)) if tilt_len > 0 else 0.0
    return out


def fin_regions(fg, cfg, P, with_tab_cut, tongue=None, slots=None):
    """Body regions (u,v coordinates; thickness tf) of a fin.  with_tab_cut: TE notch with jigsaw tabs for the trim tab; else a full plate."""
    h, c0, tf = fg["h"], fg["c0"], fg["tf"]
    cf = lambda u: c0 + (fg["ct"] - c0) * u / h
    pts = [(0.0, 0.0)]
    if tongue:
        t0, t1, Lt = tongue
        pts += [(t0, 0.0), (t0, -Lt), (t1, -Lt), (t1, 0.0)]
    pts += [(h, 0.0), (h, cf(h))]
    if with_tab_cut:
        ua, ub, vc = fg["ua"], fg["ub"], fg["v_cut"]
        r, wn, ln = fg["key"]
        pts += [(ub, cf(ub)), (ub, vc)]
        for uk in sorted(fg["ukeys"], reverse=True):
            pts += tab_path(uk, vc, r, wn, ln, 0.0)
        pts += [(ua, vc), (ua, cf(ua))]
    pts += [(0.0, cf(0.0))]
    body = ccw([snap(p) for p in dedupe(pts)])
    holes = [ccw(s) for s in (slots or [])]
    return [dict(outer=body, holes=holes, zb=0.0, zt=tf)]


def tab_regions(fg, P, angle_deg):
    h, c0, tf = fg["h"], fg["c0"], fg["tf"]
    cf = lambda u: c0 + (fg["ct"] - c0) * u / h
    ua, ub, vc = fg["ua"], fg["ub"], fg["v_cut"]
    r, wn, ln = fg["key"]
    pts = [(ua, vc)]
    for uk in sorted(fg["ukeys"]):
        pts += list(reversed(tab_path(uk, vc, r, wn, ln, P["key_clr"])))
    pts += [(ub, vc), (ub, cf(ub)), (ua, cf(ua))]
    poly = ccw([snap(p) for p in dedupe(pts)])
    v_flat = vc + fg["flat"]
    tg = math.tan(math.radians(abs(angle_deg)))
    parts = split_by_line([poly], 1, v_flat) if abs(angle_deg) > 1e-9 else [poly]
    regs = []
    for p in parts:
        rv = rep_point(p)[1]
        if abs(angle_deg) > 1e-9 and rv > v_flat:
            regs.append(dict(outer=p, holes=[], zb=(lambda u, v, _t=tg, _v=v_flat: (v - _v) * _t), zt=(lambda u, v, _t=tg, _v=v_flat, _f=tf: (v - _v) * _t + _f)))
        else:
            regs.append(dict(outer=p, holes=[], zb=0.0, zt=tf))
    return regs, poly


def build_fins(cfg, P, wing, rail_info, notes):
    fg = fin_geometry(cfg, P, wing, rail_info)
    fc = cfg["fin"]
    parts = {}
    if fc["type"] == "none":
        return fg, parts
    for f in fg["flags"]:
        notes.append(f)
    tf, tm = fg["tf"], P["t_m"]
    tongue = slots = None
    if fc["type"] == "centre":
        slot = rail_info["fin_slot"]
        tl = slot["length"] - 0.3
        hz = rail_info["bot_poly"][-1], rail_info["top_poly"][-1]
        web_h = hz[1][1] - hz[0][1]
        pad = 0.8
        fg["tongue"] = (pad, max(web_h - pad, pad + 1.0), tl)
        fg["x_le"] = rail_info["x_max"]
        fg["u0_z"] = hz[0][1]
        tongue = fg["tongue"]
        if fg["tongue"][1] - fg["tongue"][0] < 2.0:
            notes.append("FLAG: web too shallow for the fin tongue")
        fins = {"fin_centre": dict(tab=fg["tab_feasible"])}
    else:
        tip_chord = P.get("tip_chord", fg["c0"])
        if abs(tip_chord - fg["c0"]) > 0.5:
            notes.append("FLAG: fin chord %.1f mm differs from the wing-tip chord %.1f mm; the tip tabs are placed by chord fraction" % (fg["c0"], tip_chord))
        tl = float(fc.get("wing_tab_len_frac", 0.2)) * fg["c0"]
        slots = []
        for f in fc.get("wing_tab_fracs", [0.25, 0.75]):
            vc = f * fg["c0"]
            uc = fg["h"] / 2
            sh = tm + float(fc["slot_clearance_mm"])
            slots.append([(uc - sh / 2, vc - tl / 2 - 0.05), (uc + sh / 2, vc - tl / 2 - 0.05), (uc + sh / 2, vc + tl / 2 + 0.05), (uc - sh / 2, vc + tl / 2 + 0.05)])
            if fg["tab_feasible"] and fc["tab_on"] != "none" and vc + tl / 2 + 0.05 > fg["v_cut"] - 1.0:
                raise ValueError("fin slot at %.0f %% chord runs into the trim-tab zone (starts at %.0f %%); change fin.wing_tab_fracs" % (100 * f, 100 * fg["v_cut"] / fg["c0"]))
        fg["slots"] = slots
        fins = {"fin_tip_R": dict(tab=fg["tab_feasible"] and fc["tab_on"] in ("right", "both")), "fin_tip_L": dict(tab=fg["tab_feasible"] and fc["tab_on"] in ("left", "both"))}
        if min(tl, fg["c0"]) < 2.0:
            notes.append("FLAG: tip-fin slots only %.1f mm long" % tl)
    for name, d in fins.items():
        regs = fin_regions(fg, cfg, P, d["tab"], tongue, slots)
        parts[name] = dict(tris=mesh_regions(regs), regions=regs, kind="fin", has_tab=d["tab"])
    if fg["tab_feasible"]:
        for a in [abs(x) for x in fc["tab_angles_deg"]]:
            if f"fin_tab_{a:g}" in parts:
                continue
            regs, poly = tab_regions(fg, P, a)
            parts[f"fin_tab_{a:g}"] = dict(tris=mesh_regions(regs), regions=regs, kind="tab", angle=a, outline=poly)
    return fg, parts


# =============================================================================================== mass, CG, ballast
def budget(parts, glue, target, station, rho_lead=11.3, fill_margin=0.25, aft_station=None):
    """parts: list of (name, mass_g, x_mm); glue: list of (name, mass_g, x).  All x in the same frame as target / station.
    Ballast (lead) at `station` moves the CG from the bare CG to `target`: m_b = M0 (target - cg0) / (station - target)."""
    allp = list(parts) + list(glue)
    M0 = sum(m for _, m, _ in allp)
    cg0 = sum(m * x for _, m, x in allp) / M0
    out = dict(M0=M0, cg0=cg0, target=target, station=station, rows=[(n, m, x) for n, m, x in allp])
    mb = M0 * (target - cg0) / (station - target) if abs(station - target) > 1e-9 else float("inf")
    out["ballast_g_raw"] = mb
    if mb < 0:
        out["ballast_g"] = 0.0
        out["cg_ahead_of_target_mm"] = target - cg0
        out["note"] = "bare CG is %.2f mm AHEAD of the target: no nose ballast; %.2f g at x=%.1f would be needed aft" % (target - cg0, M0 * (cg0 - target) / ((aft_station if aft_station is not None else target + 50) - target), aft_station if aft_station is not None else target + 50)
    else:
        out["ballast_g"] = mb
    out["allup_g"] = M0 + out["ballast_g"]
    out["lead_volume_mm3"] = out["ballast_g"] / rho_lead * 1000.0
    out["pocket_volume_mm3"] = out["lead_volume_mm3"] * (1.0 + fill_margin)
    return out


def planform_area(poly):
    return abs(parea(poly))


def area_left_of(poly, x):
    ps = split_by_line([poly], 0, x)
    return sum(abs(parea(p)) for p in ps if max(q[0] for q in p) <= x + 1e-6)


def build_kit(cfg, base_dir=None, log=print):
    """The whole build.  Returns a results dict (nothing is written here)."""
    t0 = __import__("time").time()
    notes = []
    span = float(cfg["span_mm"])
    outline = load_outline(cfg["source"], span, base_dir)
    for n in outline.get("notes", []):
        notes.append(n)
    wing = build_wing(cfg, outline, notes)
    P, half = wing["P"], wing["half"]
    rho = float(cfg["mass"]["pla_density_g_cm3"])
    lead = float(cfg["ballast"]["density_g_cm3"])
    fm = float(cfg["ballast"]["fill_margin"])
    rc = cfg["rail"]
    mcfg = cfg["mass"]
    # ---------------- target CG in the outline frame
    x_nose = outline["nose_x"]
    tgt_cfg = mcfg.get("target_cg_mm")
    if tgt_cfg is None:
        target = None
    elif mcfg.get("cg_reference", "nose_tip") == "nose_tip":
        target = x_nose + float(tgt_cfg)
    else:
        target = float(tgt_cfg)
    # ---------------- wing masses (right + left)
    wing_parts = []
    for pc in wing["pieces"]:
        vol = mesh_volume(pc["tris_halfsolid"])
        cx = mesh_centroid(pc["tris_halfsolid"])[0]
        pc["volume"], pc["cx"] = vol, cx
        wing_parts.append((pc, vol, cx))
    m_wings = [(f"wing piece {pc['index'] + 1} (R+L)", 2 * v * rho * 1e-3, cx) for pc, v, cx in wing_parts]
    # ---------------- rail + ballast iteration
    win_len, station = 0.0, None
    rail_shells = rail_info = None
    fins_geo, fin_parts = None, {}
    bud = None
    xp0_guess = chain_at_y(le_chain(half), P["sw"] / 2)[0] + 0.5
    for it in range(10):
        if station is None:
            station = float(rc["ballast_station_x_mm"]) if rc.get("ballast_station_x_mm") is not None else xp0_guess + 0.5 + max(win_len, 6.0) / 2
        if rc.get("ballast_station_x_mm") is None and win_len > 0:
            station = xp0_guess + 0.5 + win_len / 2
        rail_shells, rail_info = build_rail(cfg, P, wing, outline, win_len, station)
        fins_geo, fin_parts = build_fins(cfg, P, wing, rail_info, [])
        m_rail = rail_info["volume_mm3"] * rho * 1e-3
        parts = list(m_wings) + [("rail", m_rail, rail_info["cx"])]
        # fins
        fin_rows = []
        fcfg = cfg["fin"]
        if fin_parts:
            if fcfg["type"] == "centre":
                for nm, d in fin_parts.items():
                    if nm == "fin_centre":
                        v = mesh_volume(d["tris"])
                        cxv = mesh_centroid(d["tris"])[1]
                        fin_rows.append(("centre fin", v * rho * 1e-3, rail_info["x_max"] + cxv))
                tab = [d for nm, d in fin_parts.items() if d["kind"] == "tab"]
                if tab and fins_geo["tab_feasible"]:
                    m1 = mesh_volume(max(tab, key=lambda d: d["angle"])["tris"]) * rho * 1e-3
                    fin_rows.append(("trim tab (heaviest variant)", m1, rail_info["x_max"] + fins_geo["v_cut"] + fins_geo["tab_chord"] / 2))
            else:
                xtip = max(p[0] for p in half if p[1] >= P["half_span"] - 1e-6)
                xtip_le = min(p[0] for p in half if p[1] >= P["half_span"] - 1e-6)
                for nm in ("fin_tip_R", "fin_tip_L"):
                    d = fin_parts[nm]
                    v = mesh_volume(d["tris"])
                    cv = mesh_centroid(d["tris"])[1]
                    fin_rows.append((nm.replace("_", " "), v * rho * 1e-3, xtip_le + cv))
                tab = [d for nm, d in fin_parts.items() if d["kind"] == "tab"]
                if tab and fins_geo["tab_feasible"]:
                    m1 = mesh_volume(max(tab, key=lambda d: d["angle"])["tris"]) * rho * 1e-3
                    fin_rows.append(("trim tab (heaviest variant)", m1, xtip_le + fins_geo["v_cut"] + fins_geo["tab_chord"] / 2))
        parts += fin_rows
        # glue
        gl = [("root seam glue", float(mcfg["glue_root_g"]), rail_info["cx"])]
        for pc in wing["pieces"][1:]:
            gl.append((f"split joint {pc['index']}", float(mcfg["glue_per_split_g"]) * 2, (min(q[0] for q in pc["poly"]) + max(q[0] for q in pc["poly"])) / 2))
        if fcfg["type"] != "none":
            gl.append(("fin glue", float(mcfg["glue_per_fin_g"]) * (2 if fcfg["type"] == "tip" else 1), (fin_rows[0][2] if fin_rows else rail_info["cx"])))
        if target is None:
            break
        bud = budget(parts, gl, target, station, lead, fm, aft_station=rail_info["x_max"])
        new_len = bud["pocket_volume_mm3"] / (float(rc["window_height_mm"]) * float(rc["ballast_block_thickness_mm"])) if bud["ballast_g"] > 0 else 0.0
        if abs(new_len - win_len) < 0.02:
            break
        win_len = new_len
    if target is None:
        bud = budget(parts, gl, 0.0, -100.0, lead, fm)
        bud["ballast_g"] = 0.0
        bud["allup_g"] = bud["M0"]
    fins_geo, fin_parts = build_fins(cfg, P, wing, rail_info, notes)
    if bud.get("note"):
        notes.append(bud["note"])
    if win_len > 0:
        wl = rail_info["window"]
        if rail_info["win_station"] + win_len / 2 + float(rc["wall_mm"]) > (P["x_hinge"] - P["hz"] / 2 if P["hinge_active"] else rail_info["plate_x"][1]):
            notes.append("FLAG: ballast window runs past the wing-plate end; shorten by a taller window_height_mm or a thicker ballast block")
    # ---------------- areas, wing loading
    A_wing_half = area_left_of(half, P["x_hinge"]) if P["x_hinge"] is not None else planform_area(half)
    if P["hinge_active"] is False and P["x_hinge"] is not None:
        A_wing_half = planform_area(half)
    A_sheet = 2 * planform_area(half)
    A_wing = 2 * (area_left_of(half, P["x_hinge"]) if P["x_hinge"] is not None else planform_area(half))
    W = bud["allup_g"] * 1e-3 * 9.80665
    ws = W / (A_wing * 1e-6) if A_wing else None
    # ---------------- print parts, fit, placement
    bedsz, step = P["bed"], P["rot_step"]
    printed = []

    def add_part(name, tris, kind, volume_tris=None, note="", count=1):
        pts = [(p[0], p[1]) for t in tris for p in t]
        fit = fit_search(pts, bedsz, step)
        placed = place_on_bed(tris, fit["angle_deg"])
        lo, hi = tri_bbox(placed)
        chk = mesh_check(tris)
        printed.append(dict(name=name, kind=kind, tris=placed, tris_raw=tris, fit=fit, bbox_lo=lo, bbox_hi=hi, check=chk, volume=chk["volume"], mass_g=chk["volume"] * rho * 1e-3, note=note, count=count))
    mir_y = [[1, 0, 0], [0, -1, 0], [0, 0, 1]]
    for pc in wing["pieces"]:
        sfx = "" if len(wing["pieces"]) == 1 else f"_p{pc['index'] + 1}"
        pc["print_L"] = pc["tris_halfsolid"]                              # printed left half = half-solid as built (rotated 180 deg about x in flight)
        pc["print_R"] = affine(pc["tris_halfsolid"], mir_y)
        add_part(f"wing_R{sfx}", pc["print_R"], "wing")
        add_part(f"wing_L{sfx}", pc["print_L"], "wing")
    orient = rc["print_orientation"]
    add_part("rail", rail_print_mesh(rail_shells, orient), "rail", note="orientation: " + orient)
    for nm, d in fin_parts.items():
        add_part(nm, d["tris"], d["kind"])
    # sheet-only mass check number: STL volumes
    res = dict(cfg=cfg, outline=outline, wing=wing, P=P, rail_shells=rail_shells, rail=rail_info, fin_geo=fins_geo, fin_parts=fin_parts, budget=bud, printed=printed, notes=notes,
               target=target, x_nose=x_nose, win_len=win_len, station=station, A_wing=A_wing, A_sheet=A_sheet, wing_loading_Nm2=ws, W_N=W, parts_rows=parts, glue_rows=gl,
               rho=rho, seconds=__import__("time").time() - t0)
    return res


# =============================================================================================== 2D drawings: DXF layers per part
def dxf_wing_piece(res, pc):
    P = res["P"]
    items = [("OUTLINE", pc["poly"], True)]
    for r in pc["regions"]:
        if r["cls"] in ("S", "J", "B"):
            items.append(("RAISED_" + r["cls"], r["outer"], True))
    for k in pc["keys"]:
        items.append(("KEYS", k["poly"], True))
    if pc["is_root"]:
        items.append(("ROOT_EDGE", [(min(q[0] for q in pc["poly"]), 0.0), (max(q[0] for q in pc["poly"]), 0.0)], False))
        if P["hinge_active"]:
            items.append(("HINGE_FOLD", [(P["x_hinge"], 0.0), (P["x_hinge"], max(q[1] for q in pc["poly"]))], False))
    return dxf_text(items)


def dxf_full_planform(res):
    P, half = res["P"], res["wing"]["half"]
    right = ccw(half)
    left = ccw([(x, -y) for x, y in half])
    items = [("OUTLINE_RIGHT", right, True), ("OUTLINE_LEFT", left, True)]
    full = [p for p in right if abs(p[1]) > 1e-9] + [(x, -y) for x, y in reversed(right) if abs(y) > 1e-9]
    items.append(("CENTRELINE", [(res["x_nose"] - 5, 0.0), (max(p[0] for p in half) + 5, 0.0)], False))
    if P["hinge_active"]:
        items.append(("HINGE_FOLD", [(P["x_hinge"], -P["half_span"]), (P["x_hinge"], P["half_span"])], False))
    if res["target"] is not None:
        items.append(("CG_TARGET", [(res["target"], -6.0), (res["target"], 6.0)], False))
    for sgn in (1, -1):
        items.append(("ROOT_STRIP_EDGE", [(min(p[0] for p in half), sgn * P["sw"]), (max(p[0] for p in half), sgn * P["sw"])], False))
    return dxf_text(items)


def dxf_rail_side(res):
    ri = res["rail"]
    items = [("OUTLINE", list(ri["bot_poly"]) + list(reversed(ri["top_poly"])), True)]
    for h in ri["holes"]:
        items.append(("BALLAST_WINDOW", h, True))
    if ri["fin_slot"]:
        x0 = ri["fin_slot"]["x0"]
        items.append(("FIN_SLOT_START", [(x0, ri["bot_poly"][-1][1]), (x0, ri["top_poly"][-1][1])], False))
    return dxf_text(items)


def dxf_rail_plates(res):
    ri, P = res["rail"], res["P"]
    items = []
    X0, X1 = ri["plate_x"]
    S0, S1 = ri["S_range"]
    items.append(("PLATE_WING", [(X0, S0), (X1, S0), (X1, S1), (X0, S1)], True))
    for sk in ri["sockets_wing"]:
        items.append(("SOCKETS", sk, True))
    for l in ri["lightening_wing"]:
        items.append(("LIGHTENING", l, True))
    if ri["hinge"]:
        off = ri["xh"] + 0.0
        a, b = ri["tail_plate_X"]
        items.append(("PLATE_TAIL_(X_from_hinge,_shown_at_hinge)", [(off + a, S0), (off + b, S0), (off + b, S1), (off + a, S1)], True))
        for sk in ri["sockets_tail"]:
            items.append(("SOCKETS", [(off + x, y) for x, y in sk], True))
        for l in ri["lightening_tail"]:
            items.append(("LIGHTENING", [(off + x, y) for x, y in l], True))
    return dxf_text(items)


def dxf_fin(res, name, d):
    items = []
    for r in d["regions"]:
        items.append(("OUTLINE", [(v, u) for u, v in r["outer"]], True))
        for h in r["holes"]:
            items.append(("SLOTS", [(v, u) for u, v in h], True))
    return dxf_text(items)


# =============================================================================================== foam kit: tiled 1:1 white print templates
PAPERS = {"A4": (210.0, 297.0), "LETTER": (215.9, 279.4)}


def _svg_head(w, h):
    return '<svg xmlns="http://www.w3.org/2000/svg" width="%.2fmm" height="%.2fmm" viewBox="0 0 %.2f %.2f" font-family="Helvetica,Arial,sans-serif">\n' % (w, h, w, h)


def tile_pages(items, bbox, paper, overlap, margin, title, texts=(), pad=4.0):
    """items: list of (kind, pts, closed, style) in page-plane mm (x right, y down).  Returns a list of SVG strings (white background, black ink, 1:1)."""
    pw, ph = PAPERS[paper.upper()]
    x0, y0, x1, y1 = bbox
    x0, y0, x1, y1 = x0 - pad, y0 - pad, x1 + pad, y1 + pad
    W, H = x1 - x0, y1 - y0
    best = None
    for landscape in (False, True):
        w, h = (ph, pw) if landscape else (pw, ph)
        uw, uh = w - 2 * margin, h - 2 * margin - 14.0          # 14 mm strip for the scale bar and title
        nx = max(1, int(math.ceil((W - overlap) / (uw - overlap)))) if W > uw else 1
        ny = max(1, int(math.ceil((H - overlap) / (uh - overlap)))) if H > uh else 1
        cand = (nx * ny, nx + ny, landscape, w, h, uw, uh, nx, ny)
        if best is None or cand[:2] < best[:2]:
            best = cand
    _, _, landscape, w, h, uw, uh, nx, ny = best
    pages = []
    for j in range(ny):
        for i in range(nx):
            tx = x0 + i * (uw - overlap)
            ty = y0 + j * (uh - overlap)
            s = [_svg_head(w, h), '<rect x="0" y="0" width="%.2f" height="%.2f" fill="#ffffff"/>\n' % (w, h)]
            s.append('<defs><clipPath id="c"><rect x="%.2f" y="%.2f" width="%.2f" height="%.2f"/></clipPath></defs>\n' % (margin, margin + 8, uw, uh))
            s.append('<g clip-path="url(#c)"><g transform="translate(%.4f,%.4f)">\n' % (margin - tx, margin + 8 - ty))
            for kind, pts, closed, style in items:
                col = style.get("color", "#000")
                dash = ' stroke-dasharray="%s"' % style["dash"] if style.get("dash") else ""
                sw = style.get("w", 0.35)
                if kind == "poly":
                    d = "M " + " L ".join("%.3f,%.3f" % p for p in pts) + (" Z" if closed else "")
                    s.append('<path d="%s" fill="none" stroke="%s" stroke-width="%.2f"%s/>\n' % (d, col, sw, dash))
                elif kind == "text":
                    s.append('<text x="%.2f" y="%.2f" font-size="%.1f" fill="%s">%s</text>\n' % (pts[0][0], pts[0][1], style.get("size", 3.2), col, html.escape(style["text"])))
            s.append("</g></g>\n")
            # registration marks at the corners of the usable area
            for cx_, cy_ in ((margin, margin + 8), (margin + uw, margin + 8), (margin, margin + 8 + uh), (margin + uw, margin + 8 + uh)):
                s.append('<g stroke="#000" stroke-width="0.25" fill="none"><line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f"/><line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f"/><circle cx="%.2f" cy="%.2f" r="1.6"/></g>\n' % (cx_ - 3, cy_, cx_ + 3, cy_, cx_, cy_ - 3, cx_, cy_ + 3, cx_, cy_))
            # overlap guides (dotted) so neighbouring tiles can be matched
            if nx > 1 or ny > 1:
                s.append('<rect x="%.2f" y="%.2f" width="%.2f" height="%.2f" fill="none" stroke="#888" stroke-width="0.15" stroke-dasharray="1 2"/>\n' % (margin + overlap / 2, margin + 8 + overlap / 2, uw - overlap, uh - overlap))
            s.append('<text x="%.2f" y="%.2f" font-size="3.4" fill="#000">%s - tile %d/%d (column %d of %d, row %d of %d) - print at 100 %% (no fit-to-page); %s</text>\n'
                     % (margin, margin + 4.5, html.escape(title), j * nx + i + 1, nx * ny, i + 1, nx, j + 1, ny, paper.upper()))
            # 50 mm scale bar
            bx, by = margin, h - margin - 2.5
            s.append('<g stroke="#000" stroke-width="0.3"><line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f"/>' % (bx, by, bx + 50, by))
            for t in range(0, 51, 10):
                s.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f"/>' % (bx + t, by - 1.5, bx + t, by + 1.5))
            s.append('</g><text x="%.2f" y="%.2f" font-size="3.2">50 mm scale bar - measure it: it must read 50.0 mm</text>\n' % (bx + 54, by + 1.1))
            s.append("</svg>\n")
            pages.append("".join(s))
    return pages, dict(paper=paper.upper(), landscape=landscape, nx=nx, ny=ny, pages=nx * ny)


def foam_items(res):
    """Planform of the whole glider in page coordinates: page x = y (span, right wing on the right), page y = x (aft, nose at the top)."""
    P, half = res["P"], res["wing"]["half"]
    to = lambda pts: [(y, x) for x, y in pts]
    right = ccw(half)
    left = ccw([(x, -y) for x, y in half])
    # one outline: right half boundary + left half boundary without the shared root chord
    full = [p for p in right if abs(p[1]) > 1e-9 or True]
    items = [("poly", to(ccw(right)), True, dict(w=0.4)), ("poly", to(ccw(left)), True, dict(w=0.4))]
    xs = [p[0] for p in half]
    xmin, xmax = min(xs), max(xs)
    ymax = P["half_span"]
    items.append(("poly", [(0.0, xmin - 3), (0.0, xmax + 3)], False, dict(dash="4 2", w=0.2, color="#555")))
    if P["hinge_active"]:
        items.append(("poly", [(-ymax, P["x_hinge"]), (ymax, P["x_hinge"])], False, dict(dash="2 1.5", w=0.3, color="#555")))
        items.append(("text", [(ymax * 0.55, P["x_hinge"] - 1.5)], False, dict(text="hinge / tail fold line (bend %.1f deg UP)" % P["bend"], size=3.0)))
    for sg in (1, -1):
        items.append(("poly", [(sg * P["sw"], xmin), (sg * P["sw"], xmax)], False, dict(dash="1 2", w=0.2, color="#777")))
    if res["target"] is not None:
        t = res["target"]
        items.append(("poly", [(-8, t), (8, t)], False, dict(w=0.3)))
        items.append(("poly", [(0, t - 8), (0, t + 8)], False, dict(w=0.3)))
        items.append(("text", [(9, t + 1.0)], False, dict(text="CG target %.1f mm aft of the nose tip" % (t - res["x_nose"]), size=3.0)))
    br = wing_bead_polyline(res)
    if br:
        for sg in (1, -1):
            items.append(("poly", [(sg * y, x) for x, y in br], False, dict(dash="3 1.5", w=0.25, color="#333")))
    return items, (-ymax, xmin, ymax, max(xmax, xmin + 1.0))


def wing_bead_polyline(res):
    bd = res["wing"]["bead"]
    return bd["inner"] if bd else None


def rail_foam_items(res):
    """Side profile of the rail (x right, z up -> page y down) for a stick cut from balsa or foam core."""
    ri = res["rail"]
    zt = max(z for _, z in ri["top_poly"])
    flip = lambda pts: [(x, zt - z) for x, z in pts]
    outer = flip(list(ri["bot_poly"]) + list(reversed(ri["top_poly"])))
    items = [("poly", outer, True, dict(w=0.4))]
    for h in ri["holes"]:
        items.append(("poly", flip(h), True, dict(w=0.3)))
    xs = [p[0] for p in outer]
    ys = [p[1] for p in outer]
    for x in range(int(math.floor(min(xs) / 10.0)) * 10, int(max(xs)) + 1, 10):
        if min(xs) <= x <= max(xs):
            items.append(("poly", [(x, max(ys)), (x, max(ys) + 2.5)], False, dict(w=0.25)))
            items.append(("text", [(x - 2, max(ys) + 6)], False, dict(text="%d" % x, size=2.6)))
    return items, (min(xs), min(ys), max(xs), max(ys) + 8)


# =============================================================================================== HTML preview (dark) and summary
CSS = """:root{color-scheme:dark;--bg:#14181d;--panel:#1c2229;--line:#333c46;--ink:#dde3ea;--muted:#9aa7b4;--link:#7fb4ff;--accent:#e89a55}
body{background:var(--bg);color:var(--ink);font:14px/1.5 Segoe UI,Arial,sans-serif;margin:0;padding:18px 28px;max-width:1200px}
h1,h2,h3{color:#fff}h2{border-bottom:1px solid var(--line);padding-bottom:4px;margin-top:30px}a{color:var(--link)}
table{border-collapse:collapse;margin:8px 0}th,td{border:1px solid var(--line);padding:3px 9px;text-align:right}th{background:#262e37}td:first-child,th:first-child{text-align:left}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:6px;padding:10px 14px;margin:10px 0}.flag{color:#ff8a80}.ok{color:#6cc08b}.muted{color:var(--muted)}
svg{background:#10141a;border:1px solid var(--line);border-radius:4px;max-width:100%}code{color:var(--accent)}"""

CLS_COL = {"R": "#35506b", "S": "#c58a3a", "J": "#8a6ad6", "B": "#4aa377"}


def _fit_svg(polys, w=760, pad=12, extra_h=0):
    allp = [p for pl in polys for p in pl["pts"]]
    x0, y0, x1, y1 = bbox2(allp)
    s = (w - 2 * pad) / max(x1 - x0, 1e-6)
    h = (y1 - y0) * s + 2 * pad + extra_h
    return x0, y0, s, w, h, pad


def svg_plan(polys, w=760, labels=(), title=""):
    """polys: list of dict(pts, fill, stroke, w).  Plan view with x aft DOWN the page and y to the right."""
    q = [dict(pl, pts=[(y, x) for x, y in pl["pts"]]) for pl in polys]
    x0, y0, s, W, H, pad = _fit_svg(q, w)
    out = [('<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d" font-family="Segoe UI,Arial">' % (W, H, W, H))]
    for pl in q:
        d = "M " + " L ".join("%.2f,%.2f" % (pad + (a - x0) * s, pad + (b - y0) * s) for a, b in pl["pts"]) + (" Z" if pl.get("closed", True) else "")
        out.append('<path d="%s" fill="%s" stroke="%s" stroke-width="%.2f"%s/>' % (d, pl.get("fill", "none"), pl.get("stroke", "#9aa7b4"), pl.get("w", 0.8), (' stroke-dasharray="%s"' % pl["dash"]) if pl.get("dash") else ""))
    for (x, y, t) in labels:
        out.append('<text x="%.1f" y="%.1f" fill="#9aa7b4" font-size="11">%s</text>' % (pad + (y - x0) * s, pad + (x - y0) * s, html.escape(t)))
    out.append('<text x="6" y="%d" fill="#9aa7b4" font-size="11">%s</text></svg>' % (H - 4, html.escape(title)))
    return "".join(out)


def svg_xz(polys, w=760, title="", flipz=True):
    q = [dict(pl, pts=[(x, -z) for x, z in pl["pts"]]) for pl in polys]
    x0, y0, s, W, H, pad = _fit_svg(q, w)
    out = [('<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d" font-family="Segoe UI,Arial">' % (W, H + 14, W, H + 14))]
    for pl in q:
        d = "M " + " L ".join("%.2f,%.2f" % (pad + (a - x0) * s, pad + (b - y0) * s) for a, b in pl["pts"]) + (" Z" if pl.get("closed", True) else "")
        out.append('<path d="%s" fill="%s" stroke="%s" stroke-width="%.2f"%s/>' % (d, pl.get("fill", "none"), pl.get("stroke", "#9aa7b4"), pl.get("w", 0.8), (' stroke-dasharray="%s"' % pl["dash"]) if pl.get("dash") else ""))
    out.append('<text x="6" y="%d" fill="#9aa7b4" font-size="11">%s</text></svg>' % (H + 10, html.escape(title)))
    return "".join(out)


def section_profile(regions, heights, x, ys):
    """Total thickness h(y) along the line x = const, for a stepped drawing (regions: list of dict(outer, holes, cls))."""
    out = []
    for y in ys:
        h = None
        for r in regions:
            if point_in_poly((x, y), r["outer"]) and not any(point_in_poly((x, y), hh) for hh in r["holes"]):
                hv = heights[r["cls"]]
                h = hv(x, y) if callable(hv) else hv
                break
        out.append(h)
    return out


def page_wing_preview(res, pc):
    P = res["P"]
    polys = []
    for r in pc["regions"]:
        polys.append(dict(pts=r["outer"], fill=CLS_COL[r["cls"]], stroke="#10141a", w=0.4))
    for k in pc["keys"]:
        polys.append(dict(pts=k["poly"], fill="#ffd166", stroke="#fff", w=0.6))
    polys.append(dict(pts=pc["poly"], fill="none", stroke="#dde3ea", w=1.0))
    plan = svg_plan(polys, title="plan view (flight planform, right wing, x aft down): R membrane, S root strip, B bead, J joint strip, yellow = jigsaw keys")
    # span-wise section at x = mid root chord
    heights = region_heights(P, res["wing"]["bead"])
    xs = (min(q[0] for q in pc["poly"]) + max(q[0] for q in pc["poly"])) / 2
    xs = min(max(xs, P["sw"]), xs)
    ymax = max(q[1] for q in pc["poly"])
    ys = [pc["y_lo"] + (ymax - pc["y_lo"]) * i / 400 for i in range(401)]
    xsec = res["wing"]["half"][0][0] if False else (P["x_hinge"] - 12 if P["hinge_active"] else sum(q[0] for q in pc["poly"]) / len(pc["poly"]))
    hs = section_profile(pc["regions"], heights, xsec, ys)
    pts = []
    zk = 6.0
    for y, h in zip(ys, hs):
        if h is not None:
            pts.append((y, h * zk))
    sec = ""
    if pts:
        poly = [(y, 0.0) for y, _ in pts] and ([(pts[0][0], 0.0)] + pts + [(pts[-1][0], 0.0)])
        sec = svg_xz([dict(pts=[(a, -b) for a, b in poly], fill="#35506b", stroke="#dde3ea", w=0.8)], w=760,
                     title="section along y at x=%.1f mm (thickness x%g, flat face = top of the wing in flight, bead/strip hang below)" % (xsec, zk))
    return plan, sec


def page_rail_preview(res):
    ri = res["rail"]
    side = [dict(pts=list(ri["bot_poly"]) + list(reversed(ri["top_poly"])), fill="#35506b", stroke="#dde3ea")]
    for h in ri["holes"]:
        side.append(dict(pts=h, fill="#10141a", stroke="#e89a55"))
    F = ri["frame"]
    # plates in side view (profile at y = S0): wing plate band and tail plate band, drawn as the bottom/top edges
    sv = svg_xz(side, title="rail side elevation (x aft right, z up): web profile, ballast window (orange)")
    # cross-sections
    secs = []
    for label, x in (("through the wing plate", (ri["plate_x"][0] + ri["plate_x"][1]) / 2), ("through the ballast window" if ri["window"] else None, (ri["win_station"]))):
        if label is None:
            continue
        th = ri["block_thickness"] if (ri["window"] and ri["x_cut_block"] and ri["x_cut_block"][0] <= x <= ri["x_cut_block"][1]) else ri["web_thickness"]
        zt, zb = _poly_z(ri["top_poly"], x), _poly_z(ri["bot_poly"], x)
        polys = [dict(pts=[(-th / 2, zt), (th / 2, zt), (th / 2, zb), (-th / 2, zb)], fill="#35506b", stroke="#dde3ea")]
        if ri["plate_x"][0] <= x <= ri["plate_x"][1]:
            for sg in (1, -1):
                S0, S1, hh = ri["S_range"][0], ri["S_range"][1], ri["hh"]
                q = [(S0, 0.0), (S1, 0.0), (S1, -hh), (S0, -hh)]
                pts = [(sg * (S * F["c"] - N * F["s"]), S * F["s"] + N * F["c"]) for S, N in q]
                polys.append(dict(pts=pts, fill="#c58a3a", stroke="#dde3ea"))
        secs.append(svg_xz(polys, w=420, title="section %s at x=%.1f mm (y across, z up; dihedral %.1f deg)" % (label, x, math.degrees(F["delta"]))))
    return sv, secs


def page_fin_preview(res):
    out = []
    for nm, d in res["fin_parts"].items():
        polys = []
        for r in d["regions"]:
            polys.append(dict(pts=[(v, u) for u, v in r["outer"]], fill="#35506b", stroke="#dde3ea"))
            for hh in r["holes"]:
                polys.append(dict(pts=[(v, u) for u, v in hh], fill="#10141a", stroke="#e89a55"))
        out.append((nm, svg_xz([dict(pts=[(x, -y) for x, y in pl["pts"]], fill=pl["fill"], stroke=pl["stroke"]) for pl in polys], w=360, title=nm + " (print plan: chord right, height up)")))
    return out


# =============================================================================================== reference check (swiftA_printed400_options.py rows)
def reference_rows():
    """Rows of results/runs/rigid_glider/swiftA_printed400_options/options.csv by option name -> dict of the numbers used here (None if the file is missing)."""
    import csv
    f = ROOT / "results/runs/rigid_glider/swiftA_printed400_options/options.csv"
    if not f.exists():
        return None
    rows = {}
    with f.open(newline="", encoding="utf-8") as fh:
        for r in csv.reader(fh):
            if len(r) > 16 and r[0] in ("A", "B", "C-h3.0"):
                rows[r[0]] = dict(membrane_t=float(r[2]), membrane_g=float(r[5]), rail_g=float(r[8]), glue_g=float(r[9]), M0=float(r[10]), cg0=float(r[11]), target=float(r[12]),
                                  station=float(r[13]), ballast_g=float(r[14]), allup_g=float(r[15]))
    return rows


def reference_check(tol=0.02):
    """Plain printed sheet (no bead, no strip) of the Swift A 400 outline at 0.6 / 0.4 mm through THIS builder's mesh + mass/CG/ballast code, with the options script's own
    rail mass (1.901 g, foam-400 rail), glue+splice (0.503 g), trim CG 36.966 mm and ballast station -12.5 mm.  Returns a list of comparison rows."""
    ref = reference_rows()
    if not ref:
        return None
    import bird_outline_v2 as v2
    import bird_outline_v4_swift as sw
    g = sw.build_geometry4(sw.GLIDE)
    k = 400.0
    rail = v2.make_rail(g["x_h"] * k - v2.RAIL_HINGE_GAP_MM, 30.0, 45.0)
    rows = []
    for key, nl in (("A", 6), ("B", 4)):
        r = ref[key]
        cfg = load_config({"name": "refcheck", "span_mm": 400, "tail": {"bend_deg": 0.0}, "sheet": {"layer_height_mm": 0.1, "n_layers": nl}, "bead": {"enabled": False},
                           "root_strip": {"width_mm": 0.0}})
        outline = load_outline(cfg["source"], 400.0)
        half = outline["half"]
        t_m = 0.1 * nl
        tris = mesh_regions([dict(outer=half, holes=[], zb=0.0, zt=t_m)])
        vol = mesh_volume(tris)
        cxs = mesh_centroid(tris)[0]
        m_sheet = 2 * vol * 1.24e-3
        parts = [("sheet", m_sheet, cxs), ("rail (reference foam-400 rail)", r["rail_g"], rail["cx"])]
        bud = budget(parts, [("glue + splice (reference)", r["glue_g"], rail["cx"])], r["target"], r["station"])
        for name, mine, theirs in (("sheet mass g", m_sheet, r["membrane_g"]), ("bare mass g", bud["M0"], r["M0"]), ("bare CG mm", bud["cg0"], r["cg0"]),
                                   ("ballast g at the -12.5 mm hole", bud["ballast_g"], r["ballast_g"]), ("all-up g", bud["allup_g"], r["allup_g"])):
            rows.append(dict(option=key, quantity=name, builder=mine, reference=theirs, rel_diff=(mine - theirs) / theirs, ok=abs(mine - theirs) / theirs <= tol))
    return rows


# =============================================================================================== writing the kit
def safe(s):
    return re.sub(r"[^A-Za-z0-9_.-]", "_", s)


def write_kit(res, out: Path, log=print):
    cfg, P = res["cfg"], res["P"]
    for sub in ("parts", "dxf", "foam"):
        (out / sub).mkdir(parents=True, exist_ok=True)
    files = []
    for pt in res["printed"]:
        fn = "parts/%s.stl" % safe(pt["name"])
        write_stl(out / fn, pt["tris"], pt["name"])
        pt["file"] = fn
        files.append(fn)
    # DXF
    wing = res["wing"]
    (out / "dxf/glider_planform.dxf").write_text(dxf_full_planform(res), encoding="ascii")
    files.append("dxf/glider_planform.dxf")
    for pc in wing["pieces"]:
        sfx = "" if len(wing["pieces"]) == 1 else "_p%d" % (pc["index"] + 1)
        fn = "dxf/wing_R%s_planform.dxf" % sfx
        (out / fn).write_text(dxf_wing_piece(res, pc), encoding="ascii")
        files.append(fn)
    (out / "dxf/rail_side_profile.dxf").write_text(dxf_rail_side(res), encoding="ascii")
    (out / "dxf/rail_plates_X_S.dxf").write_text(dxf_rail_plates(res), encoding="ascii")
    files += ["dxf/rail_side_profile.dxf", "dxf/rail_plates_X_S.dxf"]
    for nm, d in res["fin_parts"].items():
        fn = "dxf/%s.dxf" % safe(nm)
        (out / fn).write_text(dxf_fin(res, nm, d), encoding="ascii")
        files.append(fn)
    # foam templates
    fo = cfg["foam"]
    items, bb = foam_items(res)
    pages, lay = tile_pages(items, bb, fo["paper"], float(fo["tile_overlap_mm"]), float(fo["margin_mm"]), cfg["name"] + " foam template (cut outline, dashed = folds/guides)")
    for i, pg in enumerate(pages):
        fn = "foam/template_p%d.svg" % (i + 1)
        (out / fn).write_text(pg, encoding="utf-8")
        files.append(fn)
    res["foam_layout"] = lay
    ri_items, ri_bb = rail_foam_items(res)
    pages2, lay2 = tile_pages(ri_items, ri_bb, fo["paper"], float(fo["tile_overlap_mm"]), float(fo["margin_mm"]), cfg["name"] + " rail side profile (stick/cut pattern)")
    for i, pg in enumerate(pages2):
        fn = "foam/rail_profile_p%d.svg" % (i + 1)
        (out / fn).write_text(pg, encoding="utf-8")
        files.append(fn)
    res["foam_rail_layout"] = lay2
    # summary
    summ = build_summary(res)
    (out / "summary.json").write_text(json.dumps(summ, indent=1, default=str), encoding="utf-8")
    files.append("summary.json")
    (out / "README.md").write_text(readme_text(res, summ), encoding="utf-8")
    (out / "index.html").write_text(index_html(res, summ), encoding="utf-8")
    files += ["README.md", "index.html"]
    return files


def T(value, tag, unit=""):
    return dict(value=value, tag=tag, unit=unit)


def build_summary(res):
    cfg, P, bud, ri = res["cfg"], res["P"], res["budget"], res["rail"]
    parts = []
    for pt in res["printed"]:
        parts.append(dict(name=pt["name"], file=pt.get("file"), kind=pt["kind"], volume_mm3=T(pt["volume"], "CALCULATED", "mm3"), mass_g=T(pt["mass_g"], "CALCULATED", "g"),
                          bbox_mm=[round(pt["bbox_hi"][i] - pt["bbox_lo"][i], 3) for i in range(3)], fits_bed=pt["fit"]["fits"], rotate_deg=pt["fit"]["angle_deg"],
                          bed_footprint_mm=[round(pt["fit"]["width"], 2), round(pt["fit"]["height"], 2)], manifold_ok=pt["check"]["ok"], triangles=pt["check"]["n_tris"], note=pt["note"]))
    mass = dict(rows=[dict(item=n, mass_g=T(m, "CALCULATED", "g"), x_mm_aft_of_root_LE=T(x, "CALCULATED", "mm")) for n, m, x in res["parts_rows"] + res["glue_rows"]],
                bare_mass_g=T(bud["M0"], "CALCULATED", "g"), bare_cg_mm=T(bud["cg0"], "CALCULATED", "mm aft of root LE"),
                target_cg_mm=T(res["target"], "KAI" if res["target"] is not None else "UNKNOWN", "mm aft of root LE"), ballast_station_mm=T(res["station"], "ASSUMED", "mm aft of root LE"),
                ballast_g=T(bud["ballast_g"], "CALCULATED", "g"), allup_g=T(bud["allup_g"], "CALCULATED", "g"), wing_area_mm2=T(res["A_wing"], "GEOMETRY", "mm2"),
                wing_loading_N_m2=T(res["wing_loading_Nm2"], "CALCULATED", "N/m2"), lead_density=T(float(cfg["ballast"]["density_g_cm3"]), "ASSUMED", "g/cm3"),
                pla_density=T(res["rho"], "ASSUMED", "g/cm3"), pocket_volume_mm3=T(bud["pocket_volume_mm3"], "CALCULATED", "mm3"), fill_margin=T(float(cfg["ballast"]["fill_margin"]), "ASSUMED"))
    geo = dict(span_mm=T(float(cfg["span_mm"]), "KAI", "mm"), half_span_mm=T(P["half_span"], "GEOMETRY", "mm"), membrane_mm=T(P["t_m"], "KAI", "mm"), n_layers=P["n_layers"], layer_height_mm=P["layer"],
               root_strip_width_mm=T(P["sw"], "ASSUMED", "mm"), root_strip_thickness_mm=T(P["Hs"], "ASSUMED", "mm"), key_clearance_mm=T(P["key_clr"], "ASSUMED", "mm"),
               bead=(dict(width_root=P["bead_w0"], width_tip=P["bead_w1"], height_root=P["bead_h0"], height_tip=P["bead_h1"], stop_mm=P["bead_stop"], y_start_mm=P["sw"]) if P["bead_on"] else None),
               dihedral_deg=T(P["dihedral"], "KAI", "deg"), tail_bend_deg=T(P["bend"], "KAI", "deg"), x_hinge_mm=P["x_hinge"], root_gap_mm=T(P["root_gap"], "ASSUMED", "mm"),
               root_gap_convergence_mm=P["root_gap_conv"], pieces=len(res["wing"]["pieces"]), cut_y_mm=[pc["cut_hi"] for pc in res["wing"]["pieces"] if pc["cut_hi"] is not None],
               unsplit_half_fits_bed=res["wing"]["fit_unsplit"]["fits"], key_count_root=len(res["wing"]["pieces"][0]["keys"]))
    rail = dict(length_mm=ri["x_max"] - ri["x_min"], x_range_mm=[ri["x_min"], ri["x_max"]], plate_thickness_mm=ri["hh"], pocket_depth_mm=ri["depth"], web_thickness_mm=ri["web_thickness"],
                ballast_block_mm=ri["block_thickness"], window=ri["window"], fin_slot=ri["fin_slot"], union_volume_mm3=T(ri["volume_mm3"], "CALCULATED", "mm3"), overlap_mm3=ri["overlap_mm3"],
                print_orientation=cfg["rail"]["print_orientation"], dihedral_face_deg=P["dihedral"], tail_face_deg=P["bend"] if P["hinge_active"] else 0.0, sockets=ri["n_sockets"])
    out = dict(name=cfg["name"], generated=__import__("datetime").datetime.now().isoformat(timespec="seconds"), seconds=round(res["seconds"], 2), geometry=geo, rail=rail, parts=parts, mass=mass,
               fin=(dict(type=cfg["fin"]["type"], tab_feasible=res["fin_geo"].get("tab_feasible"), tab_chord_mm=res["fin_geo"].get("tab_chord"), tilt_len_mm=res["fin_geo"].get("tilt_len"),
                         step_per_2deg_mm=res["fin_geo"].get("tab_angle_step_mm")) if res["fin_geo"] else None),
               outline=dict(source=res["outline"].get("source"), kind=res["outline"].get("kind", "builtin"), asymmetry_max_mm=res["outline"].get("asymmetry_max_mm"), asymmetry_mean_mm=res["outline"].get("asymmetry_mean_mm"),
                            builtin=res["outline"].get("builtin"), entity_counts=res["outline"].get("entity_counts")),
               flags=[n for n in res["notes"] if n.startswith("FLAG")], notes=[n for n in res["notes"] if not n.startswith("FLAG")], foam=dict(templates=res["foam_layout"], rail=res["foam_rail_layout"]), config=cfg)
    return out


def readme_text(res, S):
    cfg, P, bud, ri = res["cfg"], res["P"], res["budget"], res["rail"]
    L = []
    a = L.append
    a("# %s - sheet-builder kit\n" % cfg["name"])
    a("Generated %s by `rigid_glider/sheet_builder.py` from `%s` (builtin outline: %s). Nothing here is flight-validated; every number is tagged MODEL / ASSUMED / CALCULATED / GEOMETRY / KAI.\n"
      % (S["generated"], cfg["name"] + ".json", S["outline"].get("builtin") or S["outline"].get("source")))
    a("## What is in the folder\n")
    a("- `parts/*.stl` - print files (already rotated onto the bed; %g mm bed). `dxf/*.dxf` - 2D outlines in mm for an Onshape sketch import. `foam/*.svg` - 1:1 white print templates. `summary.json`, `index.html` (dark previews).\n" % P["bed"])
    a("## Settings used\n")
    a("| item | value | tag |\n|---|---|---|")
    rows = [("full span", "%g mm" % cfg["span_mm"], "KAI"), ("membrane", "%d x %.2f mm = %.2f mm" % (P["n_layers"], P["layer"], P["t_m"]), "KAI (parameter by size)"),
            ("dihedral (rail faces)", "%.2f deg per half" % P["dihedral"], "KAI"), ("tail bend (rail face)", "%.2f deg" % (P["bend"] if P["hinge_active"] else 0.0), "KAI"),
            ("root strip", "%.1f mm wide x %.2f mm thick" % (P["sw"], P["Hs"]), "ASSUMED"),
            ("LE bead", ("%.1f -> %.1f mm wide, %.2f -> %.2f mm high above the membrane, stops at %.0f %% of the half span (rectangular section)" % (P["bead_w0"], P["bead_w1"], P["bead_h0"], P["bead_h1"], 100 * P["bead_stop"] / P["half_span"])) if P["bead_on"] else "none", "ASSUMED"),
            ("jigsaw root keys", "%d keys, head r %.2f, neck %.2f x %.2f, post %.2f mm high; socket = key + %.2f mm on every edge, %.2f mm deeper" % (S["geometry"]["key_count_root"], P["key_r"], P["key_wn"], P["key_ln"], P["kh"], P["key_clr"], P["key_clr"]), "ASSUMED"),
            ("root gap between halves", "%.2f mm%s" % (P["root_gap"], (" (raised from %.2f: tail bend x dihedral converges the root edges by %.2f mm)" % (P["root_gap_req"], P["root_gap_conv"])) if P["root_gap"] > P["root_gap_req"] + 1e-9 else ""), "ASSUMED"),
            ("PLA density / lead density", "%.2f / %.1f g/cm3, pocket fill margin %.0f %%" % (res["rho"], float(cfg["ballast"]["density_g_cm3"]), 100 * float(cfg["ballast"]["fill_margin"])), "ASSUMED"),
            ("bed", "%g x %g mm (design); rotation search every %.1f deg" % (P["bed"], P["bed"], P["rot_step"]), "KAI")]
    for r in rows:
        a("| %s | %s | %s |" % r)
    a("\nEvery setting is described in `rigid_glider/sheet_builder_configs/README.md`.\n")
    a("## Design\n")
    a("**Wing halves.** Two mirrored halves, each printed with its FLAT face on the bed (this face is the top of the wing). The leading-edge bead, the root strip and the jigsaw posts stand up from the "
      "flat face on the print and hang BELOW the wing in flight, toward the rail. The printed right-wing half therefore has the mirrored planform. One unioned closed shell per part "
      "(raised regions are extra height of the same extrusion, vertical walls between regions).\n")
    a("**Jigsaw key.** A post (%.2f mm high) with a round head and a straight neck stands on the underside of the root strip. The rail has a matching pocket in each saddle face: the pocket outline is the key outline moved outward by "
      "**%.2f mm on every edge** (ASSUMED clearance) and is %.2f mm deeper than the post. The pocket walls fix the half in x (fore-aft) and y; the strip's bottom face sits on the saddle face, which fixes z and the dihedral. "
      "Keys alternate neck direction (+x / -x) so a half cannot be turned end for end.\n" % (P["kh"], P["key_clr"], P["key_clr"]))
    a("**Rail.** A thin web (%.1f mm, %.1f mm in the ballast block) carrying four face plates (wing and tail, right and left, %.2f mm thick, lightened between the keys). The wing plates are tilted +/- %.2f deg about x (dihedral); "
      "the tail plates are rotated by %.2f deg about the fold line x = %s so the tail bend is a printed face, not a hand bend. The plates overlap the web top by %.1f mm (overlapping closed shells in one STL, volume computed as the union).\n"
      % (ri["web_thickness"], ri["block_thickness"], ri["hh"], P["dihedral"], P["bend"] if P["hinge_active"] else 0.0, ("%.1f mm" % P["x_hinge"]) if P["hinge_active"] else "(no tail)", ri["ov"]))
    if cfg["rail"]["print_orientation"] == "side":
        why = ("The rail lies on its side (profile plane on the bed, the flight y axis becomes build height). The two dihedral faces and the tail face are then near-vertical walls made by perimeters "
               "(the dihedral only leans a wall by %.2f deg, a lateral shift of %.4f mm per 0.1 mm layer), so their position and angle depend on XY accuracy (about 0.05 mm) instead of on layer "
               "stair-stepping (an upright print would stair-step a %.1f deg face with 0.1 mm steps and must bridge the underside of the plates). "
               "Weak point: the bed contact is the edge of one plate (%.1f mm wide) - use a brim and a clean plate; if it lifts, set `rail.print_orientation` to `upright` and accept stair-stepped faces.\n"
               % (P["dihedral"], 0.1 * math.tan(math.radians(P["dihedral"])), P["dihedral"], ri["hh"]))
    else:
        why = "The rail is printed as flown.\n"
    a("**Rail print orientation: `%s`.** %s" % (cfg["rail"]["print_orientation"], why))
    a("## Mass, CG and ballast (CALCULATED from the STL volumes)\n")
    a("| item | mass g | x mm aft of root LE |\n|---|---|---|")
    for n, m, x in res["parts_rows"] + res["glue_rows"]:
        a("| %s | %.3f | %.2f |" % (n, m, x))
    a("| **bare total / CG** | **%.3f** | **%.2f** |" % (bud["M0"], bud["cg0"]))
    if res["target"] is not None:
        a("| ballast (lead) at x = %.1f mm | %.3f | %.1f |" % (res["station"], bud["ballast_g"], res["station"]))
        a("| **all-up / target CG** | **%.3f** | **%.2f** (%.1f mm from the nose tip) |" % (bud["allup_g"], res["target"], res["target"] - res["x_nose"]))
    a("\nWing loading %.2f N/m2 on a wing area of %.0f mm2 (planform ahead of the hinge, both halves); ballast pocket %.0f mm3 for %.0f mm3 of lead (fill margin %.0f %%); window %s.\n"
      % (res["wing_loading_Nm2"], res["A_wing"], bud["pocket_volume_mm3"], bud["lead_volume_mm3"], 100 * float(cfg["ballast"]["fill_margin"]),
         ("%.1f x %.1f mm through the %.1f mm ballast block, centre x = %.1f mm" % (ri["window"]["len"], ri["window"]["height"], ri["block_thickness"], res["station"])) if ri["window"] else "none needed"))
    a("## Bed fit\n")
    a("| part | footprint after rotation | rotation | fits %g mm |\n|---|---|---|---|" % P["bed"])
    for pt in res["printed"]:
        a("| %s | %.1f x %.1f | %.1f deg | %s |" % (pt["name"], pt["fit"]["width"], pt["fit"]["height"], pt["fit"]["angle_deg"], "yes" if pt["fit"]["fits"] else "**NO**"))
    if len(res["wing"]["pieces"]) > 1:
        a("\nThe half does not fit the bed whole (rotation searched in %.1f deg steps), so it is split at y = %s mm with jigsaw joints on %.1f mm wide, %.2f mm thick joint strips. "
          "Glue each pair on a flat surface (lap the joint strips' side walls; the seam has no gap, the key has the clearance above)." % (P["rot_step"], ", ".join("%.1f" % c for c in S["geometry"]["cut_y_mm"]), P["wj"], P["Hj"]))
    a("\n## Flags\n")
    fl = S["flags"] or ["none"]
    for f in fl:
        a("- %s" % f)
    if S["notes"]:
        a("\nNotes:")
        for n in S["notes"]:
            a("- %s" % n)
    a("\n## Assembly\n")
    a("1. Print the wings flat-face-down (bead up) in PLA, the rail as stated above, fins and tabs flat. Do not scale in the slicer. Brim for the rail.")
    a("2. Dry fit: each half must drop onto its rail plate with every post in its pocket, the strip flat on the plate. Do not file the pockets; if too tight, re-print with a larger `keys.clearance_mm`.")
    a("3. Glue each half to the rail with thin CA (or hot glue) on the strip/plate contact faces and rib walls; press flat on a table for the wings (the table, not your hand, sets the sheet).")
    if P["hinge_active"]:
        a("4. Tail: press the tail fan down onto the tail plates (the thin hinge zone bends to the printed face at %.1f deg) and glue it; do not hand-bend to a gauge." % P["bend"])
    a("5. Ballast: fit lead into the rail window (%s), seal both sides with tape or CA, weigh, and compare the CG with the target mark on the DXF / foam template." % ("%.1f g" % bud["ballast_g"] if ri["window"] else "none needed"))
    if cfg["fin"]["type"] != "none":
        a("6. Fin: %s" % ("centre fin: slide the tongue into the rail's aft slot and glue." if cfg["fin"]["type"] == "centre" else "tip fins: the wing tip tabs go through the fin slots; glue."))
        if res["fin_geo"] and res["fin_geo"].get("tab_feasible"):
            a("   Trim tab family (printed angles 0, 2, 4 deg; a negative angle is the same tab turned over): glue the chosen tab to the fin's trailing-edge jigsaw. One tab step of 2 deg moves the tab tip by %.2f mm over %.1f mm of tilted tab." % (res["fin_geo"]["tab_angle_step_mm"], res["fin_geo"]["tilt_len"]))
    a("\n## ASSUMED / not validated\n")
    for s in ["PLA 1.24 g/cm3, lead 11.3 g/cm3, glue masses and the pocket fill margin are ASSUMED; weigh the first printed half (CURRENT.md next action 2).",
              "Key/socket clearance 0.15 mm is ASSUMED, never printed. Seams are zero-gap butt joints.",
              "The bead is a rectangular step section, not a half round (the options study used half-round beads of the same width and height; areas differ).",
              "Tail halves are bent about slanted fold lines when dihedral and tail bend are both set; the root gap is enlarged automatically to stay clear (reported above).",
              "CG uses planform x of the sheet masses (bent-tail x shortening of 0.5 % ignored).",
              "Fin position (centre fin on the rail's aft end; tip fins on the tip tabs) is a choice made here, not the position in fin_kit_v4.",
              "No aerodynamic model is run on this geometry (AVL deck export not done)."]:
        a("- " + s)
    return "\n".join(L) + "\n"


def index_html(res, S):
    cfg, P, bud, ri = res["cfg"], res["P"], res["budget"], res["rail"]
    h = ["<!doctype html><html lang='en'><head><meta charset='utf-8'><title>%s sheet builder</title><style>%s</style></head><body>" % (html.escape(cfg["name"]), CSS)]
    h.append("<h1>%s</h1><p class='muted'>sheet builder kit, generated %s. All views are geometry from the same data as the STL files. Dark theme on screen; the foam templates are white print pages.</p>" % (html.escape(cfg["name"]), S["generated"]))
    h.append("<div class='panel'><b>All-up %.2f g</b> (bare %.2f g + ballast %.2f g), CG %.2f mm aft of root LE%s, wing loading %.2f N/m2. Flags: %s</div>"
             % (bud["allup_g"], bud["M0"], bud["ballast_g"], bud["cg0"] if res["target"] is None else res["target"], "" if res["target"] is None else " (bare %.2f)" % bud["cg0"], res["wing_loading_Nm2"],
                ("<span class='flag'>" + "; ".join(html.escape(f) for f in S["flags"]) + "</span>") if S["flags"] else "<span class='ok'>none</span>"))
    h.append("<h2>Parts</h2><table><tr><th>part</th><th>mass g</th><th>footprint mm</th><th>rotate</th><th>fits</th><th>manifold</th><th>file</th></tr>")
    for pt in res["printed"]:
        h.append("<tr><td>%s</td><td>%.3f</td><td>%.1f x %.1f</td><td>%.1f&deg;</td><td class='%s'>%s</td><td>%s</td><td><a href='%s'>stl</a></td></tr>"
                 % (pt["name"], pt["mass_g"], pt["fit"]["width"], pt["fit"]["height"], pt["fit"]["angle_deg"], "ok" if pt["fit"]["fits"] else "flag", "yes" if pt["fit"]["fits"] else "NO", "ok" if pt["check"]["ok"] else "FAIL", pt["file"]))
    h.append("</table><h2>Wing pieces (right wing shown; the left is its mirror)</h2>")
    for pc in res["wing"]["pieces"]:
        plan, sec = page_wing_preview(res, pc)
        h.append("<h3>piece %d: y %.1f to %.1f mm, %d keys on the root strip</h3>%s<br>%s" % (pc["index"] + 1, pc["y_lo"], pc["y_hi"], len(pc["keys"]), plan, sec))
    sv, secs = page_rail_preview(res)
    h.append("<h2>Rail</h2>%s<br>%s" % (sv, "<br>".join(secs)))
    fp = page_fin_preview(res)
    if fp:
        h.append("<h2>Fin and trim-tab family</h2>" + "<br>".join(s for _, s in fp))
    h.append("<h2>Mass table</h2><table><tr><th>item</th><th>g</th><th>x mm</th></tr>")
    for n, m, x in res["parts_rows"] + res["glue_rows"]:
        h.append("<tr><td>%s</td><td>%.3f</td><td>%.2f</td></tr>" % (html.escape(n), m, x))
    h.append("</table><p><a href='README.md'>README.md</a> - <a href='summary.json'>summary.json</a> - foam templates: %s</p></body></html>" %
             ", ".join("<a href='foam/template_p%d.svg'>p%d</a>" % (i + 1, i + 1) for i in range(res["foam_layout"]["pages"])))
    return "".join(h)


# =============================================================================================== OrcaSlicer
def find_orca():
    for base in (r"C:\Program Files\OrcaSlicer", r"C:\Program Files (x86)\OrcaSlicer"):
        p = Path(base) / "orca-slicer.exe"
        if p.exists():
            return p
    for base in (Path.home() / "AppData/Local", Path.home() / "AppData/Roaming"):
        for p in base.glob("**/orca-slicer.exe"):
            return p
    return None


def orca_slice_all(out: Path, timeout=300):
    """Slice every STL in out/parts with the OrcaSlicer CLI (no GUI): Bambu A1 0.4 nozzle, 0.12 mm Fine, generic PLA.  Returns list of dict(part, ok, layers, filament_cm3, seconds, cmd)."""
    import subprocess, time
    exe = find_orca()
    if exe is None:
        return None
    B = exe.parent / "resources" / "profiles" / "BBL"
    od = out / "orca_check"
    od.mkdir(exist_ok=True)

    def flat(kind, name):
        """The CLI does not resolve 'inherits' (it would fall back to a 200 mm bed), so merge the parent chain into one file."""
        chain, nm = [], name
        while nm:
            d = json.loads((B / kind / (nm + ".json")).read_text(encoding="utf-8"))
            chain.append(d)
            nm = d.get("inherits")
        m = {}
        for d in reversed(chain):
            m.update(d)
        m.pop("inherits", None)
        m["name"] = name
        p = od / ("flat_%s.json" % kind)
        p.write_text(json.dumps(m, indent=1), encoding="utf-8")
        return p
    mach, proc, fil = flat("machine", "Bambu Lab A1 0.4 nozzle"), flat("process", "0.12mm Fine @BBL A1"), flat("filament", "Generic PLA @BBL A1")
    rows = []
    for stl in sorted((out / "parts").glob("*.stl")):
        d = od / stl.stem
        d.mkdir(exist_ok=True)
        cmd = [str(exe), "--slice", "0", "--load-settings", "%s;%s" % (mach, proc), "--load-filaments", str(fil), "--outputdir", str(d), str(stl)]
        t0 = time.time()
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
            rc = r.returncode
        except subprocess.TimeoutExpired:
            rc = -1
        g = d / "plate_1.gcode"
        layers = cm3 = None
        if g.exists():
            txt = g.read_text(errors="replace")
            m = re.search(r"; total layer number: (\d+)", txt)
            layers = int(m.group(1)) if m else None
            m = re.search(r"; filament used \[cm3\] = ([0-9.]+)", txt)
            cm3 = float(m.group(1)) if m else None
        rows.append(dict(part=stl.name, ok=(rc == 0 and g.exists() and layers is not None), returncode=rc, layers=layers, filament_cm3=cm3, seconds=round(time.time() - t0, 1), cmd=" ".join('"%s"' % c if " " in c else c for c in cmd)))
    (od / "orca_check.json").write_text(json.dumps(rows, indent=1), encoding="utf-8")
    return rows


# =============================================================================================== CLI
def main(argv=None):
    ap = argparse.ArgumentParser(description="flat-sheet glider kit builder")
    ap.add_argument("config")
    ap.add_argument("--out", help="output folder (default results/runs/sheet_builder/<name>)")
    ap.add_argument("--force", action="store_true", help="allow writing into an existing output folder (files are overwritten)")
    ap.add_argument("--orca", action="store_true", help="afterwards slice every STL with the OrcaSlicer CLI (if installed)")
    a = ap.parse_args(argv)
    cpath = Path(a.config)
    cfg = load_config(cpath)
    out = Path(a.out) if a.out else ROOT / "results/runs/sheet_builder" / cfg["name"]
    if out.exists() and any(out.iterdir()) and not a.force:
        print("refusing to overwrite existing output folder %s (use --force or another --out)" % out)
        return 2
    res = build_kit(cfg, cpath.parent)
    out.mkdir(parents=True, exist_ok=True)
    files = write_kit(res, out)
    b = res["budget"]
    print("%s: %d parts, all-up %.2f g, ballast %.2f g, CG %.2f; flags: %s" % (cfg["name"], len(res["printed"]), b["allup_g"], b["ballast_g"], res["target"] if res["target"] is not None else b["cg0"], [n for n in res["notes"] if n.startswith("FLAG")] or "none"))
    print("wrote %d files to %s" % (len(files), out))
    if a.orca:
        rows = orca_slice_all(out)
        print("orca:", "not installed" if rows is None else ["%s %s (%s layers, %s cm3)" % (r["part"], "ok" if r["ok"] else "FAILED", r["layers"], r["filament_cm3"]) for r in rows])
    return 0


if __name__ == "__main__":
    sys.exit(main())
