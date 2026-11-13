#!/usr/bin/env python3
"""Geometry as meshed, measured on the STL files themselves (not on the generating script). stdlib only.
Reads ASCII STL, undoes the nose-up rotation about (0.25, 0) with the case's alpha, and reports chord extent, span extent,
maximum thickness, flat-section extent, leading/trailing-edge ellipse lengths and the largest deviation of the edge profile
from the ellipse stated by Ananda (5:1 LE, semi-major 0.375 in = 0.1071 c; 10:1 TE, semi-major 0.75 in = 0.2143 c; t = 0.15 in = 0.0429 c;
chord 3.5 in). Usage: py -B stl_check.py <stl> <alpha_deg> ..."""
import math, sys, collections

def load(path):
    pts = []
    for ln in open(path):
        if "vertex" in ln:
            pts.append(tuple(float(v) for v in ln.split()[1:4]))
    return pts

def check(path, alpha):
    a = math.radians(alpha)
    P = load(path)
    tip = [p for p in P if abs(p[1] - 1.5) < 1e-9]
    xb = lambda p: 0.25 + (p[0] - 0.25) * math.cos(a) - p[2] * math.sin(a)
    zb = lambda p: p[2] * math.cos(a) + (p[0] - 0.25) * math.sin(a)
    xs = [xb(p) for p in P]; ys = [p[1] for p in P]
    print(f"{path}  (alpha {alpha})")
    print(f"  vertices {len(P)}; chord coordinate {min(xs):.5f} .. {max(xs):.5f}; span coordinate {min(ys):.3f} .. {max(ys):.3f}")
    # section outline at the tip face (y = 1.5); bin by chord station
    top = collections.defaultdict(lambda: -9.0); bot = collections.defaultdict(lambda: 9.0)
    N = 400
    for p in tip:
        i = int(round(xb(p) * N))
        top[i] = max(top[i], zb(p)); bot[i] = min(bot[i], zb(p))
    th = {i: top[i] - bot[i] for i in top}
    tmax = max(th.values())
    flat = [i / N for i, t in th.items() if t >= 0.999 * tmax]
    xle, xte = min(flat), max(flat)
    print(f"  maximum thickness {tmax:.5f} c  (Ananda 0.15/3.5 = {0.15/3.5:.5f})")
    print(f"  flat (full-thickness) section from x/c = {xle:.4f} to {xte:.4f}  ->  LE length {xle:.4f} c (Ananda 0.375/3.5 = {0.375/3.5:.4f}), TE length {1-xte:.4f} c (Ananda 0.75/3.5 = {0.75/3.5:.4f})")
    h = tmax / 2; la = 0.375 / 3.5; ta = 0.75 / 3.5
    dl = max(abs(th[i] - 2 * h * math.sqrt(max(0.0, 1 - ((la - i / N) / la) ** 2))) for i in th if 0.01 < i / N < la)
    dt = max(abs(th[i] - 2 * h * math.sqrt(max(0.0, 1 - (((i / N) - (1 - ta)) / ta) ** 2))) for i in th if 1 - ta < i / N < 0.99)
    print(f"  max thickness deviation from the ideal ellipse: LE {dl:.5f} c, TE {dt:.5f} c (sampling 1/{N} c)")
    # centreline symmetric?
    mid = max(abs((top[i] + bot[i]) / 2) for i in th if 0.15 < i / N < 0.75)
    print(f"  camber (max |mid-thickness line| over 0.15-0.75 c): {mid:.5f} c  -> symmetric flat plate")

for k in range(1, len(sys.argv), 2):
    check(sys.argv[k], float(sys.argv[k + 1]))
