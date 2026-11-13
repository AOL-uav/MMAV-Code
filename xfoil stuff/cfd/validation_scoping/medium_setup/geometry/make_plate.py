#!/usr/bin/env python3
"""Ananda, Sukumar & Selig (2015) rectangular flat-plate wing, AR 3, half model.
Units: chord c = 1.  Thickness t = 0.15 in / 3.5 in chord = 0.042857 c (4.3 %).
Leading edge : half-ellipse, semi-major a = 0.375 in = 0.107143 c  (5-to-1 ellipse)
Trailing edge: half-ellipse, semi-major b = 0.75  in = 0.214286 c  (10-to-1 ellipse)
Plate is rotated nose-up by alpha about the quarter-chord point (x=0.25, z=0).
Span runs along +y; root at y=0 (symmetry plane); tip (flat cut) at y=AR/2=1.5.
The STL is extended to y=-0.1 so it pokes through the symmetry plane cleanly.
Tip shape is NOT stated in the paper text; a square-cut flat tip is assumed.
usage: make_plate.py alpha_deg out.stl
"""
import sys, math
import numpy as np
alpha = math.radians(float(sys.argv[1])); out = sys.argv[2]
t = 0.15/3.5; h = t/2; a = 0.375/3.5; b = 0.75/3.5
Y0, Y1 = -0.1, 1.5
n = 40
th = np.linspace(0, math.pi/2, n)
le = [(a*(1-math.cos(q)), h*math.sin(q)) for q in th]          # x 0 -> a
te = [((1-b)+b*math.sin(q), h*math.cos(q)) for q in th]        # x 1-b -> 1
top = le + te                                                   # LE -> TE (flat part is the straight join)
bot = [(x, -z) for (x, z) in reversed(top)]
loop = top[:-1] + bot[:-1]                                      # drop duplicated TE and LE points
# clean zero-length duplicates (le end and te start are both z=h)
clean = [loop[0]]
for p in loop[1:]:
    if math.hypot(p[0]-clean[-1][0], p[1]-clean[-1][1]) > 1e-9: clean.append(p)
loop = clean
ca, sa = math.cos(alpha), math.sin(alpha)
def rot(p):  # nose-up rotation about (0.25,0): LE rises
    x, z = p[0]-0.25, p[1]
    return (0.25 + x*ca + z*sa, z*ca - x*sa)
loop = [rot(p) for p in loop]
cen = rot((0.5, 0.0))
tris = []
N = len(loop)
for i in range(N):
    p, q = loop[i], loop[(i+1) % N]
    A = (p[0], Y0, p[1]); B = (q[0], Y0, q[1]); Cc = (q[0], Y1, q[1]); D = (p[0], Y1, p[1])
    tris += [(A, B, Cc), (A, Cc, D)]
    tris.append(((cen[0], Y1, cen[1]), D, Cc))      # cap at Y1, normal +y
    tris.append(((cen[0], Y0, cen[1]), B, A))       # cap at Y0, normal -y
# closedness / orientation checks
from collections import Counter
edges = Counter()
for (u, v, w) in tris:
    for e in ((u, v), (v, w), (w, u)): edges[e] += 1
bad = [e for e in edges if edges.get((e[1], e[0]), 0) != edges[e]]
vol = 0.0
for (u, v, w) in tris:
    vol += np.dot(u, np.cross(v, w))/6.0
sys.stderr.write("tris=%d nonmatching_edges=%d signed_volume=%.5f (expected ~ +%.5f)\n"
                 % (len(tris), len(bad), vol, 1.6*(0.9*t*0.95)))
if bad or vol <= 0: sys.exit("STL orientation/closure check failed")
with open(out, "w") as f:
    f.write("solid plate\n")
    for (u, v, w) in tris:
        nrm = np.cross(np.subtract(v, u), np.subtract(w, u)); nn = np.linalg.norm(nrm)
        if nn == 0: continue
        nrm /= nn
        f.write(" facet normal %.6e %.6e %.6e\n  outer loop\n" % tuple(nrm))
        for P in (u, v, w): f.write("   vertex %.8e %.8e %.8e\n" % P)
        f.write("  endloop\n endfacet\n")
    f.write("endsolid plate\n")
