#!/usr/bin/env python3
"""Reversed-flow zones on the upper surface of the refined 6 deg run, same method as ../../scripts/bubble_analysis.py
(s = -tau_c > 0 forward flow; smoothing +-0.03 c; ignore x/c < 0.03; a zone is s < 0 for at least 0.05 c).
Reads ../surface_profiles_ref.csv (time-averaged t=8-20 and the t=20 snapshot) and the coarse laminar ../../surface_profiles.csv."""
import csv, collections, os
HERE = os.path.dirname(os.path.abspath(__file__))
WIN, XMIN, HOLD = 0.03, 0.03, 0.05
def smooth(xs, ys, w):
    return [sum(ys[j] for j in range(len(xs)) if abs(xs[j]-x) <= w)/sum(1 for j in range(len(xs)) if abs(xs[j]-x) <= w) for x in xs]
def analyse(xs, s):
    ss = smooth(xs, s, WIN); idx = [i for i, x in enumerate(xs) if x >= XMIN]; segs, start = [], None
    for i in idx:
        if ss[i] < 0 and start is None: start = xs[i]
        if ss[i] >= 0 and start is not None:
            if xs[i]-start >= HOLD: segs.append((start, xs[i]))
            start = None
    if start is not None and xs[idx[-1]]-start >= HOLD: segs.append((start, None))
    return segs, sum(((e if e is not None else 1.0)-b) for b, e in segs)/(1.0-XMIN)
R = collections.defaultdict(list)
for r in csv.DictReader(open(os.path.join(HERE, "..", "surface_profiles_ref.csv"))):
    R[(r["basis"], float(r["span_frac"]), r["surface"])].append((float(r["xi"]), float(r["Cp"]), -float(r["tau_c"])))
for k in R: R[k].sort()
L = collections.defaultdict(list)
for r in csv.DictReader(open(os.path.join(HERE, "..", "..", "surface_profiles.csv"))):
    if r["alpha_deg"] == "6": L[(float(r["time"]), float(r["span_frac"]), r["surface"])].append((float(r["xi"]), float(r["Cp"]), -float(r["tau_c"])))
for k in L: L[k].sort()
out = []
for sp in (0.25, 0.5, 0.75):
    for basis in ("mean_8_20", "snap_20"):
        xs = [p[0] for p in R[(basis, sp, "U")]]; s = [p[2] for p in R[(basis, sp, "U")]]
        segs, frac = analyse(xs, s)
        out.append(["refined", sp, basis, round(frac, 3), len(segs), ";".join("%.2f-%s" % (b, "TE" if e is None else "%.2f" % e) for b, e in segs)])
    a = L[(15.0, sp, "U")]; b = L[(20.0, sp, "U")]; n = min(len(a), len(b))
    xs = [a[i][0] for i in range(n)]; s = [(a[i][2]+b[i][2])/2 for i in range(n)]
    segs, frac = analyse(xs, s)
    out.append(["coarse laminar", sp, "mean of t=15,20 snapshots", round(frac, 3), len(segs), ";".join("%.2f-%s" % (b_, "TE" if e is None else "%.2f" % e) for b_, e in segs)])
with open(os.path.join(HERE, "..", "bubble_extent_ref.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["case", "span_frac", "basis", "reversed_fraction_upper", "n_zones", "zones_xi"]); w.writerows(out)
for r in out: print(r)
# sanity: lower surface should be forward (s>0) at mid chord
for sp in (0.25, 0.5, 0.75):
    ll = [p[2] for p in R[("mean_8_20", sp, "L")] if 0.3 < p[0] < 0.7]
    print("lower surface mean s, span %.2f, x/c 0.3-0.7: %.5f (positive = attached)" % (sp, sum(ll)/len(ll)))
