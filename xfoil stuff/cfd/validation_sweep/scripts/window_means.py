#!/usr/bin/env python3
"""Quarter-window means of CL, CD, CM (stationarity check) from the existing force-coefficient histories. stdlib only.
Run: python window_means.py [--runs DIR]  -> ../window_means.csv
Window: t = 8..20 for the 20-chord cases (alpha 0-8), t = 20..40 for the 40-chord cases (alpha 10, 12); alpha 4 is
vp_a4_coarse (8..20). Each window is split into four equal time intervals; means are time-weighted (adaptive dt).
drift = Q4 mean - Q1 mean; slope = least-squares line over the window, change per 10 chord passages."""
import argparse, csv, glob, os, re, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from collect_sweep import read_coeffs, end_time, default_runs   # same reader as the sweep collector

ap = argparse.ArgumentParser(); ap.add_argument("--runs", default=default_runs()); a = ap.parse_args()
cases = [(4.0, os.path.join(a.runs, "vp_a4_coarse"))]
for d in glob.glob(os.path.join(a.runs, "vp_sweep_a*")):
    m = re.search(r"vp_sweep_a(\d+)$", d)
    if m: cases.append((float(m.group(1)), d))
cases.sort()


def wmean(rows, t0, t1, k):
    sel = [r for r in rows if t0 <= r[0] <= t1]
    ts = [r[0] for r in sel]
    w = []
    for i, t in enumerate(ts):
        lo = ts[i - 1] if i > 0 else t
        hi = ts[i + 1] if i < len(ts) - 1 else t
        w.append((hi - lo) / (2.0 if 0 < i < len(ts) - 1 else 1.0))
    return sum(x * r[k] for x, r in zip(w, sel)) / sum(w)


def slope(rows, t0, t1, k):
    sel = [(r[0], r[k]) for r in rows if t0 <= r[0] <= t1]
    n = len(sel); mt = sum(t for t, _ in sel) / n; my = sum(y for _, y in sel) / n
    return sum((t - mt) * (y - my) for t, y in sel) / sum((t - mt) ** 2 for t, _ in sel)


out = []
hdr = ["alpha_deg", "window", "coef", "Q1", "Q2", "Q3", "Q4", "window_mean", "drift_Q4_minus_Q1", "drift_pct_of_mean", "slope_per_10_chords", "statement"]
for alpha, case in cases:
    rows = read_coeffs(case)
    endt = end_time(case) or rows[-1][0]
    t0 = 8.0 if endt <= 20.5 else endt / 2.0
    edges = [t0 + (endt - t0) * i / 4 for i in range(5)]
    for name, k in (("CL", 2), ("CD", 1), ("CM", 3)):
        q = [wmean(rows, edges[i], edges[i + 1], k) for i in range(4)]
        mean = wmean(rows, t0, endt, k)
        dr = q[3] - q[0]
        sl = slope(rows, t0, endt, k) * 10
        pct = 100 * dr / mean if abs(mean) > 1e-9 else float("nan")
        spread = max(q) - min(q)
        mono = all(q[i + 1] >= q[i] for i in range(3)) or all(q[i + 1] <= q[i] for i in range(3))
        if name == "CM":
            stmt = "monotone drift" if mono and abs(dr) > 0.002 else ("moves around, no clear trend" if spread > 0.002 else "stationary within 0.002")
        else:
            stmt = "monotone drift" if mono and abs(pct) > 2 else ("moves around, no clear trend" if abs(spread / mean) > 0.02 else "stationary within 2%")
        out.append([alpha, "%g-%g" % (t0, endt), name] + [round(v, 5) for v in q] + [round(mean, 5), round(dr, 5), round(pct, 2), round(sl, 5), stmt])
with open(os.path.join(HERE, "..", "window_means.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(hdr); w.writerows(out)
for r in out:
    if r[0] in (6, 8, 10, 12):
        print("%4.0f %-6s %-3s Q: %s mean %.4f drift %+.4f (%+.1f%%) %s" % (r[0], r[1], r[2], " ".join("%.4f" % v for v in r[3:7]), r[7], r[8], r[9], r[11]))
