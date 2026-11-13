#!/usr/bin/env python3
"""Post-processing of the refined 8 deg run (stopped early by decision) against the fixed criteria (read-only). stdlib only.
Run: python analyze_ref_a08.py -> prints tables, writes ../time_binned_ref.csv
Graded window: t = 4 to the stop time (disclosed departure, see ../README.md "Result"); t = 8 to stop shown for comparison."""
import csv, math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", ".."))
from collect_sweep import read_coeffs, default_runs
rows = read_coeffs(os.path.join(default_runs(), "vp_ref_a08"))
ALPHA = 8.0; ar = math.radians(ALPHA)
T_END = rows[-1][0]

def wmean(t0, t1, k):
    sel = [r for r in rows if t0 <= r[0] <= t1]
    ts = [r[0] for r in sel]; w = []
    for i, t in enumerate(ts):
        lo = ts[i-1] if i > 0 else t; hi = ts[i+1] if i < len(ts)-1 else t
        w.append((hi-lo)/(2.0 if 0 < i < len(ts)-1 else 1.0))
    m = sum(x*r[k] for x, r in zip(w, sel))/sum(w)
    sd = (sum(x*(r[k]-m)**2 for x, r in zip(w, sel))/sum(w))**0.5
    return m, sd

def stats(t0, t1):
    cd, sd_d = wmean(t0, t1, 1); cl, sd_l = wmean(t0, t1, 2); cm, sd_m = wmean(t0, t1, 3)
    cn = cl*math.cos(ar) + cd*math.sin(ar)
    return dict(CL=cl, CD=cd, CM=cm, CN=cn, xcp=0.25-cm/cn, sdCL=sd_l, sdCD=sd_d, sdCM=sd_m)

print("last time in force file: %.4f" % T_END)
out = []; bins = []
print("time bins (1 chord passage): CL CD CM xcp")
for b in range(0, int(T_END)):
    s = stats(b, b+1) if b >= 1 else stats(0.3, 1)
    out.append([b, b+1, round(s["CL"], 5), round(s["CD"], 5), round(s["CM"], 5), round(s["xcp"], 4), round(s["sdCL"], 5), round(s["sdCM"], 5)])
    if b >= 4: bins.append(s)
    print("%2d-%2d  %.4f %.4f %+.5f  %.4f   (sd CL %.4f CM %.4f)" % (b, b+1, s["CL"], s["CD"], s["CM"], s["xcp"], s["sdCL"], s["sdCM"]))
with open(os.path.join(HERE, "..", "time_binned_ref.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["t_start", "t_end", "CL", "CD", "CM", "xcp", "sdCL", "sdCM"]); w.writerows(out)
print()
n = len(bins); xs = [s["xcp"] for s in bins]; mx = sum(xs)/n
sdx = (sum((x-mx)**2 for x in xs)/(n-1))**0.5
print("whole bins from t=4: n %d, bin-to-bin SD of xcp %.4f, standard error %.4f" % (n, sdx, sdx/n**0.5))
for lab, (a, b) in (("4-end (graded)", (4, T_END)), ("8-end (comparison)", (8, T_END))):
    s = stats(a, b); mid = (a+b)/2; h1 = stats(a, mid); h2 = stats(mid, b)
    print("window %s [%g, %.3f]: CL %.4f (sd %.4f) CD %.4f CM %+.5f (sd %.4f) CN %.4f xcp %.4f" % (lab, a, b, s["CL"], s["sdCL"], s["CD"], s["CM"], s["sdCM"], s["CN"], s["xcp"]))
    print("   halves [%g,%.3f] CL %.4f CM %+.5f xcp %.4f | [%.3f,%.3f] CL %.4f CM %+.5f xcp %.4f | dCL %.2f%% dCM %+.5f dxcp %+.4f" % (a, mid, h1["CL"], h1["CM"], h1["xcp"], mid, b, h2["CL"], h2["CM"], h2["xcp"], 100*(h2["CL"]-h1["CL"])/((h1["CL"]+h2["CL"])/2), h2["CM"]-h1["CM"], h2["xcp"]-h1["xcp"]))
    if a == 4:
        print("   margin to 0.230 edge %.4f, to 0.215 edge %.4f" % (0.230 - s["xcp"], s["xcp"] - 0.215))
