#!/usr/bin/env python3
"""Post-processing of the refined 6 deg run against the fixed criteria (read-only). stdlib only.
Run: python analyze_ref.py -> prints tables, writes ../time_binned_ref.csv"""
import csv, math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", ".."))
from collect_sweep import read_coeffs, default_runs
rows = read_coeffs(os.path.join(default_runs(), "vp_ref_a06"))
ALPHA = 6.0; ar = math.radians(ALPHA)

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

out = []
print("time bins (1 chord passage): CL CD CM xcp")
for b in range(0, 20):
    s = stats(b, b+1) if b >= 1 else stats(0.3, 1)
    out.append([b, b+1, round(s["CL"], 5), round(s["CD"], 5), round(s["CM"], 5), round(s["xcp"], 4), round(s["sdCL"], 5), round(s["sdCM"], 5)])
    print("%2d-%2d  %.4f %.4f %+.4f  %.3f   (sd CL %.4f CM %.4f)" % (b, b+1, s["CL"], s["CD"], s["CM"], s["xcp"], s["sdCL"], s["sdCM"]))
with open(os.path.join(HERE, "..", "time_binned_ref.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["t_start", "t_end", "CL", "CD", "CM", "xcp", "sdCL", "sdCM"]); w.writerows(out)
print()
for lab, (a, b) in (("10-20", (10, 20)), ("8-20", (8, 20)), ("12-20", (12, 20))):
    s = stats(a, b); mid = (a+b)/2; h1 = stats(a, mid); h2 = stats(mid, b)
    print("window %s: CL %.4f (sd %.4f) CD %.4f CM %+.4f (sd %.4f) CN %.4f xcp %.4f" % (lab, s["CL"], s["sdCL"], s["CD"], s["CM"], s["sdCM"], s["CN"], s["xcp"]))
    print("   halves [%g,%g] CL %.4f CM %+.4f xcp %.3f | [%g,%g] CL %.4f CM %+.4f xcp %.3f | dCL %.2f%% dCM %+.4f" % (a, mid, h1["CL"], h1["CM"], h1["xcp"], mid, b, h2["CL"], h2["CM"], h2["xcp"], 100*(h2["CL"]-h1["CL"])/((h1["CL"]+h2["CL"])/2), h2["CM"]-h1["CM"]))
# measured at 6.0 deg, both legs
leg = {"increasing": [], "decreasing": []}
for r in csv.DictReader(open(os.path.join(HERE, "..", "..", "..", "..", "research", "plate_data", "ananda2012_flatplate_AR3_taper1.csv"))):
    if r["run"] == "0383ar": leg[r["sweep"]].append((float(r["alpha_deg"]), float(r["CL"]), float(r["CD"]), float(r["CM_c4"])))
def interp(L, a, c):
    L = sorted(L)
    for p, q in zip(L, L[1:]):
        if p[0] <= a <= q[0]: return p[c] + (a-p[0])/(q[0]-p[0])*(q[c]-p[c])
meas = {}
for nm, c in (("CL", 1), ("CD", 2), ("CM", 3)):
    u = interp(leg["increasing"], 6.0, c); d = interp(leg["decreasing"], 6.0, c); meas[nm] = (u, d, (u+d)/2)
    print("measured %s at 6.0: up %.4f down %.4f mean %.4f" % (nm, u, d, (u+d)/2))
cnm = meas["CL"][2]*math.cos(ar) + meas["CD"][2]*math.sin(ar)
print("measured xcp (leg means): %.4f" % (0.25 - meas["CM"][2]/cnm))
for nm, (u, d, m) in meas.items(): pass
