#!/usr/bin/env python3
"""usage: stats.py <case_dir> <t_start> [t_end] [label]
Reads postProcessing/forceCoeffs1/*/coefficient.dat (all restart dirs, in order).
Prints mean / std-dev(=RMS of fluctuation) / min / max of Cl, Cd, CmPitch over the window,
plus 4-sub-block means (spread = crude statistical-convergence indicator).
Also writes a decimated CSV and a PNG force-history plot next to the case (analysis/)."""
import sys, glob, os
import numpy as np
case = sys.argv[1]; t0 = float(sys.argv[2])
t1 = float(sys.argv[3]) if len(sys.argv) > 3 else 1e9
label = sys.argv[4] if len(sys.argv) > 4 else os.path.basename(case)
rows = []
for f in sorted(glob.glob(case + "/postProcessing/forceCoeffs1/*/coefficient.dat"), key=lambda s: float(s.split("/")[-2])):
    for ln in open(f):
        if ln.startswith("#") or not ln.strip(): continue
        rows.append([float(x) for x in ln.split()])
d = np.array(rows)
d = d[np.unique(d[:, 0], return_index=True)[1]]
t, Cd, Cl, Cm = d[:, 0], d[:, 1], d[:, 4], d[:, 7]
os.makedirs(case + "/analysis", exist_ok=True)
step = max(1, len(t)//1500)
np.savetxt(case + "/analysis/forces_%s.csv" % label, np.c_[t, Cl, Cd, Cm][t >= 0.5][::step], delimiter=",",
           header="t_chordpassages,CL,CD,CM_c4(nose-up positive)", comments="", fmt="%.6g")
tend = min(t1, t[-1]); m = (t >= t0) & (t <= tend)
print("%s: run reached t=%.3f (%d samples); window [%.2f, %.2f], %d samples" % (label, t[-1], len(t), t0, tend, m.sum()))
for nm, y in (("CL", Cl), ("CD", Cd), ("CM", Cm)):
    # time-weighted statistics (adaptive dt)
    tt = t[m]; yy = y[m]; w = np.gradient(tt)
    mean = np.sum(w*yy)/np.sum(w); sd = np.sqrt(np.sum(w*(yy-mean)**2)/np.sum(w))
    edges = np.linspace(tt[0], tt[-1], 5); bm = []
    for i in range(4):
        k = (tt >= edges[i]) & (tt <= edges[i+1])
        bm.append(np.sum(w[k]*yy[k])/np.sum(w[k]))
    print("  %s mean=% .5f  rms(fluct)=%.5f  min=% .5f max=% .5f  4-block means: %s" %
          (nm, mean, sd, yy.min(), yy.max(), " ".join("% .4f" % b for b in bm)))
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
fig, ax = plt.subplots(3, 1, sharex=True, figsize=(8, 7))
pm = t >= 0.5
for a, y, nm in zip(ax, (Cl, Cd, Cm), ("CL", "CD", "CM c/4 (nose-up +)")):
    a.plot(t[pm], y[pm], lw=0.7); a.set_ylabel(nm); a.grid(alpha=.3); a.axvspan(t0, tend, color="orange", alpha=.15)
ax[-1].set_xlabel("t [chord passages]"); ax[0].set_title(label + "  (shaded = averaging window)")
fig.tight_layout(); fig.savefig(case + "/analysis/forces_%s.png" % label, dpi=90)
