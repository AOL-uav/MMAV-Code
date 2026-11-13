# Height of the separated region above the upper surface in the COARSE laminar 8 deg solution (vp_sweep_a08, read through a copy in /home/cfd/run/diag_a08c).
# Only instantaneous snapshots (t = 15 and 20) exist for that case (no mean field).  Run with ParaView's pvbatch on Windows:
#   "C:\Program Files\ParaView 5.12.0\bin\pvbatch.exe" --force-offscreen-rendering box_height_check.py
# For span stations 25, 50, 75 % and chord bins of 0.05 from 0 to 0.85 (body frame), the cell-centre values of the chordwise velocity u_b = U . (cos a, 0, -sin a)
# are binned by height h above the upper surface (0.005 steps); we report the height of the top of the reversed zone (last bin with u_b < 0 before the first
# bin with u_b >= 0 above it) and the height where u_b first reaches 0.9 (edge of the shear layer, free-stream speed 1).
import math, os, csv
import numpy as np
from paraview.simple import *
from paraview import servermanager as sm
from vtk.numpy_interface import dataset_adapter as dsa
bs = chr(92)
ROOT = bs + bs + "wsl.localhost" + bs + "MMAV-CFD" + bs + "home" + bs + "cfd" + bs + "run" + bs + "diag_a08c" + bs + "case.foam"
HERE = os.path.dirname(os.path.abspath(__file__))
alpha = 8.0; a = math.radians(alpha); HT = 0.15 / 3.5 / 2
r = OpenFOAMReader(FileName=ROOT)
r.CaseType = "Decomposed Case"; r.MeshRegions = ["internalMesh"]; r.CellArrays = ["U"]; r.UpdatePipelineInformation()
vol = MergeBlocks(Input=ExtractBlock(Input=r, Selectors=["/Root/internalMesh"]))
rows = []
for t in (15.0, 20.0):
    for sf in (0.25, 0.5, 0.75):
        sl = Slice(Input=vol); sl.SliceType = "Plane"; sl.SliceType.Origin = [0.5, sf * 1.5, 0.0]; sl.SliceType.Normal = [0, 1, 0]
        cc = CellCenters(Input=sl); cc.UpdatePipeline(t)
        d = dsa.WrapDataObject(sm.Fetch(cc)); P = np.array(d.Points); U = np.array(d.PointData["U"])
        x = P[:, 0]; z = P[:, 2]
        xb = 0.25 + (x - 0.25) * math.cos(a) - z * math.sin(a); zb = z * math.cos(a) + (x - 0.25) * math.sin(a)
        ub = U[:, 0] * math.cos(a) - U[:, 2] * math.sin(a); h = zb - HT
        for x0 in [i * 0.05 for i in range(17)]:
            m = (xb >= x0) & (xb < x0 + 0.05) & (h > 0.0) & (h < 0.25)
            hs, us = h[m], ub[m]
            if len(hs) < 5: continue
            bins = np.arange(0.0, 0.2501, 0.005); mean = []
            for b0 in bins[:-1]:
                k = (hs >= b0) & (hs < b0 + 0.005)
                mean.append(us[k].mean() if k.any() else np.nan)
            mean = np.array(mean); idx = np.where(mean < 0)[0]
            if len(idx):
                top = bins[idx[-1] + 1]
                # last negative bin before the first non-negative bin above the first negative one
                j = idx[0]
                while j < len(mean) and not (mean[j] >= 0): j += 1
                top = bins[j] if j < len(mean) else float("nan")
            else:
                top = 0.0
            e = np.where(mean >= 0.9)[0]
            edge = bins[e[0]] if len(e) else float("nan")
            rows.append([t, sf, round(x0, 2), round(float(top), 3), round(float(edge), 3)])
        Delete(cc); Delete(sl)
with open(os.path.join(HERE, "..", "setup", "box_height_check.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["time", "span_frac", "chord_bin_start", "reversed_zone_top_h", "u_b_reaches_0.9_at_h"]); w.writerows(rows)
for t in (15.0, 20.0):
    for sf in (0.25, 0.5, 0.75):
        sel = [x for x in rows if x[0] == t and x[1] == sf and x[2] <= 0.80]
        tops = [x[3] for x in sel if not math.isnan(x[3])]; edges = [x[4] for x in sel if not math.isnan(x[4])]
        print("t=%g span %.2f: max reversed-zone top h = %.3f c (at x/c %.2f), max shear-layer-edge h = %.3f c" % (t, sf, max(tops), sel[[x[3] for x in sel].index(max(tops))][2], max(edges)))
print("DONE")
