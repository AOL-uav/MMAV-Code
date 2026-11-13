# Surface Cp / wall shear on the plate (3 span stations) and near-surface gammaInt / nut at mid-span, from COPIES of the
# t = 25 and t = 30 snapshots of the kOmegaSSTLM runs in /home/cfd/run/diag_lm/<aNN>.
#   "C:\Program Files\ParaView 5.12.0\bin\pvbatch.exe" --force-offscreen-rendering extract_lm.py
# Writes ../surface_profiles_lm.csv (alpha,time,span_frac,surface,xi,Cp,tau_c,tau_n) and
#        ../near_surface_lm.csv    (alpha,time,xi,h,gammaInt,nut_over_nu,k) for points within h < 0.06 c above the upper surface.
import csv, math, os
import numpy as np
from paraview.simple import *
from paraview import servermanager as sm
from vtk.numpy_interface import dataset_adapter as dsa

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = r"\\wsl.localhost\MMAV-CFD\home\cfd\run\diag_lm"
CASES = {4: "a04", 8: "a08"}
TIMES = [25.0, 30.0]
NU = 1.0 / 60000
HT = 0.15 / 3.5 / 2   # plate half thickness (c)

surf_rows, near_rows = [], []
for alpha, name in CASES.items():
    a = math.radians(alpha)
    r = OpenFOAMReader(registrationName=name, FileName=ROOT + "\\" + name + "\\case.foam")
    r.CaseType = "Decomposed Case"
    r.MeshRegions = ["internalMesh", "patch/plate"]
    r.CellArrays = ["U", "p", "k", "nut", "gammaInt", "ReThetat", "wallShearStress"]
    r.UpdatePipelineInformation()
    print(name, "times", list(r.TimestepValues))
    plate = MergeBlocks(Input=ExtractBlock(Input=r, Selectors=["/Root/boundary/plate"]))
    vol = MergeBlocks(Input=ExtractBlock(Input=r, Selectors=["/Root/internalMesh"]))

    def to_body(x, z):
        xb = 0.25 + (x - 0.25) * math.cos(a) - z * math.sin(a)
        zb = z * math.cos(a) + (x - 0.25) * math.sin(a)
        return xb, zb

    for t in TIMES:
        for sf in (0.25, 0.5, 0.75):
            sl = Slice(Input=plate); sl.SliceType = "Plane"
            sl.SliceType.Origin = [0.5, sf * 1.5, 0.0]; sl.SliceType.Normal = [0, 1, 0]
            cc = CellCenters(Input=sl); cc.UpdatePipeline(t)
            d = dsa.WrapDataObject(sm.Fetch(cc))
            pts = np.array(d.Points); p = np.array(d.PointData["p"]); tau = np.array(d.PointData["wallShearStress"])
            xb, zb = to_body(pts[:, 0], pts[:, 2])
            tc = tau[:, 0] * math.cos(a) - tau[:, 2] * math.sin(a)
            tn = tau[:, 0] * math.sin(a) + tau[:, 2] * math.cos(a)
            for i in range(len(xb)):
                surf_rows.append([alpha, t, sf, "U" if zb[i] > 0 else "L", round(float(xb[i]), 5), round(2 * float(p[i]), 5),
                                  round(float(tc[i]), 6), round(float(tn[i]), 6)])
            Delete(cc); Delete(sl)
        # near-surface transition indicators at mid-span
        sl = Slice(Input=vol); sl.SliceType = "Plane"
        sl.SliceType.Origin = [0.5, 0.75, 0.0]; sl.SliceType.Normal = [0, 1, 0]
        cc = CellCenters(Input=sl); cc.UpdatePipeline(t)
        d = dsa.WrapDataObject(sm.Fetch(cc))
        pts = np.array(d.Points)
        g = np.array(d.PointData["gammaInt"]); nut = np.array(d.PointData["nut"]); k = np.array(d.PointData["k"])
        xb, zb = to_body(pts[:, 0], pts[:, 2])
        h = zb - HT
        m = (xb > -0.05) & (xb < 1.2) & (h > 0.0) & (h < 0.06)
        for i in np.where(m)[0]:
            near_rows.append([alpha, t, round(float(xb[i]), 4), round(float(h[i]), 4), round(float(g[i]), 5),
                              round(float(nut[i] / NU), 4), float("%.4g" % k[i])])
        print(name, t, "near-surface points", int(m.sum()))
        Delete(cc); Delete(sl)

with open(os.path.join(HERE, "..", "surface_profiles_lm.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["alpha_deg", "time", "span_frac", "surface", "xi", "Cp", "tau_c", "tau_n"]); w.writerows(surf_rows)
with open(os.path.join(HERE, "..", "near_surface_lm.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["alpha_deg", "time", "xi", "h", "gammaInt", "nut_over_nu", "k"]); w.writerows(near_rows)
print("DONE", len(surf_rows), len(near_rows))
