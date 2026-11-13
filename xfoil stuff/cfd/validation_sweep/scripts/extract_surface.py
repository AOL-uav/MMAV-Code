# Extract chordwise surface pressure and wall shear on the plate at three span stations, from COPIES of the
# t = 15 and t = 20 snapshots in /home/cfd/run/diag/<aXX> (made with: pimpleFoam -postProcess -parallel -func wallShearStress).
# Run on Windows with ParaView's python (no window):
#   "C:\Program Files\ParaView 5.12.0\bin\pvbatch.exe" --force-offscreen-rendering extract_surface.py
# Writes ../surface_profiles.csv:  alpha_deg, time, span_frac, surface (U/L), xi (chord fraction), Cp, tau_c, tau_n
#   Cp  = 2 p   (kinematic p, freestream p = 0, U = 1)
#   tau_c = wallShearStress . e_chord, e_chord = (cos a, 0, -sin a) from LE to TE (positive = flow toward the trailing edge,
#           checked on the attached lower surface). tau_n = wallShearStress . e_normal.
import csv, math, os
import numpy as np
from paraview.simple import *
from paraview import servermanager as sm
from vtk.numpy_interface import dataset_adapter as dsa

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "surface_profiles.csv")
ROOT = r"\\wsl.localhost\MMAV-CFD\home\cfd\run\diag"
CASES = {2: "a02", 4: "a04", 6: "a06", 8: "a08"}
TIMES = [15.0, 20.0]
SPAN = [0.25, 0.5, 0.75]
HALF = 1.5

rows = []
for alpha, name in CASES.items():
    a = math.radians(alpha)
    r = OpenFOAMReader(registrationName=name, FileName=ROOT + "\\" + name + "\\case.foam")
    r.CaseType = "Decomposed Case"
    r.MeshRegions = ["internalMesh", "patch/plate"]
    r.CellArrays = ["U", "p", "wallShearStress"]
    r.UpdatePipelineInformation()
    print(name, "times", list(r.TimestepValues))
    eb = ExtractBlock(Input=r, Selectors=["/Root/boundary/plate"])
    mb = MergeBlocks(Input=eb)
    for t in TIMES:
        for sf in SPAN:
            y0 = sf * HALF
            sl = Slice(Input=mb)
            sl.SliceType = "Plane"
            sl.SliceType.Origin = [0.5, y0, 0.0]
            sl.SliceType.Normal = [0, 1, 0]
            cc = CellCenters(Input=sl)
            cc.UpdatePipeline(t)
            d = dsa.WrapDataObject(sm.Fetch(cc))
            pts = np.array(d.Points)
            if len(pts) == 0:
                print("no points", name, t, sf); continue
            p = np.array(d.PointData["p"])
            tau = np.array(d.PointData["wallShearStress"])
            x = pts[:, 0]; z = pts[:, 2]
            xb = 0.25 + (x - 0.25) * math.cos(a) - z * math.sin(a)   # body (chord) coordinates
            zb = z * math.cos(a) + (x - 0.25) * math.sin(a)
            tc = tau[:, 0] * math.cos(a) - tau[:, 2] * math.sin(a)
            tn = tau[:, 0] * math.sin(a) + tau[:, 2] * math.cos(a)
            for i in range(len(xb)):
                rows.append([alpha, t, sf, "U" if zb[i] > 0 else "L", round(float(xb[i]), 5), round(2 * float(p[i]), 5),
                             round(float(tc[i]), 6), round(float(tn[i]), 6)])
            Delete(cc); Delete(sl)
            print(name, t, sf, len(xb), "points")
    Delete(mb); Delete(r)

with open(OUT, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["alpha_deg", "time", "span_frac", "surface", "xi", "Cp", "tau_c", "tau_n"])
    w.writerows(rows)
print("wrote", os.path.abspath(OUT), len(rows), "rows")
