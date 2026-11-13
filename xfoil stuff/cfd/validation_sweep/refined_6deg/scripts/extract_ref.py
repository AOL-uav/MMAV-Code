# Surface Cp and wall shear on the plate of the refined 6 deg run at 3 span stations, from a COPY (/home/cfd/run/diag_ref) of
# the t = 20 fields (the time averages pMean and wallShearStressMean over t = 8-20, and the instantaneous p and wallShearStress).
# Nothing is read from or written into the case folder by ParaView.
#   "C:\Program Files\ParaView 5.12.0\bin\pvbatch.exe" --force-offscreen-rendering extract_ref.py
import csv, math, os
import numpy as np
from paraview.simple import *
from paraview import servermanager as sm
from vtk.numpy_interface import dataset_adapter as dsa

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = "\\\\wsl.localhost\\MMAV-CFD\\home\\cfd\\run\\diag_ref\\case.foam"
alpha = 6.0
a = math.radians(alpha)
r = OpenFOAMReader(registrationName="ref", FileName=ROOT)
r.CaseType = "Decomposed Case"
r.MeshRegions = ["internalMesh", "patch/plate"]
r.CellArrays = ["p", "pMean", "wallShearStress", "wallShearStressMean"]
r.UpdatePipelineInformation()
print("times", list(r.TimestepValues))
plate = MergeBlocks(Input=ExtractBlock(Input=r, Selectors=["/Root/boundary/plate"]))
rows = []
for basis, pname, wname in (("mean_8_20", "pMean", "wallShearStressMean"), ("snap_20", "p", "wallShearStress")):
    for sf in (0.25, 0.5, 0.75):
        sl = Slice(Input=plate); sl.SliceType = "Plane"
        sl.SliceType.Origin = [0.5, sf * 1.5, 0.0]; sl.SliceType.Normal = [0, 1, 0]
        cc = CellCenters(Input=sl); cc.UpdatePipeline(20.0)
        d = dsa.WrapDataObject(sm.Fetch(cc))
        pts = np.array(d.Points); p = np.array(d.PointData[pname]); tau = np.array(d.PointData[wname])
        x = pts[:, 0]; z = pts[:, 2]
        xb = 0.25 + (x - 0.25) * math.cos(a) - z * math.sin(a)
        zb = z * math.cos(a) + (x - 0.25) * math.sin(a)
        tc = tau[:, 0] * math.cos(a) - tau[:, 2] * math.sin(a)
        for i in range(len(xb)):
            rows.append([alpha, basis, sf, "U" if zb[i] > 0 else "L", round(float(xb[i]), 5), round(2 * float(p[i]), 5), round(float(tc[i]), 6)])
        print(basis, sf, len(xb))
        Delete(cc); Delete(sl)
with open(os.path.join(HERE, "..", "surface_profiles_ref.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["alpha_deg", "basis", "span_frac", "surface", "xi", "Cp", "tau_c"]); w.writerows(rows)
print("DONE", len(rows))
