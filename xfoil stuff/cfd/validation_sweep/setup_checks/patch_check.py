# Which faces are in the integrated 'plate' patch? Run: pvbatch.exe --force-offscreen-rendering patch_check.py
# Uses the mesh copy in /home/cfd/run/diag/a04 (alpha 4, from vp_a4_coarse). Cell normals are classed by their y component.
import numpy as np
from paraview.simple import *
from paraview import servermanager as sm
from vtk.numpy_interface import dataset_adapter as dsa
r = OpenFOAMReader(FileName=r"\\wsl.localhost\MMAV-CFD\home\cfd\run\diag\a04\case.foam")
r.CaseType = "Decomposed Case"; r.MeshRegions = ["internalMesh", "patch/plate"]; r.CellArrays = ["U"]; r.UpdatePipelineInformation()
r.UpdatePipeline(20.0)
p = MergeBlocks(Input=ExtractBlock(Input=r, Selectors=["/Root/boundary/plate"]))
p.UpdatePipeline(20.0)
print("p cells after update", p.GetDataInformation().GetNumberOfCells(), "r cells", r.GetDataInformation().GetNumberOfCells(), list(r.TimestepValues))
s = ExtractSurface(Input=p)
n = GenerateSurfaceNormals(Input=s); n.ComputeCellNormals = 1; n.Splitting = 0
c = CellSize(Input=n); c.ComputeArea = 1
c.UpdatePipeline(20.0)
print("cells p", p.GetDataInformation().GetNumberOfCells(), "s", s.GetDataInformation().GetNumberOfCells(), "n", n.GetDataInformation().GetNumberOfCells(), "c", c.GetDataInformation().GetNumberOfCells())
d = dsa.WrapDataObject(sm.Fetch(c))
print(type(d), d.GetNumberOfCells())
area = np.array(d.CellData["Area"]); nrm = np.array(d.CellData["Normals"])
b = d.GetBounds()
print("patch bounds x %.4f..%.4f  y %.4f..%.4f  z %.4f..%.4f" % tuple(b))
print("faces", len(area), " total area %.5f" % area.sum())
for name, m in (("tip-facing (ny > +0.8)", nrm[:, 1] > 0.8), ("root-facing (ny < -0.8)", nrm[:, 1] < -0.8),
                ("upper/lower surfaces (|ny| < 0.5)", abs(nrm[:, 1]) < 0.5)):
    print("%-36s faces %6d  area %.5f" % (name, m.sum(), area[m].sum()))
