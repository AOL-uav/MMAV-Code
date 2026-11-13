# Off-screen ParaView screenshots of a FINISHED real sweep case (alpha 8), read-only.
#
# How the read-only copy folder was made (inside the distro; symlinks do NOT work through \wsl.localhost):
#   V=/home/cfd/run/vp_sweep_a08; W=/home/cfd/run/learn/_view_a08
#   mkdir -p $W/constant $W/system; touch $W/case.foam
#   cp $V/system/{controlDict,fvSchemes,fvSolution} $W/system/
#   for i in $(seq 0 11); do mkdir -p $W/processor$i/constant $W/processor$i/20
#     nice -n 19 cp -r $V/processor$i/constant/polyMesh $W/processor$i/constant/
#     nice -n 19 cp $V/processor$i/20/U $V/processor$i/20/p $W/processor$i/20/; done      # about 166 MB
# Delete $W afterwards (rm -rf). The case itself is only ever read.
# Nothing is written into the case: the reader opens a separate COPY folder (learn/_view_a08)
# holding that one time's fields and mesh plus an empty case.foam.
#   "C:\Program Files\ParaView 5.12.0\bin\pvbatch.exe" --force-offscreen-rendering make_images_sweep.py
import os
from paraview.simple import *

HERE = os.path.dirname(os.path.abspath(__file__))
IMG = os.path.join(HERE, "img")
os.makedirs(IMG, exist_ok=True)
FOAM = r"\\wsl.localhost\MMAV-CFD\home\cfd\run\learn\_view_a08\case.foam"
BG = [0.078, 0.094, 0.114]

r = OpenFOAMReader(registrationName="a08.foam", FileName=FOAM)
r.CaseType = "Decomposed Case"
r.MeshRegions = ["internalMesh", "patch/plate"]
r.CellArrays = ["U", "p"]
r.UpdatePipelineInformation()
times = list(r.TimestepValues)
print("times:", times)
t = times[-1]
r.UpdatePipeline(t)
print("last time", t)


def view(size):
    v = CreateView("RenderView")
    v.ViewSize = list(size)
    try:
        v.UseColorPaletteForBackground = 0
    except Exception:
        pass
    v.Background = BG
    v.OrientationAxesVisibility = 0
    v.ViewTime = t
    return v


def bar(d, v, title):
    d.SetScalarBarVisibility(v, True)
    b = GetScalarBar(d.LookupTable, v)
    b.Title = title
    b.ComponentTitle = ""
    b.TitleColor = [0.87, 0.89, 0.92]
    b.LabelColor = [0.87, 0.89, 0.92]


# --- velocity slice at mid semi-span (y = 0.75 c), looking along the span
data = ExtractBlock(Input=r, Selectors=["/Root/internalMesh"])
sl = Slice(Input=data)
sl.SliceType = "Plane"
sl.SliceType.Origin = [0.5, 0.75, 0.0]
sl.SliceType.Normal = [0, 1, 0]
v = view((1000, 520))
d = Show(sl, v)
ColorBy(d, ("POINTS", "U", "Magnitude"))
GetColorTransferFunction("U").RescaleTransferFunction(0.0, 1.5)
bar(d, v, "|U| / U_inf")
v.CameraPosition = [0.5, -6.0, 0.0]
v.CameraFocalPoint = [0.5, 0.75, 0.0]
v.CameraViewUp = [0, 0, 1]
v.CameraParallelProjection = 1
v.CameraParallelScale = 0.7
SaveScreenshot(os.path.join(IMG, "a08_slice_speed.png"), v, ImageResolution=[1000, 520])
print("saved a08_slice_speed.png")
Delete(d); Delete(v)

# --- pressure on the plate surface (upper side), looking down from +z
pl = ExtractBlock(Input=r, Selectors=["/Root/boundary/plate"])
v = view((1000, 620))
d = Show(pl, v)
ColorBy(d, ("POINTS", "p"))
GetColorTransferFunction("p").RescaleTransferFunction(-0.6, 0.3)
bar(d, v, "p / rho (plate surface)")
v.CameraPosition = [0.5, 0.75, 6.0]
v.CameraFocalPoint = [0.5, 0.75, 0.0]
v.CameraViewUp = [0, 1, 0]
v.CameraParallelProjection = 1
v.CameraParallelScale = 0.95
SaveScreenshot(os.path.join(IMG, "a08_plate_pressure_top.png"), v, ImageResolution=[1000, 620])
print("saved a08_plate_pressure_top.png")
print("DONE")
