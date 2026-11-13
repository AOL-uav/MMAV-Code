# Off-screen ParaView screenshots for the lesson page.
# Run on Windows (no window opens):
#   "C:\Program Files\ParaView 5.12.0\bin\pvbatch.exe" --force-offscreen-rendering make_images.py
# Reads the practice cases through the .foam files in the WSL distro; writes PNGs to img/ next to this script.
import os, sys
from paraview.simple import *

HERE = os.path.dirname(os.path.abspath(__file__))
IMG = os.path.join(HERE, "img")
os.makedirs(IMG, exist_ok=True)
ROOT = r"\\wsl.localhost\MMAV-CFD\home\cfd\run\learn"
BG = [0.078, 0.094, 0.114]  # #14181d


def setup_view(size=(1000, 560)):
    v = CreateView("RenderView")
    v.ViewSize = list(size)
    try:
        v.UseColorPaletteForBackground = 0
    except Exception:
        pass
    v.Background = BG
    v.OrientationAxesVisibility = 0
    return v


def hide_bars(view):
    for n in ("U", "p"):
        GetScalarBar(GetColorTransferFunction(n), view).Visibility = 0


def style_bar(disp, view, title):
    hide_bars(view)
    disp.SetScalarBarVisibility(view, True)
    bar = GetScalarBar(disp.LookupTable, view)
    bar.Title = title
    bar.ComponentTitle = ""
    bar.TitleColor = [0.87, 0.89, 0.92]
    bar.LabelColor = [0.87, 0.89, 0.92]


def shot(view, name, parallel_scale=None, focal=None):
    view.CameraPosition = [focal[0], focal[1], 10] if focal else view.CameraPosition
    if focal:
        view.CameraFocalPoint = [focal[0], focal[1], 0]
        view.CameraViewUp = [0, 1, 0]
    if parallel_scale:
        view.CameraParallelProjection = 1
        view.CameraParallelScale = parallel_scale
    Render(view)
    SaveScreenshot(os.path.join(IMG, name), view, ImageResolution=list(view.ViewSize))
    print("saved", name)


# ---------------- cavity: velocity magnitude
r = OpenFOAMReader(registrationName="cavity.foam", FileName=ROOT + r"\01_cavity\case.foam")
r.MeshRegions = ["internalMesh"]
r.CellArrays = ["U", "p"]
r.UpdatePipelineInformation()
times = list(r.TimestepValues)
print("cavity times:", times)
v = setup_view((700, 620))
d = Show(r, v)
last = times[-1]
v.ViewTime = last
ColorBy(d, ("CELLS", "U", "Magnitude"))
d.RescaleTransferFunctionToDataRange(True, False)
style_bar(d, v, "|U| (m/s)")
shot(v, "cavity_speed.png", parallel_scale=0.06, focal=[0.05, 0.05])
Delete(v); del v

# ---------------- plate: speed, pressure, streamlines, slice
p = OpenFOAMReader(registrationName="plate.foam", FileName=ROOT + r"\02_plate2d\case.foam")
p.MeshRegions = ["internalMesh"]
p.CellArrays = ["U", "p"]
p.UpdatePipelineInformation()
pt = list(p.TimestepValues)
print("plate times:", pt)
tlast = pt[-1]
# cell data -> point data so colours are smooth
c2p = CellDatatoPointData(Input=p)

v = setup_view()
v.ViewTime = tlast
d = Show(c2p, v)
ColorBy(d, ("POINTS", "U", "Magnitude"))
d.RescaleTransferFunctionToDataRange(True, False)
style_bar(d, v, "|U| / U_inf")
shot(v, "plate_speed.png", parallel_scale=1.0, focal=[0.8, 0.0])
Delete(d)

# pressure, zoomed near the plate, fixed colour range
d = Show(c2p, v)
ColorBy(d, ("POINTS", "p"))
lut = GetColorTransferFunction("p")
lut.RescaleTransferFunction(-0.4, 0.4)
style_bar(d, v, "p / rho")
shot(v, "plate_pressure.png", parallel_scale=0.45, focal=[0.5, 0.0])
Delete(d)

# streamlines: line of seed points upstream of the plate, drawn over a faint speed field
st = StreamTracer(Input=c2p, SeedType="Line")
st.Vectors = ["POINTS", "U"]
st.SeedType.Point1 = [-1.5, -0.6, 0.0]
st.SeedType.Point2 = [-1.5, 0.8, 0.0]
st.SeedType.Resolution = 40
st.MaximumStreamlineLength = 6.0
d0 = Show(c2p, v)
ColorBy(d0, ("POINTS", "U", "Magnitude"))
hide_bars(v)
lut = GetColorTransferFunction("U")
lut.RescaleTransferFunction(0.0, 1.4)
d0.Opacity = 0.5
ds = Show(st, v)
ColorBy(ds, None)
ds.AmbientColor = [1.0, 1.0, 1.0]
ds.DiffuseColor = [1.0, 1.0, 1.0]
ds.LineWidth = 1.5
shot(v, "plate_streamlines.png", parallel_scale=0.9, focal=[0.6, 0.0])
Delete(ds); Delete(d0); Delete(st)

# the plate surface itself (patch) to show where the plate is: slice at z=0 used on the page
sl = Slice(Input=p)
sl.SliceType = "Plane"
sl.SliceType.Origin = [0, 0, 0]
sl.SliceType.Normal = [0, 0, 1]
print("slice points:", sl.GetDataInformation().GetNumberOfPoints())
print("DONE")
