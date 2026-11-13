#!/bin/bash
# usage: mkcase.sh <casename> <alpha_deg> <profile: coarse|medium> [endTime]
# Creates a self-contained OpenFOAM case in /home/cfd/run/<casename>.
# Non-dimensional units: chord c = 1, U = 1, nu = 1/Re  (Re = 60000).
set -e
NAME=$1; ALPHA=$2; PROFILE=$3; ENDT=${4:-20}
RE=60000
NU=$(python3 -c "print('%.8g' % (1.0/$RE))")
C=/home/cfd/run/$NAME
rm -rf $C; mkdir -p $C/{0.orig,constant/triSurface,system,geometry}
cd $C

# ---- refinement profile (cell sizes: base 0.25 chord, level n -> 0.25/2^n) ----
case $PROFILE in
 coarse) SURF=5; SHELL="((0.03 5) (0.08 4) (0.20 3))"; NEAR=3; LETE=5; WAKE1=3; WAKE2=2; TIPL=4;;
 medium) SURF=6; SHELL="((0.03 6) (0.08 5) (0.20 4))"; NEAR=4; LETE=6; WAKE1=4; WAKE2=3; TIPL=5;;
 *) echo bad profile; exit 1;;
esac
echo "$PROFILE SURF=$SURF NEAR=$NEAR LETE=$LETE WAKE1=$WAKE1 WAKE2=$WAKE2 TIPL=$TIPL shell=$SHELL" > geometry/profile.txt

cat > geometry/make_plate.py <<'EOF'
#!/usr/bin/env python3
"""Ananda, Sukumar & Selig (2015) rectangular flat-plate wing, AR 3, half model.
Units: chord c = 1.  Thickness t = 0.15 in / 3.5 in chord = 0.042857 c (4.3 %).
Leading edge : half-ellipse, semi-major a = 0.375 in = 0.107143 c  (5-to-1 ellipse)
Trailing edge: half-ellipse, semi-major b = 0.75  in = 0.214286 c  (10-to-1 ellipse)
Plate is rotated nose-up by alpha about the quarter-chord point (x=0.25, z=0).
Span runs along +y; root at y=0 (symmetry plane); tip (flat cut) at y=AR/2=1.5.
The STL is extended to y=-0.1 so it pokes through the symmetry plane cleanly.
Tip shape is NOT stated in the paper text; a square-cut flat tip is assumed.
usage: make_plate.py alpha_deg out.stl
"""
import sys, math
import numpy as np
alpha = math.radians(float(sys.argv[1])); out = sys.argv[2]
t = 0.15/3.5; h = t/2; a = 0.375/3.5; b = 0.75/3.5
Y0, Y1 = -0.1, 1.5
n = 40
th = np.linspace(0, math.pi/2, n)
le = [(a*(1-math.cos(q)), h*math.sin(q)) for q in th]          # x 0 -> a
te = [((1-b)+b*math.sin(q), h*math.cos(q)) for q in th]        # x 1-b -> 1
top = le + te                                                   # LE -> TE (flat part is the straight join)
bot = [(x, -z) for (x, z) in reversed(top)]
loop = top[:-1] + bot[:-1]                                      # drop duplicated TE and LE points
# clean zero-length duplicates (le end and te start are both z=h)
clean = [loop[0]]
for p in loop[1:]:
    if math.hypot(p[0]-clean[-1][0], p[1]-clean[-1][1]) > 1e-9: clean.append(p)
loop = clean
ca, sa = math.cos(alpha), math.sin(alpha)
def rot(p):  # nose-up rotation about (0.25,0): LE rises
    x, z = p[0]-0.25, p[1]
    return (0.25 + x*ca + z*sa, z*ca - x*sa)
loop = [rot(p) for p in loop]
cen = rot((0.5, 0.0))
tris = []
N = len(loop)
for i in range(N):
    p, q = loop[i], loop[(i+1) % N]
    A = (p[0], Y0, p[1]); B = (q[0], Y0, q[1]); Cc = (q[0], Y1, q[1]); D = (p[0], Y1, p[1])
    tris += [(A, B, Cc), (A, Cc, D)]
    tris.append(((cen[0], Y1, cen[1]), D, Cc))      # cap at Y1, normal +y
    tris.append(((cen[0], Y0, cen[1]), B, A))       # cap at Y0, normal -y
# closedness / orientation checks
from collections import Counter
edges = Counter()
for (u, v, w) in tris:
    for e in ((u, v), (v, w), (w, u)): edges[e] += 1
bad = [e for e in edges if edges.get((e[1], e[0]), 0) != edges[e]]
vol = 0.0
for (u, v, w) in tris:
    vol += np.dot(u, np.cross(v, w))/6.0
sys.stderr.write("tris=%d nonmatching_edges=%d signed_volume=%.5f (expected ~ +%.5f)\n"
                 % (len(tris), len(bad), vol, 1.6*(0.9*t*0.95)))
if bad or vol <= 0: sys.exit("STL orientation/closure check failed")
with open(out, "w") as f:
    f.write("solid plate\n")
    for (u, v, w) in tris:
        nrm = np.cross(np.subtract(v, u), np.subtract(w, u)); nn = np.linalg.norm(nrm)
        if nn == 0: continue
        nrm /= nn
        f.write(" facet normal %.6e %.6e %.6e\n  outer loop\n" % tuple(nrm))
        for P in (u, v, w): f.write("   vertex %.8e %.8e %.8e\n" % P)
        f.write("  endloop\n endfacet\n")
    f.write("endsolid plate\n")
EOF
python3 geometry/make_plate.py $ALPHA constant/triSurface/plate.stl

H='FoamFile { version 2.0; format ascii; class dictionary; object x; }'
hdr() { printf 'FoamFile\n{\n    version 2.0;\n    format ascii;\n    class %s;\n    object %s;\n}\n' "$1" "$2"; }

# ---------- system ----------
{ hdr dictionary blockMeshDict; cat <<EOF
convertToMeters 1;
vertices ( (-10 0 -8) (15 0 -8) (15 7.5 -8) (-10 7.5 -8) (-10 0 8) (15 0 8) (15 7.5 8) (-10 7.5 8) );
blocks ( hex (0 1 2 3 4 5 6 7) (100 30 64) simpleGrading (1 1 1) );
edges ();
boundary
(
  inlet  { type patch; faces ((0 4 7 3)); }
  outlet { type patch; faces ((1 2 6 5)); }
  bottom { type patch; faces ((0 3 2 1)); }
  top    { type patch; faces ((4 5 6 7)); }
  side   { type patch; faces ((3 7 6 2)); }
  sym    { type symmetryPlane; faces ((0 1 5 4)); }
);
EOF
} > system/blockMeshDict

{ hdr dictionary snappyHexMeshDict; cat <<EOF
castellatedMesh true;
snap true;
addLayers true;
geometry
{
  plate { type triSurfaceMesh; file "plate.stl"; }
  leBox { type searchableBox; min (-0.03 -1 -0.04); max (0.06 1.56 0.05); }
  teBox { type searchableBox; min (0.90 -1 -0.07); max (1.03 1.56 0.03); }
  tipBox{ type searchableBox; min (-0.1 1.3 -0.12); max (1.25 1.8 0.12); }
  nearBox { type searchableBox; min (-0.4 -1 -0.40); max (1.6 2.0 0.40); }
  wake1 { type searchableBox; min (0.0 -1 -0.30); max (2.5 1.9 0.30); }
  wake2 { type searchableBox; min (-0.5 -1 -1.0); max (6.0 2.8 1.0); }
}
castellatedMeshControls
{
  maxLocalCells 4000000; maxGlobalCells 20000000; minRefinementCells 10;
  maxLoadUnbalance 0.2; nCellsBetweenLevels 3;
  features ();
  refinementSurfaces { plate { level ($SURF $SURF); patchInfo { type wall; } } }
  resolveFeatureAngle 30;
  refinementRegions
  {
    plate { mode distance; levels $SHELL; }
    leBox { mode inside; levels ((1e15 $LETE)); }
    teBox { mode inside; levels ((1e15 $LETE)); }
    tipBox{ mode inside; levels ((1e15 $TIPL)); }
    nearBox { mode inside; levels ((1e15 $NEAR)); }
    wake1 { mode inside; levels ((1e15 $WAKE1)); }
    wake2 { mode inside; levels ((1e15 $WAKE2)); }
  }
  locationInMesh (-5.1 3.3 2.7);
  allowFreeStandingZoneFaces true;
}
snapControls
{
  nSmoothPatch 3; tolerance 2.0; nSolveIter 60; nRelaxIter 5; nFeatureSnapIter 10;
  implicitFeatureSnap false; explicitFeatureSnap false; multiRegionFeatureSnap false;
}
addLayersControls
{
  relativeSizes true;
  layers { "plate.*" { nSurfaceLayers 5; } }
  expansionRatio 1.2; finalLayerThickness 0.5; minThickness 0.05;
  nGrow 0; featureAngle 130; slipFeatureAngle 30;
  nRelaxIter 5; nSmoothSurfaceNormals 1; nSmoothNormals 3; nSmoothThickness 10;
  maxFaceThicknessRatio 0.5; maxThicknessToMedialRatio 0.3; minMedialAxisAngle 90;
  nBufferCellsNoExtrude 0; nLayerIter 50; nRelaxedIter 20;
}
meshQualityControls
{
  maxNonOrtho 65; maxBoundarySkewness 20; maxInternalSkewness 4; maxConcave 80;
  minVol 1e-13; minTetQuality 1e-15; minArea -1; minTwist 0.02; minDeterminant 0.001;
  minFaceWeight 0.05; minVolRatio 0.01; minTriangleTwist -1; nSmoothScale 4; errorReduction 0.75;
  relaxed { maxNonOrtho 75; }
}
mergeTolerance 1e-6;
EOF
} > system/snappyHexMeshDict

{ hdr dictionary decomposeParDict; cat <<EOF
numberOfSubdomains 12;
method scotch;
EOF
} > system/decomposeParDict

{ hdr dictionary controlDict; cat <<EOF
application pimpleFoam;
startFrom latestTime;
startTime 0;
stopAt endTime;
endTime $ENDT;
deltaT 1e-5;
writeControl adjustableRunTime;
writeInterval 5;
purgeWrite 2;
writeFormat binary;
writePrecision 8;
timeFormat general;
timePrecision 8;
runTimeModifiable false;
adjustTimeStep yes;
maxCo 1.5;
maxDeltaT 0.02;
functions
{
  forceCoeffs1
  {
    type forceCoeffs; libs (forces);
    writeControl timeStep; writeInterval 1;
    patches (plate);
    rho rhoInf; rhoInf 1;
    liftDir (0 0 1); dragDir (1 0 0);
    CofR (0.25 0 0); pitchAxis (0 1 0);
    magUInf 1; lRef 1; Aref 1.5;
  }
  yPlus1
  {
    type yPlus; libs (fieldFunctionObjects);
    writeControl writeTime; patches (plate);
  }
}
EOF
} > system/controlDict

{ hdr dictionary fvSchemes; cat <<EOF
ddtSchemes { default backward; }
gradSchemes { default Gauss linear; }
divSchemes
{
  default none;
  div(phi,U) Gauss limitedLinearV 1;
  div((nuEff*dev2(T(grad(U))))) Gauss linear;
}
laplacianSchemes { default Gauss linear corrected; }
interpolationSchemes { default linear; }
snGradSchemes { default corrected; }
wallDist { method meshWave; }
EOF
} > system/fvSchemes

{ hdr dictionary fvSolution; cat <<EOF
solvers
{
  p { solver GAMG; smoother GaussSeidel; tolerance 1e-6; relTol 0.05; }
  pFinal { \$p; relTol 0; }
  "U.*" { solver smoothSolver; smoother symGaussSeidel; tolerance 1e-9; relTol 0; }
}
PIMPLE
{
  momentumPredictor yes;
  nOuterCorrectors 1;
  nCorrectors 3;
  nNonOrthogonalCorrectors 1;
}
EOF
} > system/fvSolution

# ---------- constant ----------
{ hdr dictionary transportProperties; echo "transportModel Newtonian; nu $NU;"; } > constant/transportProperties
{ hdr dictionary momentumTransport; echo "simulationType laminar;"; } > constant/momentumTransport
cp constant/momentumTransport constant/turbulenceProperties

# ---------- 0.orig ----------
{ hdr volVectorField U; cat <<EOF
dimensions [0 1 -1 0 0 0 0];
internalField uniform (1 0 0);
boundaryField
{
  inlet  { type fixedValue; value uniform (1 0 0); }
  outlet { type inletOutlet; inletValue uniform (1 0 0); value uniform (1 0 0); }
  bottom { type slip; }
  top    { type slip; }
  side   { type slip; }
  sym    { type symmetryPlane; }
  plate  { type noSlip; }
}
EOF
} > 0.orig/U
{ hdr volScalarField p; cat <<EOF
dimensions [0 2 -2 0 0 0 0];
internalField uniform 0;
boundaryField
{
  inlet  { type zeroGradient; }
  outlet { type fixedValue; value uniform 0; }
  bottom { type zeroGradient; }
  top    { type zeroGradient; }
  side   { type zeroGradient; }
  sym    { type symmetryPlane; }
  plate  { type zeroGradient; }
}
EOF
} > 0.orig/p
echo "case $C ready (alpha=$ALPHA profile=$PROFILE nu=$NU)"
