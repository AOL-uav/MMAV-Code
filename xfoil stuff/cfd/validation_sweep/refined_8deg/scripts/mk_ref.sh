#!/bin/bash
# usage: mk_ref.sh <case> <alpha_deg> <endTime> [avgStart]
# Refined LAMINAR case: coarse recipe (mkcase.sh coarse) with a locally refined upper-surface shear-layer region and
# 6 relative near-wall layers (first layer 0.2 of the local cell: 0.0004 c at level 7, 0.0008 c at level 6 -> y+ about 1).  Plate rotated about (0.25, 0) as in all sweep cases.
set -e
N=$1; A=$2; END=$3; AVG=${4:-8}
S=/home/cfd/run/sweep
bash $S/mkcase.sh $N $A coarse $END > /dev/null
C=/home/cfd/run/$N; cd $C
# rotated-box corners in the lab frame: body (xb, y, zb) -> lab x = 0.25+(xb-0.25)cos+zb sin ; z = zb cos-(xb-0.25) sin
read E1X E1Z E3X E3Z <<< $(python3 -c "import math;a=math.radians($A);print(math.cos(a),-math.sin(a),math.sin(a),math.cos(a))")
corner() { python3 -c "import math;a=math.radians($A);xb,y,zb=$1,$2,$3;print('(%.6f %.6f %.6f)'%(0.25+(xb-0.25)*math.cos(a)+zb*math.sin(a),y,zb*math.cos(a)-(xb-0.25)*math.sin(a)))"; }
# Box7: x_b -0.04..0.20, y -0.1..1.62, z_b 0..0.0414 (about 0.02 c above the upper surface); Box6: x_b -0.07..0.85, y -0.1..1.65, z_b 0..0.0714 (about 0.05 c above)
O7=$(corner -0.04 -0.1 0.0);  O6=$(corner -0.07 -0.1 0.0)
cat > system/snappyHexMeshDict <<EOF2
FoamFile { version 2.0; format ascii; class dictionary; object snappyHexMeshDict; }
castellatedMesh true;
snap true;
addLayers true;
geometry
{
  plate { type triSurfaceMesh; file "plate.stl"; }
  box7 { type rotatedBox; span (0.24 1.72 0.0414); origin $O7; e1 ($E1X 0 $E1Z); e3 ($E3X 0 $E3Z); }
  box6 { type rotatedBox; span (0.92 1.75 0.0714); origin $O6; e1 ($E1X 0 $E1Z); e3 ($E3X 0 $E3Z); }
  nearBox { type searchableBox; min (-0.4 -1 -0.40); max (1.6 2.0 0.40); }
  wake1 { type searchableBox; min (0.0 -1 -0.30); max (2.5 1.9 0.30); }
  wake2 { type searchableBox; min (-0.5 -1 -1.0); max (6.0 2.8 1.0); }
}
castellatedMeshControls
{
  maxLocalCells 4000000; maxGlobalCells 20000000; minRefinementCells 10;
  maxLoadUnbalance 0.2; nCellsBetweenLevels 3;
  features ();
  refinementSurfaces { plate { level (5 5); patchInfo { type wall; } } }
  resolveFeatureAngle 30;
  refinementRegions
  {
    plate { mode distance; levels ((0.03 5) (0.08 4) (0.20 3)); }
    box7 { mode inside; levels ((1e15 7)); }
    box6 { mode inside; levels ((1e15 6)); }
    nearBox { mode inside; levels ((1e15 3)); }
    wake1 { mode inside; levels ((1e15 3)); }
    wake2 { mode inside; levels ((1e15 2)); }
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
  layers { "plate.*" { nSurfaceLayers 6; } }
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
EOF2
# controlDict: same forceCoeffs/yPlus, plus wall shear and time averages (fields U p wallShearStress), one write per chord passage, all times kept
cat > system/controlDict <<EOF2
FoamFile { version 2.0; format ascii; class dictionary; object controlDict; }
application pimpleFoam;
startFrom latestTime;
startTime 0;
stopAt endTime;
endTime $END;
deltaT 1e-05;
writeControl adjustableRunTime;
writeInterval 1;
purgeWrite 0;
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
  yPlus1 { type yPlus; libs (fieldFunctionObjects); writeControl writeTime; patches (plate); }
  wallShearStress1 { type wallShearStress; libs (fieldFunctionObjects); writeControl writeTime; patches (plate); }
  fieldAverage1
  {
    type fieldAverage; libs (fieldFunctionObjects);
    writeControl writeTime;
    timeStart $AVG;
    fields
    (
      U { mean on; prime2Mean off; base time; }
      p { mean on; prime2Mean off; base time; }
      wallShearStress { mean on; prime2Mean off; base time; }
    );
  }
}
EOF2
echo "case $C built: alpha $A end $END avgStart $AVG"
