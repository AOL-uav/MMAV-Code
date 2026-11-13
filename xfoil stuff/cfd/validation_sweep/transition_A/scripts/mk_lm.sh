#!/bin/bash
# usage: mk_lm.sh <newcase> <source_case_dir> <endTime> <nprocs>
# Builds a kOmegaSSTLM (transition) copy of an existing laminar sweep case: SAME mesh, same U/p BCs, same
# forceCoeffs definitions (CofR 0.25 0 0, pitchAxis 0 1 0, Aref 1.5). Only the turbulence set-up is new.
set -e
N=$1; SRC=$2; END=$3; NP=$4
C=/home/cfd/run/$N
[ -d $C ] && { echo "$C exists, refusing to overwrite"; exit 1; }
mkdir -p $C/constant $C/system $C/0.orig
cp -r $SRC/constant/polyMesh $C/constant/
cp $SRC/constant/transportProperties $C/constant/
cp $SRC/0.orig/U $SRC/0.orig/p $C/0.orig/
cd $C

# ---- inlet turbulence (c = 1, U = 1, nu = 1/60000).  ASSUMED: Tu = 0.1 % (Ananda 2015: tunnel Tu "less than 0.1%"), nut/nu = 10
NU=1.6666667e-05
TU=0.001
K=$(python3 -c "print('%.6g' % (1.5*$TU**2))")
OM=$(python3 -c "print('%.6g' % (1.5*$TU**2/(10*1.6666667e-05)))")
NUT=$(python3 -c "print('%.6g' % (10*1.6666667e-05))")
RET=$(python3 -c "tu=100*$TU; print('%.4f' % (1173.51-589.428*tu+0.2196/tu**2))")
echo "inlet: Tu=$TU k=$K omega=$OM nut=$NUT ReThetat=$RET gammaInt=1" | tee inlet_values.txt

hdr() { printf 'FoamFile\n{\n    version 2.0;\n    format ascii;\n    class %s;\n    object %s;\n}\n' "$1" "$2"; }

{ hdr dictionary turbulenceProperties; cat <<EOF
simulationType RAS;
RAS { RASModel kOmegaSSTLM; turbulence on; printCoeffs on; }
EOF
} > constant/turbulenceProperties

mkfield() { # name dims value wallbc outletbc
  { hdr volScalarField $1; cat <<EOF
dimensions $2;
internalField uniform $3;
boundaryField
{
  inlet  { type fixedValue; value uniform $3; }
  outlet { type inletOutlet; inletValue uniform $3; value uniform $3; }
  bottom { type zeroGradient; }
  top    { type zeroGradient; }
  side   { type zeroGradient; }
  sym    { type symmetryPlane; }
  plate  { $4 }
}
EOF
  } > 0.orig/$1
}
mkfield k "[0 2 -2 0 0 0 0]" $K "type fixedValue; value uniform 0;"
mkfield omega "[0 0 -1 0 0 0 0]" $OM "type omegaWallFunction; value uniform $OM;"
mkfield ReThetat "[0 0 0 0 0 0 0]" $RET "type zeroGradient;"
mkfield gammaInt "[0 0 0 0 0 0 0]" 1 "type zeroGradient;"
{ hdr volScalarField nut; cat <<EOF
dimensions [0 2 -1 0 0 0 0];
internalField uniform $NUT;
boundaryField
{
  inlet  { type calculated; value uniform $NUT; }
  outlet { type calculated; value uniform $NUT; }
  bottom { type calculated; value uniform $NUT; }
  top    { type calculated; value uniform $NUT; }
  side   { type calculated; value uniform $NUT; }
  sym    { type symmetryPlane; }
  plate  { type nutkWallFunction; value uniform 0; }
}
EOF
} > 0.orig/nut

{ hdr dictionary controlDict; cat <<EOF
application pimpleFoam;
startFrom latestTime;
startTime 0;
stopAt endTime;
endTime $END;
deltaT 1e-05;
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
  probes1
  {
    type probes; libs (sampling);
    writeControl timeStep; writeInterval 20;
    fields (k nut);
    probeLocations ( (-9.5 0.75 0.5) (-0.3 0.75 0.5) );
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
  div(phi,k) Gauss limitedLinear 1;
  div(phi,omega) Gauss limitedLinear 1;
  div(phi,gammaInt) Gauss limitedLinear 1;
  div(phi,ReThetat) Gauss limitedLinear 1;
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
  "(U|k|omega|gammaInt|ReThetat).*" { solver smoothSolver; smoother symGaussSeidel; tolerance 1e-9; relTol 0; }
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

{ hdr dictionary decomposeParDict; echo "numberOfSubdomains $NP; method scotch;"; } > system/decomposeParDict
touch case.foam
echo "built $C"
