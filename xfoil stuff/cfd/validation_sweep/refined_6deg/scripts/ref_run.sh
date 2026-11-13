#!/bin/bash
# Prepares and runs the refined 6 deg case on 12 ranks. Start: coarse laminar 6 deg solution (t = 20) mapped onto the refined mesh.
source /usr/lib/openfoam/openfoam2606/etc/bashrc >/dev/null 2>&1
R=/home/cfd/run; C=$R/vp_ref_a06; SRC=$R/diag_a06src; cd $C || exit 1
rm -f DONE FAILED
if [ ! -d $SRC/20 ]; then
  rm -rf $SRC; mkdir -p $SRC/system $SRC/constant
  cp -r $R/vp_sweep_a06/constant/polyMesh $R/vp_sweep_a06/constant/transportProperties $R/vp_sweep_a06/constant/turbulenceProperties $SRC/constant/
  cp $R/vp_sweep_a06/system/controlDict $R/vp_sweep_a06/system/fvSchemes $R/vp_sweep_a06/system/fvSolution $R/vp_sweep_a06/system/decomposeParDict $SRC/system/
  for i in $(seq 0 11); do mkdir -p $SRC/processor$i/20; cp -r $R/vp_sweep_a06/processor$i/constant $SRC/processor$i/; cp $R/vp_sweep_a06/processor$i/20/U $R/vp_sweep_a06/processor$i/20/p $SRC/processor$i/20/; done
  ( cd $SRC && reconstructPar -time 20 > log.reconstructPar 2>&1 )
fi
rm -rf processor* 0 postProcessing log.pimpleFoam log.mapFields
cp -r 0.orig 0
printf 'FoamFile { version 2.0; format ascii; class dictionary; object mapFieldsDict; }\npatchMap ();\ncuttingPatches ();\n' > system/mapFieldsDict
mapFields $SRC -sourceTime 20 -mapMethod cellPointInterpolate > log.mapFields 2>&1
tail -2 log.mapFields
decomposePar -force > log.decomposeFields 2>&1
rm -rf 0
s=$(date +%s)
( while true; do sleep 60; r=$(ps -C pimpleFoam -o rss= | awk '{s+=$1} END{print s/1048576}'); echo "$(date +%s) $r" >> mem.log; done ) & SAMP=$!
mpirun -np 12 pimpleFoam -parallel > log.pimpleFoam 2>&1
rc=$?; kill $SAMP 2>/dev/null
e=$(date +%s); echo "solver wall $((e-s)) s rc=$rc" > wall.txt
last=$(grep "^Time =" log.pimpleFoam | tail -1 | awk '{print $3}')
if grep -q "^End" log.pimpleFoam; then echo "last t=$last wall=$((e-s))" > DONE; else echo "rc=$rc last t=$last: $(grep -m1 -E 'FATAL|Floating|sigFpe' log.pimpleFoam)" > FAILED; fi
