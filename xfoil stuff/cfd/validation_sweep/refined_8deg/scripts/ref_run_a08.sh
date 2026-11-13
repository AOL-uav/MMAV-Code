#!/bin/bash
# Refined 8 deg case vp_ref_a08, 8 ranks, nice 19.
#   fresh start : maps the coarse laminar 8 deg solution (vp_sweep_a08, t = 20, read only), decomposes, runs.
#   resume      : if the case already has complete written time directories, continues from the LATEST COMPLETE one (no re-mapping, no re-decomposing);
#                 time directories newer than it that are incomplete (a write interrupted by a kill) are moved aside to _incomplete_<t>;
#                 fieldAverage continues (restartOnRestart false: the mean fields and the averaging time are read back from <t>/ and <t>/uniform).
# Test hooks (for the dry test only): CASE_DIR=<dir> NPROCS=<n> DRY_RUN=1 (prints the decision and stops before doing anything).
source /usr/lib/openfoam/openfoam2606/etc/bashrc >/dev/null 2>&1
R=/home/cfd/run; C=${CASE_DIR:-$R/vp_ref_a08}; SRC=$R/diag_a08src; NP=${NPROCS:-8}; cd $C || exit 1
RESUME_T=""
times=$(ls -d processor0/[0-9]* 2>/dev/null | sed 's#processor0/##' | grep -v '^0$' | sort -g -r)
for t in $times; do
  ok=1
  for i in $(seq 0 $((NP-1))); do
    for f in U p phi; do [ -f processor$i/$t/$f ] || ok=0; done
  done
  if [ $ok = 1 ]; then RESUME_T=$t; break; fi
done
if [ -n "$RESUME_T" ]; then
  echo "DECISION: resume from t = $RESUME_T (latest complete time directory); newer incomplete directories moved aside:"
  for t in $times; do
    if awk -v a="$t" -v b="$RESUME_T" 'BEGIN{exit !(a>b)}'; then echo "  newer than resume time: $t"; fi
  done
else
  echo "DECISION: fresh start (no complete written time directory): map coarse 8 deg t = 20, decompose to $NP, run"
fi
[ -n "$DRY_RUN" ] && exit 0
rm -f DONE FAILED
if [ -n "$RESUME_T" ]; then
  for t in $times; do
    if awk -v a="$t" -v b="$RESUME_T" 'BEGIN{exit !(a>b)}'; then
      for i in $(seq 0 $((NP-1))); do mv processor$i/$t processor$i/_incomplete_$t; done
    fi
  done
  echo "=== resumed at $(date) from t = $RESUME_T" >> log.pimpleFoam
else
  if [ ! -d $SRC/20 ]; then
    rm -rf $SRC; mkdir -p $SRC/system $SRC/constant
    cp -r $R/vp_sweep_a08/constant/polyMesh $R/vp_sweep_a08/constant/transportProperties $R/vp_sweep_a08/constant/turbulenceProperties $SRC/constant/
    cp $R/vp_sweep_a08/system/controlDict $R/vp_sweep_a08/system/fvSchemes $R/vp_sweep_a08/system/fvSolution $R/vp_sweep_a08/system/decomposeParDict $SRC/system/
    for i in $(seq 0 11); do mkdir -p $SRC/processor$i/20; cp -r $R/vp_sweep_a08/processor$i/constant $SRC/processor$i/; cp $R/vp_sweep_a08/processor$i/20/U $R/vp_sweep_a08/processor$i/20/p $SRC/processor$i/20/; done
    ( cd $SRC && nice -n 19 reconstructPar -time 20 > log.reconstructPar 2>&1 )
  fi
  rm -rf processor* 0 postProcessing log.pimpleFoam log.mapFields mem.log
  cp -r 0.orig 0
  printf 'FoamFile { version 2.0; format ascii; class dictionary; object mapFieldsDict; }\npatchMap ();\ncuttingPatches ();\n' > system/mapFieldsDict
  nice -n 19 mapFields $SRC -sourceTime 20 -mapMethod cellPointInterpolate > log.mapFields 2>&1
  tail -2 log.mapFields
  nice -n 19 decomposePar -force > log.decomposeFields 2>&1
  rm -rf 0
fi
s=$(date +%s)
( while true; do sleep 60; r=$(ps -C pimpleFoam -o rss= | awk '{s+=$1} END{print s/1048576}'); echo "$(date +%s) $r" >> mem.log; done ) & SAMP=$!
nice -n 19 mpirun -np $NP pimpleFoam -parallel >> log.pimpleFoam 2>&1
rc=$?; kill $SAMP 2>/dev/null
e=$(date +%s); echo "solver wall $((e-s)) s rc=$rc" >> wall.txt
last=$(grep "^Time =" log.pimpleFoam | tail -1 | awk '{print $3}')
if grep -q "^End" log.pimpleFoam; then echo "last t=$last" > DONE; else echo "rc=$rc last t=$last: $(grep -m1 -E 'FATAL|Floating|sigFpe' log.pimpleFoam)" > FAILED; fi
