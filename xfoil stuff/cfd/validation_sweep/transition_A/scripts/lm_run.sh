#!/bin/bash
# usage: lm_run.sh <case> <nprocs>
source /usr/lib/openfoam/openfoam2606/etc/bashrc >/dev/null 2>&1
C=/home/cfd/run/$1; NP=$2; cd $C || exit 1
rm -rf processor* 0 postProcessing log.pimpleFoam DONE FAILED
cp -r 0.orig 0
decomposePar -force > log.decomposePar 2>&1
rm -rf 0
s=$(date +%s)
( while true; do sleep 60; r=$(ps -C pimpleFoam -o rss= | awk '{s+=$1} END{print s/1048576}'); echo "$(date +%s) $r" >> mem_all.log; done ) & SAMP=$!
mpirun -np $NP pimpleFoam -parallel > log.pimpleFoam 2>&1
rc=$?; kill $SAMP 2>/dev/null
e=$(date +%s); echo "solver wall $((e-s)) s rc=$rc" > wall.txt
last=$(grep "^Time =" log.pimpleFoam | tail -1 | awk '{print $3}')
if grep -q "^End" log.pimpleFoam; then echo "last t=$last wall=$((e-s))" > DONE; else echo "rc=$rc last t=$last: $(grep -m1 -E 'FATAL|Floating|sigFpe' log.pimpleFoam)" > FAILED; fi
