#!/bin/bash
# usage: solve.sh <case> [endTime]
source ~/.bashrc
cd /home/cfd/run/$1
[ -n "$2" ] && foamDictionary system/controlDict -entry endTime -set $2 >/dev/null
foamDictionary system/controlDict -entry deltaT -set 1e-5 >/dev/null
rm -rf postProcessing log.pimpleFoam log.potentialFoam processor* 0 [1-9]*
cp -r 0.orig 0
potentialFoam -initialiseUBCs -writep > log.potentialFoam 2>&1
tail -3 log.potentialFoam
decomposePar -force > log.decomposeFields 2>&1
rm -rf 0
s=$(date +%s)
mpirun -np 12 pimpleFoam -parallel > log.pimpleFoam 2>&1
e=$(date +%s); echo "solver wall $((e-s)) s" | tee wall.txt
