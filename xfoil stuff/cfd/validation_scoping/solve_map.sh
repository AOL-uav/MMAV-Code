#!/bin/bash
# usage: solve_map.sh <case> <source_case_dir> <endTime>  : start from coarse solution mapped onto this mesh
source ~/.bashrc
cd /home/cfd/run/$1
foamDictionary system/controlDict -entry endTime -set $3 >/dev/null
foamDictionary system/controlDict -entry deltaT -set 2e-4 >/dev/null
rm -rf postProcessing log.pimpleFoam processor* 0
cp -r 0.orig 0
( cd $2 && [ -d 20 ] || reconstructPar -time 20 > log.reconstructPar20 2>&1 )
mapFields $2 -sourceTime 20 -mapMethod cellPointInterpolate > log.mapFields 2>&1
tail -4 log.mapFields
decomposePar -force > log.decomposeFields 2>&1
rm -rf 0
s=$(date +%s)
mpirun -np 12 pimpleFoam -parallel > log.pimpleFoam 2>&1
e=$(date +%s); echo "solver wall $((e-s)) s" | tee wall.txt
