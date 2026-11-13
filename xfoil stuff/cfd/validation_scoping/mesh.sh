#!/bin/bash
# usage: mesh.sh <casename>   (snappy on 12 ranks, then reconstruct to one serial mesh)
source ~/.bashrc
cd /home/cfd/run/$1
s=$(date +%s)
rm -rf processor* log.* constant/polyMesh
blockMesh > log.blockMesh 2>&1
decomposePar > log.decomposePar 2>&1
mpirun -np 12 snappyHexMesh -parallel -overwrite > log.snappyHexMesh 2>&1
e=$(date +%s); echo "snappy wall $((e-s)) s"
rm -rf constant/polyMesh
reconstructParMesh -constant -mergeTol 1e-6 > log.reconstructParMesh 2>&1
rm -rf processor*
checkMesh -allGeometry -allTopology > log.checkMesh 2>&1
grep -E "cells:|Mesh OK|Failed|non-orthogonality|skewness|aspect|plate" log.checkMesh
grep -A4 "^patch faces" log.snappyHexMesh | tail -12
