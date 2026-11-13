#!/bin/bash
# Angle-of-attack sweep driver for the Ananda AR3 flat-plate validation (coarse mesh recipe, Re 60k).
# Runs cases strictly one after another. A failed case writes FAILED and the queue continues.
# Per case: <case>/DONE or <case>/FAILED (reason inside). Overall log: /home/cfd/run/sweep/driver.log
# Usage: sweep_driver.sh   (normally started through sweep_launch.sh)
S=/home/cfd/run/sweep
LOG=$S/driver.log
source /usr/lib/openfoam/openfoam2606/etc/bashrc >/dev/null 2>&1
export OMPI_MCA_rmaps_base_oversubscribe=0
# order: useful angles first. "alpha:endTime(chord passages)"
QUEUE="${SWEEP_QUEUE:-2:20 6:20 8:20 0:20 10:40 12:40}"
say() { echo "$(date '+%F %T') $*" | tee -a $LOG; }
echo $$ > $S/driver.pid
say "driver start pid $$  queue: $QUEUE"
for item in $QUEUE; do
  A=${item%%:*}; END=${item##*:}
  N=$(printf "vp_sweep_a%02d" $A); C=/home/cfd/run/$N
  say "=== $N alpha=$A endTime=$END"
  if [ -f $C/DONE ]; then say "$N already DONE, skipping"; continue; fi
  rm -f $C/DONE $C/FAILED
  fail() { say "$N FAILED: $1"; mkdir -p $C; echo "$(date '+%F %T') $1" > $C/FAILED; }
  # 1. case files (same recipe as vp_a4_coarse; only the plate rotation angle differs)
  bash $S/mkcase.sh $N $A coarse $END > $S/$N.mkcase.log 2>&1 || { fail "mkcase"; continue; }
  # 2. mesh: snappyHexMesh on 12 ranks, reconstructed to a serial mesh
  bash $S/mesh.sh $N > $S/$N.mesh.log 2>&1
  if ! grep -q "^Mesh OK" $C/log.checkMeshDefault 2>/dev/null; then
    ( cd $C && checkMesh > log.checkMeshDefault 2>&1 )
  fi
  if ! grep -q "Mesh OK" $C/log.checkMeshDefault 2>/dev/null; then fail "default checkMesh not OK (see $C/log.checkMeshDefault)"; continue; fi
  say "$N mesh: $(grep -E 'cells:' $C/log.checkMeshDefault | head -1 | tr -s ' ')"
  # 3. solve on 12 ranks
  t0=$(date +%s)
  bash $S/solve.sh $N $END > $S/$N.solve.log 2>&1
  t1=$(date +%s)
  last=$(grep "^Time =" $C/log.pimpleFoam | tail -1 | awk '{print $3}')
  if grep -q "^End" $C/log.pimpleFoam && awk -v t="$last" -v e=$END 'BEGIN{exit !(t>=e-1e-6)}'; then
    echo "alpha=$A end=$END last=$last wall=$((t1-t0))s" > $C/DONE
    say "$N DONE last t=$last wall $((t1-t0)) s"
  else
    fail "solver did not reach endTime (last t=$last): $(grep -m1 -E 'FATAL|Floating point|sigFpe' $C/log.pimpleFoam | head -1)"
  fi
done
say "driver finished all cases"
rm -f $S/driver.pid
