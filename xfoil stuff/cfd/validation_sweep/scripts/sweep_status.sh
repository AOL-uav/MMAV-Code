#!/bin/bash
# One-command progress report for the sweep.
S=/home/cfd/run/sweep
echo "--- driver: $(pgrep -u cfd -f sweep_driver.sh | head -1 | sed 's/^/pid /') ; $(date '+%F %T')"
tail -n 8 $S/driver.log
for C in /home/cfd/run/vp_sweep_a*; do
  [ -d $C ] || continue
  st=running; [ -f $C/DONE ] && st=DONE; [ -f $C/FAILED ] && st="FAILED: $(cat $C/FAILED)"
  t=$(grep "^Time =" $C/log.pimpleFoam 2>/dev/null | tail -1 | awk '{print $3}')
  co=$(grep "^Courant" $C/log.pimpleFoam 2>/dev/null | tail -1 | awk '{print $6}')
  echo "$(basename $C): $st  t=${t:-n/a}  maxCo=${co:-n/a}  $(tail -1 $C/postProcessing/forceCoeffs1/0/coefficient.dat 2>/dev/null | awk '{printf "Cd=%s Cl=%s Cm=%s", $2,$5,$8}')"
done
