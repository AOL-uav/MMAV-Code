#!/bin/bash
for c in vp_lm_a04 vp_lm_a08; do C=/home/cfd/run/$c
  st=running; [ -f $C/DONE ] && st="DONE $(cat $C/DONE)"; [ -f $C/FAILED ] && st="FAILED $(cat $C/FAILED)"
  t=$(grep "^Time =" $C/log.pimpleFoam 2>/dev/null | tail -1 | awk '{print $3}'); co=$(grep "^Courant" $C/log.pimpleFoam 2>/dev/null | tail -1 | awk '{print $6}')
  et=$(grep ExecutionTime $C/log.pimpleFoam 2>/dev/null | tail -1 | awk '{print $3}')
  echo "$c: $st  t=${t:-n/a}/30  maxCo=${co:-n/a}  exec=${et:-n/a}s  $(tail -1 $C/postProcessing/forceCoeffs1/0/coefficient.dat 2>/dev/null | awk '{printf "Cd=%s Cl=%s Cm=%s",$2,$5,$8}')"
done
free -m | sed -n 2p
