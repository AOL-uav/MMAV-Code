#!/bin/bash
C=/home/cfd/run/vp_ref_a06
st=running; [ -f $C/DONE ] && st="DONE $(cat $C/DONE)"; [ -f $C/FAILED ] && st="FAILED $(cat $C/FAILED)"
t=$(grep "^Time =" $C/log.pimpleFoam 2>/dev/null | tail -1 | awk '{print $3}'); co=$(grep "^Courant" $C/log.pimpleFoam 2>/dev/null | tail -1 | awk '{print $6}')
et=$(grep ExecutionTime $C/log.pimpleFoam 2>/dev/null | tail -1 | awk '{print $3}'); n=$(grep -c "^Time =" $C/log.pimpleFoam 2>/dev/null)
echo "vp_ref_a06: $st  t=${t:-n/a}/20  steps=$n  maxCo=${co:-n/a}  exec=${et:-n/a}s  $(tail -1 $C/postProcessing/forceCoeffs1/0/coefficient.dat 2>/dev/null | awk '{printf "Cd=%s Cl=%s Cm=%s",$2,$5,$8}')"
[ -n "$t" ] && [ -n "$et" ] && python3 -c "t=$t;e=$et;print('rate: %.2f s per chord passage so far (incl. start-up); projected to t=20: %.1f h' % (e/t, e/t*20/3600))"
sort -k2 -n $C/mem.log 2>/dev/null | tail -1 | awk '{printf "peak memory so far %.1f GB\n",$2}'
free -m | sed -n 2p
