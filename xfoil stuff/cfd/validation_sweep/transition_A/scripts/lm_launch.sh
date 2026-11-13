#!/bin/bash
# Starts both transition runs concurrently (6 MPI ranks each), detached; stays attached to keep the distro alive.
L=/home/cfd/run/lm
if [ -f $L/launch.pid ] && kill -0 $(cat $L/launch.pid) 2>/dev/null; then echo "already running"; exit 1; fi
echo $$ > $L/launch.pid
for c in vp_lm_a04 vp_lm_a08; do
  setsid nohup bash $L/lm_run.sh $c 6 > $L/$c.run.log 2>&1 < /dev/null &
  echo $! > $L/$c.pid
done
sleep 5
echo "launched: $(cat $L/vp_lm_a04.pid) $(cat $L/vp_lm_a08.pid)" | tee -a $L/launch.log
while kill -0 $(cat $L/vp_lm_a04.pid) 2>/dev/null || kill -0 $(cat $L/vp_lm_a08.pid) 2>/dev/null; do sleep 30; done
rm -f $L/launch.pid
