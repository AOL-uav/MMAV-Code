#!/bin/bash
# Stops the sweep: driver first (so it cannot start the next case), then the solver/mesher of the running case.
S=/home/cfd/run/sweep
[ -f $S/driver.pid ] && kill $(cat $S/driver.pid) 2>/dev/null
pkill -u cfd -f sweep_driver.sh
pkill -u cfd -f sweep_launch.sh
sleep 1
pkill -u cfd -f "pimpleFoam -parallel"
pkill -u cfd -f "snappyHexMesh -parallel"
sleep 2
echo "$(date '+%F %T') sweep stopped by sweep_stop.sh" >> $S/driver.log
rm -f $S/driver.pid
pgrep -u cfd -a "pimpleFoam|snappyHexMesh|sweep" || echo "nothing left running"
