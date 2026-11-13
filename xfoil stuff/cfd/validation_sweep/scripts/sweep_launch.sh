#!/bin/bash
# Starts the driver detached (own session, immune to hangup) and then stays attached to it, so that
# the Windows-side wsl.exe process that runs THIS script keeps the MMAV-CFD distro from idle shutdown.
S=/home/cfd/run/sweep
if [ -f $S/driver.pid ] && kill -0 $(cat $S/driver.pid) 2>/dev/null; then echo "driver already running pid $(cat $S/driver.pid)"; exit 1; fi
setsid nohup bash $S/sweep_driver.sh > $S/driver.stdout 2>&1 < /dev/null &
sleep 3
P=$(cat $S/driver.pid)
echo "launched driver pid $P"
# hold the session open until the driver exits
while kill -0 $P 2>/dev/null; do sleep 20; done
