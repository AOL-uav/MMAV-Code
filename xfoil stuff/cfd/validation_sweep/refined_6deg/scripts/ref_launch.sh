#!/bin/bash
# Detached launcher; stays attached so the Windows-side wsl.exe keeps the distro alive.
L=/home/cfd/run/ref
if [ -f $L/ref.pid ] && kill -0 $(cat $L/ref.pid) 2>/dev/null; then echo "already running"; exit 1; fi
setsid nohup bash $L/ref_run.sh > $L/ref_run.log 2>&1 < /dev/null &
echo $! > $L/ref.pid; echo "launched $(cat $L/ref.pid) $(date)" >> $L/ref_launch.log
sleep 5
while kill -0 $(cat $L/ref.pid) 2>/dev/null; do sleep 30; done
rm -f $L/ref.pid
