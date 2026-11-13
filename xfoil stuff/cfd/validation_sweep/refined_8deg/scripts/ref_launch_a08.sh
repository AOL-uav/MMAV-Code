#!/bin/bash
# Detached launcher for vp_ref_a08 (own session so that it can be stopped by session id; stays attached so the Windows-side wsl.exe keeps the distro alive).
L=/home/cfd/run/ref
if [ -f $L/ref_a08.pid ] && kill -0 $(cat $L/ref_a08.pid) 2>/dev/null; then echo "already running"; exit 1; fi
setsid nohup nice -n 19 bash $L/ref_run_a08.sh > $L/ref_run_a08.log 2>&1 < /dev/null &
echo $! > $L/ref_a08.pid; echo "launched $(cat $L/ref_a08.pid) $(date)" >> $L/ref_launch_a08.log
sleep 5
while kill -0 $(cat $L/ref_a08.pid) 2>/dev/null; do sleep 30; done
rm -f $L/ref_a08.pid
