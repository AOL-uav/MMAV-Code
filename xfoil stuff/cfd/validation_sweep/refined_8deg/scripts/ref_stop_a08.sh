#!/bin/bash
# Stops ONLY the vp_ref_a08 run (its own session); vp_ref_a06 and anything else is left alone.
L=/home/cfd/run/ref
if [ -f $L/ref_a08.pid ]; then pkill -u cfd -s $(cat $L/ref_a08.pid); fi
pkill -u cfd -f ref_launch_a08.sh
sleep 2; rm -f $L/ref_a08.pid; echo "stopped vp_ref_a08 (if it was running)"; pgrep -u cfd -a pimpleFoam | wc -l
