#!/bin/bash
pkill -u cfd -f ref_launch.sh; pkill -u cfd -f ref_run.sh; sleep 1; pkill -u cfd -f "pimpleFoam -parallel"; rm -f /home/cfd/run/ref/ref.pid; echo stopped
