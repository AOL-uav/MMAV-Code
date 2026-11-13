#!/bin/bash
pkill -u cfd -f lm_launch.sh; pkill -u cfd -f lm_run.sh; sleep 1; pkill -u cfd -f "pimpleFoam -parallel"; rm -f /home/cfd/run/lm/launch.pid; echo stopped
