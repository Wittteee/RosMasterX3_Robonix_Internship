#!/usr/bin/env bash
tail -65 "$HOME/rosmaster_x3_deploy/live_test/sensors.log"
ls -l /dev/ttyUSB* /dev/myserial
ps -eo pid,args | grep -E '[p]inned_chassis|[M]cnamu_driver|[r]oslaunch'
