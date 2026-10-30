#!/usr/bin/env bash
grep -n -A 18 -B 3 -E 'Traceback|Error|Exception|driver_node.*died|cannot launch' "$HOME/rosmaster_x3_deploy/live_test/sensors.log" || true
