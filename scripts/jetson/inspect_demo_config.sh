#!/usr/bin/env bash
cat "$HOME/yahboomcar_ws/src/yahboomcar_bringup/launch/bringup.launch"
cat "$HOME/rosmaster_x3_deploy/live_test/bringup_no_joy.launch"
cat "$HOME/rosmaster_x3_deploy/live_test/sensors.launch"
cat "$HOME/rosmaster_x3_deploy/scripts/probe_control_board.py"
ls "$HOME/rosmaster_x3_deploy/scripts"
ls "$HOME/refs"
find "$HOME/yahboomcar_ws/src" -name '*driver*launch' -o -name '*bringup*launch'
