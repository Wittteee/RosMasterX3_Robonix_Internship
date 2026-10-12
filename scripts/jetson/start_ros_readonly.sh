#!/usr/bin/env bash
set -euo pipefail
export LC_ALL=C.UTF-8
set +u
source /opt/ros/melodic/setup.bash
set -u
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
task_logs="$HOME/rosmaster_x3_deploy/logs"
mkdir -p "$task_logs"
if ! timeout 3 rosnode list >/dev/null 2>&1; then
    nohup roscore >"$task_logs/roscore.log" 2>&1 </dev/null &
    echo "$!" >"$task_logs/roscore.pid"
    for task_attempt in $(seq 1 15); do
        timeout 2 rosnode list >/dev/null 2>&1 && break
        sleep 1
    done
fi
if ! ss -lnt | grep -q ':9090'; then
    nohup roslaunch rosbridge_server rosbridge_websocket.launch address:=127.0.0.1 port:=9090 >"$task_logs/rosbridge.log" 2>&1 </dev/null &
    echo "$!" >"$task_logs/rosbridge.pid"
    for task_attempt in $(seq 1 15); do
        ss -lnt | grep -q ':9090' && break
        sleep 1
    done
fi
rosnode list
rostopic list
ss -lnt | grep -E '11311|9090'
echo 'Only ROS Master and rosbridge were started; no chassis or navigation goal was started.'
