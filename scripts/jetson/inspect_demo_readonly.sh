#!/usr/bin/env bash
set -u
date -Iseconds
hostname
ip -4 -brief address
ls -l /dev/myserial /dev/rplidar /dev/ttyUSB* /dev/video* 2>/dev/null
ps -eo pid,args | grep -E '[r]osmaster_main|[r]oscore|[r]oslaunch|[m]ove_base|[a]mcl|[s]lam|[y]ahboomcar'
ss -lnt | grep -E ':11311|:9090|:6000'
ls -ld "$HOME/rosmaster_x3_deploy" "$HOME/x3_rosbridge_adapter" "$HOME/miniforge3/envs/robonix" 2>/dev/null
set +u
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
timeout 8 rosnode list
timeout 8 rostopic list
for topic in /edition /voltage /odom /scan /amcl_pose; do
 echo "CHECK $topic"
 timeout 4 rostopic echo -n 1 "$topic" | head -35
done
