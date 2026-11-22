set -e
export LC_ALL=C.UTF-8
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
source /opt/ros/melodic/setup.bash
source /home/jetson/yahboomcar_ws/devel/setup.bash
source /home/jetson/software/library_ws/devel/setup.bash
rosnode list
printf 'Target state\n'
timeout 4 rostopic echo -n 1 /target_navigation/state || true
printf 'Chassis voltage and version\n'
timeout 3 rostopic echo -n 1 /voltage || true
timeout 3 rostopic echo -n 1 /edition || true
printf 'Pose\n'
timeout 4 rostopic echo -n 1 /amcl_pose || true
printf 'Fused target\n'
timeout 6 rostopic echo -n 1 /target_navigation/fused_target || true
printf 'Recent launch messages\n'
tail -n 45 /home/jetson/rosmaster_x3_runtime/obstacle1_20261121_5cm/runtime.log
