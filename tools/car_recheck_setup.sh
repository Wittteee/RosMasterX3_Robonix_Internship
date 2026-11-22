set -e
export LC_ALL=C.UTF-8
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
source /opt/ros/melodic/setup.bash
source /home/jetson/yahboomcar_ws/devel/setup.bash
source /home/jetson/software/library_ws/devel/setup.bash
TRIAL=/home/jetson/rosmaster_x3_runtime/obstacle1_20261121_raised_trial
mkdir -p "$TRIAL"
cp /home/jetson/rosmaster_x3_runtime/obstacle1_20261121_5cm/camera_latest.png "$TRIAL/before.png"
printf 'Current control mode and state\n'
rosparam get /target_navigation/target_task_manager/execute_enabled
timeout 4 rostopic echo -n 1 /target_navigation/state || true
printf 'Recent fused target\n'
timeout 8 rostopic echo -n 1 /target_navigation/fused_target || true
printf 'RGB detector diagnostics\n'
rosnode info /target_navigation/rgbd_target_node | head -25
printf 'Recent detector warnings\n'
tail -n 14 /home/jetson/rosmaster_x3_runtime/obstacle1_20261121_5cm/preview_runtime.log
