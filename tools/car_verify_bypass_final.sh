set -e
export LC_ALL=C.UTF-8
source /opt/ros/melodic/setup.bash
source /home/jetson/yahboomcar_ws/devel/setup.bash
TASK=/home/jetson/rosmaster_x3_runtime/obstacle2_20261121
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=10.206.170.35
sha256sum -c "$TASK/function1_binary.sha256"
sha256sum "$TASK/build_ws/devel/lib/x3_target_navigation/target_task_manager"
timeout 6 rostopic echo -n 1 /target_navigation/state
timeout 6 rostopic echo -n 1 /target_navigation/motion_permission
timeout 6 rostopic echo -n 1 /odom/twist/twist
