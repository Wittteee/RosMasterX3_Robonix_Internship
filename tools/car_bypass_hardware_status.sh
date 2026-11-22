set -e
export LC_ALL=C.UTF-8
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
source /opt/ros/melodic/setup.bash
source /home/jetson/rosmaster_x3_runtime/obstacle2_20261121/build_ws/devel/setup.bash
timeout 4 rostopic echo -n 1 /target_navigation/state || true
timeout 4 rostopic echo -n 1 /target_navigation/motion_permission || true
rosnode info /driver_node | head -26
tail -n 22 /home/jetson/rosmaster_x3_runtime/obstacle2_20261121/hardware_left_01/runtime.log
