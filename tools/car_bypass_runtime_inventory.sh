set -e
export LC_ALL=C.UTF-8
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
source /opt/ros/melodic/setup.bash
source /home/jetson/yahboomcar_ws/devel/setup.bash
ps -eo pid,ppid,args | grep -E 'roslaunch|target_task_manager|car_live|car_observe' | grep -v grep
rosnode info /driver_node | head -35
rosparam get /driver_node
