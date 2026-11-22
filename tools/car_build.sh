set -e
export LC_ALL=C.UTF-8
source /opt/ros/melodic/setup.bash
source /home/jetson/software/library_ws/devel/setup.bash
cd /home/jetson/yahboomcar_ws
export CMAKE_PREFIX_PATH=/home/jetson/software/library_ws/devel:$CMAKE_PREFIX_PATH
catkin_make --only-pkg-with-deps x3_target_navigation rplidar_ros -j2 -l2 2>&1 | tee /home/jetson/rosmaster_x3_runtime/obstacle1_20261121_5cm/build.log
exit "${PIPESTATUS[0]}"
