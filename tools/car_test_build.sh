set -e
export LC_ALL=C.UTF-8
source /opt/ros/melodic/setup.bash
source /home/jetson/yahboomcar_ws/devel/setup.bash
source /home/jetson/software/library_ws/devel/setup.bash
cd /home/jetson/yahboomcar_ws
set -o pipefail
catkin_make run_tests_x3_target_navigation -j2 -l2 2>&1 | tee /home/jetson/rosmaster_x3_runtime/obstacle1_20261121_5cm/ros_tests.log
catkin_test_results build/test_results/x3_target_navigation
