set -e
set -o pipefail
export LC_ALL=C.UTF-8
TASK=/home/jetson/rosmaster_x3_runtime/obstacle2_20261121
source /opt/ros/melodic/setup.bash
source "$TASK/build_ws/devel/setup.bash"
cd "$TASK/build_ws"
catkin_make target_task_manager -j2 -l2 2>&1 | tee "$TASK/manager_rebuild.log"
catkin_make run_tests_x3_target_navigation -j2 -l2 2>&1 | tee "$TASK/ros_tests.log"
catkin_test_results build/test_results/x3_target_navigation
sha256sum -c "$TASK/function1_binary.sha256"
