set -e
set -o pipefail
export LC_ALL=C.UTF-8
TASK=/home/jetson/rosmaster_x3_runtime/obstacle2_20261121
source /opt/ros/melodic/setup.bash
source "$TASK/build_ws/devel/setup.bash"
rostest x3_target_navigation bypass_behavior.test scenario:=right --results-base-dir "$TASK/five_cm_results" 2>&1 | tee "$TASK/five_cm_right_test.log"
