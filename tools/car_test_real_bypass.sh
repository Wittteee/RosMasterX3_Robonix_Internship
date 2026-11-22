set -e
set -o pipefail
export LC_ALL=C.UTF-8
export ROBOT_TYPE=X3
TASK=/home/jetson/rosmaster_x3_runtime/obstacle2_20261121
source /opt/ros/melodic/setup.bash
source "$TASK/build_ws/devel/setup.bash"
scenario=real_far_left
rostest x3_target_navigation bypass_real_planner.test scenario:="$scenario" --results-base-dir "$TASK/real_planner_results_$scenario" 2>&1 | tee "$TASK/real_planner_$scenario.log"
sha256sum -c "$TASK/function1_binary.sha256"
