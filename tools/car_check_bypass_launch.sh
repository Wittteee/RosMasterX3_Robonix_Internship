set -e
export LC_ALL=C.UTF-8
export ROBOT_TYPE=X3
TASK=/home/jetson/rosmaster_x3_runtime/obstacle2_20261121
source /opt/ros/melodic/setup.bash
source "$TASK/build_ws/devel/setup.bash"
roslaunch x3_target_navigation target_navigation.launch --dump-params obstacle_bypass:=true start_bringup:=false start_astra:=false start_navigation:=true execute:=false map:=final_site_20261107 > "$TASK/launch_params.yaml"
python - <<'PY'
import yaml
data=yaml.safe_load(open('/home/jetson/rosmaster_x3_runtime/obstacle2_20261121/launch_params.yaml'))
assert data['/target_navigation/target_task_manager/obstacle_bypass_enabled'] is True
assert data['/target_navigation/target_task_manager/bypass_detection_distance_m'] == .05
assert data['/target_navigation/target_task_manager/execute_enabled'] is False
assert data['/move_base/DWAPlannerROS/max_vel_y']==.03
assert data['/move_base/DWAPlannerROS/min_vel_y']==-.03
assert data['/move_base/DWAPlannerROS/vy_samples']==9
assert data['/move_base/NavfnROS/allow_unknown'] is False
assert data['/move_base/base_global_planner']=='global_planner/GlobalPlanner'
assert data['/move_base/GlobalPlanner/allow_unknown'] is False
assert data['/move_base/local_costmap/resolution']==.02
assert data['/move_base/local_costmap/footprint'][0]==[-.117,-.1]
print('Expanded ROS launch parameters verified; no hardware nodes launched')
PY
sha256sum -c "$TASK/function1_binary.sha256"
