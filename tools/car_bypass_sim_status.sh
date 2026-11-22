set -e
export LC_ALL=C.UTF-8
TASK=/home/jetson/rosmaster_x3_runtime/obstacle2_20261121
source /opt/ros/melodic/setup.bash
source "$TASK/build_ws/devel/setup.bash"
SIM_URI=$(python3 - <<'PY'
import os
prefix=b'/home/jetson/rosmaster_x3_runtime/obstacle2_20261121/build_ws/devel/lib/x3_target_navigation/target_task_manager'
for entry in os.listdir('/proc'):
    if not entry.isdigit():continue
    try:
        args=open('/proc/'+entry+'/cmdline','rb').read().split(b'\0')
        if args[0]!=prefix:continue
        variables=dict(item.split(b'=',1) for item in open('/proc/'+entry+'/environ','rb').read().split(b'\0') if b'=' in item)
        uri=variables.get(b'ROS_MASTER_URI',b'').decode()
        if ':11311' not in uri: print(uri);break
    except IOError:pass
PY
)
if [ -n "$SIM_URI" ]; then
  export ROS_MASTER_URI="$SIM_URI"
  printf 'Isolated simulation %s\n' "$SIM_URI"
  timeout 3 rostopic echo -n 1 /target_navigation/state || true
  timeout 3 rostopic echo -n 1 /target_navigation/approach_velocity || true
  timeout 3 rostopic echo -n 1 /amcl_pose/pose/pose || true
fi
tail -n 20 "$TASK/real_planner_test.log"
