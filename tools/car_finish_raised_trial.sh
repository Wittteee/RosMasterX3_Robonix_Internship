set -e
export LC_ALL=C.UTF-8
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
source /opt/ros/melodic/setup.bash
source /home/jetson/yahboomcar_ws/devel/setup.bash
source /home/jetson/software/library_ws/devel/setup.bash
TRIAL=/home/jetson/rosmaster_x3_runtime/obstacle1_20261121_raised_trial
printf 'Latched state\n'
timeout 5 rostopic echo -n 1 /target_navigation/state
printf 'Repeat confirm must be rejected\n'
rosservice call /target_navigation/confirm
printf 'Motion permission\n'
timeout 5 rostopic echo -n 1 /target_navigation/motion_permission
printf 'Final odometry\n'
timeout 5 rostopic echo -n 1 /odom/twist/twist
cp /home/jetson/rosmaster_x3_runtime/obstacle1_20261121_5cm/camera_latest.png "$TRIAL/after.png"
printf 'Controller stop trigger\n'
grep 'Obstacle stop commanded' "$TRIAL/runtime.log" || true
python3 - <<'PY'
import json,hashlib
path='/home/jetson/rosmaster_x3_runtime/obstacle1_20261121_raised_trial'
result=json.load(open(path+'/trial_report.json'))
summary={'hardware_trial':'passed' if result.get('confirm_success') and result.get('held_stopped') and result.get('terminal_state')=='FAILED: BLOCKED_NO_SAFE_PATH' and not result.get('supervisor_cancel') else 'not_passed',
         'stop_command_front_gap_m':result.get('first_zero',{}).get('gap'),
         'final_front_gap_m':result.get('final_gap_m'),'held_stopped':result.get('held_stopped'),
         'initial_front_gap_m':result.get('initial_gap_m'),'odom_travel_m':result.get('odom_travel_m'),
         'terminal_state':result.get('terminal_state'), 'final_applied':result.get('final_applied'),
         'distance_note':'Distances are lidar/TF estimates using configured front offset, not ruler measurements.',
         'test_scope':'One real approach to an obstacle already present; removal-after-stop and insertion-during-motion were not physically tested.',
         'deployed_header_sha256':hashlib.sha256(open('/home/jetson/yahboomcar_ws/src/x3_target_navigation/include/x3_target_navigation/approach_obstacle.hpp','rb').read()).hexdigest()}
with open(path+'/validation_summary.json','w') as output:json.dump(summary,output,indent=2)
print(json.dumps(summary,indent=2))
PY
tar -czf "$TRIAL/trial_evidence.tar.gz" -C "$TRIAL" validation_summary.json trial_report.json trial_samples.jsonl trial.log runtime.log before.png after.png readiness.json
