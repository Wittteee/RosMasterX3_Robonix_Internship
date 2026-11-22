set -e
export LC_ALL=C.UTF-8
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
source /opt/ros/melodic/setup.bash
source /home/jetson/yahboomcar_ws/devel/setup.bash
source /home/jetson/software/library_ws/devel/setup.bash
printf 'Execution enabled\n'
rosparam get /target_navigation/target_task_manager/execute_enabled
printf 'Preview confirm rejection\n'
rosservice call /target_navigation/confirm
printf 'Permission\n'
timeout 4 rostopic echo -n 1 /target_navigation/motion_permission
printf 'Odometry\n'
timeout 4 rostopic echo -n 1 /odom/twist/twist
TASK_DIR=/home/jetson/rosmaster_x3_runtime/obstacle1_20261121_5cm
python3 - <<'PY'
import json, os, signal, time, xml.etree.ElementTree as ET, hashlib
root='/home/jetson/rosmaster_x3_runtime/obstacle1_20261121_5cm'
pid=int(open(root+'/observer.pid').read())
try:
    argv=open('/proc/%d/cmdline'%pid,'rb').read().split(b'\0')
    if (root+'/car_observe.py').encode() in argv:
        os.kill(pid, signal.SIGINT)
        time.sleep(1)
except IOError: pass
reports={}
for filename in ['gtest-test_target_core.xml','rosunit-preview_safety.xml','rosunit-obstacle_threshold.xml']:
    path='/home/jetson/yahboomcar_ws/build/test_results/x3_target_navigation/'+filename
    element=ET.parse(path).getroot()
    reports[filename]={'cases':len(list(element.iter('testcase'))),
        'failures':len(list(element.iter('failure'))),'errors':len(list(element.iter('error')))}
report={'date':'2026-11-21','host':'10.206.170.35','deployed':True,'ros_package_build':'passed',
        'ros_tests':reports,'hardware_motion_test':'not_executed_obstacle_not_detected_by_lidar',
        'static_hardware_check':'completed','laser_height_from_base_footprint_m':.1915,
        'stop_command_threshold_m':.05,'final_mode':'execute=false preview only',
        'source_backup':root+'/source_before_deploy.tar.gz',
        'package_source_sha256':hashlib.sha256(open('/home/jetson/yahboomcar_ws/src/x3_target_navigation/src/target_task_manager.cpp','rb').read()).hexdigest(),
        'notes':['Camera sees low tissue obstacle; lidar only returns bottle near x=0.797m.',
                 'Depth image tissue ROI had no valid range; no hardware forward confirmation was issued.',
                 'Isolated rostest confirms 0.051m forward, 0.050m zero, latched after removing obstacle.']}
with open(root+'/validation_report.json','w') as out: json.dump(report,out,indent=2)
print(json.dumps(report,indent=2))
PY
tar -czf "$TASK_DIR/evidence.tar.gz" -C "$TASK_DIR" validation_report.json build.log ros_tests.log runtime.log preview_runtime.log static_lidar_check.json static_depth_check.json observations.jsonl camera_latest.png
