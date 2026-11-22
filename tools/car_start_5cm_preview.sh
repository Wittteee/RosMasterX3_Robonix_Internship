set -e
export LC_ALL=C.UTF-8
export ROBOT_TYPE=X3
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
TASK=/home/jetson/rosmaster_x3_runtime/obstacle2_20261121
PREVIEW="$TASK/hardware_5cm_preview"
mkdir -p "$PREVIEW"
source /opt/ros/melodic/setup.bash
source "$TASK/build_ws/devel/setup.bash"
python3 - <<'PY'
import json,hashlib,yaml
root='/home/jetson/rosmaster_x3_runtime/obstacle2_20261121'
binary=root+'/build_ws/devel/lib/x3_target_navigation/target_task_manager'
assert hashlib.sha256(open(binary,'rb').read()).hexdigest()=='b02450c4c15917d312ed4f83caa6cc6040eddbf11736e271a9ef90c58c8ae333'
config=yaml.safe_load(open(root+'/build_ws/src/x3_target_navigation/config/astra_pro.yaml'))
assert config['target_task_manager']['bypass_detection_distance_m']==.05
last=json.load(open(root+'/hardware_10cm_01/trial_report.json'))
assert last['final_permission']=='STOP'
p=last['samples'][-1]['pose']
launch='''<launch>
 <include file="$(find x3_target_navigation)/launch/bottle_bypass.launch">
  <arg name="execute" value="false"/>
 </include>
 <param name="/amcl/initial_pose_x" value="{x}"/>
 <param name="/amcl/initial_pose_y" value="{y}"/>
 <param name="/amcl/initial_pose_a" value="{yaw}"/>
 <param name="/amcl/initial_cov_xx" value="0.0025"/>
 <param name="/amcl/initial_cov_yy" value="0.0025"/>
 <param name="/amcl/initial_cov_aa" value="0.01"/>
 <!-- Retain the documented trial-only wheel heading until IMU calibration. -->
 <rosparam param="/ekf_localization/imu0_config">[false,false,false,false,false,false,false,false,false,false,false,false,false,false,false]</rosparam>
</launch>'''.format(**p)
open(root+'/hardware_5cm_preview/preview.launch','w').write(launch)
json.dump({'execute':False,'front_trigger_m':.05,'wheel_heading_trial':True,'initial_pose_seed':p},open(root+'/hardware_5cm_preview/configuration.json','w'),indent=2)
print('Verified latest 5 cm binary; execute=false')
PY
python "$TASK/build_ws/src/x3_target_navigation/scripts/check_exclusive_devices.py" true true
nohup roslaunch "$PREVIEW/preview.launch" > "$PREVIEW/runtime.log" 2>&1 < /dev/null &
echo "$!" > "$PREVIEW/runtime.pid"
echo "5 cm preview started, PID $!; no driving confirmation issued"
