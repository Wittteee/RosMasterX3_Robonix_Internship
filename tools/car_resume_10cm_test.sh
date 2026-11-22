set -e
export LC_ALL=C.UTF-8
export ROBOT_TYPE=X3
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
TASK=/home/jetson/rosmaster_x3_runtime/obstacle2_20261121
TRIAL="$TASK/hardware_10cm_01"
mkdir -p "$TRIAL"
source /opt/ros/melodic/setup.bash
source "$TASK/build_ws/devel/setup.bash"
python3 - <<'PY'
import os,json
root='/home/jetson/rosmaster_x3_runtime/obstacle2_20261121'
old=json.load(open(root+'/hardware_left_01/trial_report.json'))
p=old['samples'][-1]['pose']
assert old['final_permission']=='STOP'
launch='''<launch>
 <include file="$(find x3_target_navigation)/launch/bottle_bypass.launch"><arg name="execute" value="true"/></include>
 <param name="/amcl/initial_pose_x" value="{x}"/>
 <param name="/amcl/initial_pose_y" value="{y}"/>
 <param name="/amcl/initial_pose_a" value="{yaw}"/>
 <param name="/amcl/initial_cov_xx" value="0.0025"/>
 <param name="/amcl/initial_cov_yy" value="0.0025"/>
 <param name="/amcl/initial_cov_aa" value="0.01"/>
 <!-- Trial-only wheel heading; biased IMU still needs calibration. -->
 <rosparam param="/ekf_localization/imu0_config">[false,false,false,false,false,false,false,false,false,false,false,false,false,false,false]</rosparam>
</launch>'''.format(**p)
open(root+'/hardware_10cm_01/trial.launch','w').write(launch)
json.dump({'initial_pose':p,'trial_only_wheel_heading':True,'trigger_front_gap_m':.10},open(root+'/hardware_10cm_01/configuration.json','w'),indent=2)
PY
python "$TASK/build_ws/src/x3_target_navigation/scripts/check_exclusive_devices.py" true true
nohup roslaunch "$TRIAL/trial.launch" > "$TRIAL/runtime.log" 2>&1 < /dev/null &
echo "$!" > "$TRIAL/runtime.pid"
echo '10 cm trial stack started; awaiting independent scene check and confirm'
