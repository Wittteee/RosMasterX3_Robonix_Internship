#!/usr/bin/env bash
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
python - <<'PY'
import os,json,yaml,rospy,xml.etree.ElementTree as ET
rospy.init_node('readonly_nav_analysis',anonymous=True)
base=os.path.expanduser('~/yahboomcar_ws/src')
for relative in ('yahboomcar_bringup/param/robot_localization.yaml','yahboomcar_nav/param/common/dwa_local_planner_params_X3.yaml'):
 data=yaml.safe_load(open(os.path.join(base,relative)))
 print(relative)
 if 'DWAPlannerROS' in data: print(json.dumps(data['DWAPlannerROS']))
 else: print(json.dumps({k:v for k,v in data.items() if k.startswith(('odom','imu','twist','use_control'))}))
tree=ET.fromstring(rospy.get_param('/robot_description'))
for joint in tree.findall('joint'):
 if 'imu' in joint.get('name',''):
  print(ET.tostring(joint).decode('utf-8'))
PY
