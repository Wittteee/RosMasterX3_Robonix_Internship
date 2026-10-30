import os
import json
from pathlib import Path
import xml.etree.ElementTree as ET
from nav_calibration_policy import require_calibration
home = Path.home()
root = home / 'rosmaster_x3_deploy/live_test'
profile=require_calibration(json.loads((home/'rosmaster_x3_deploy/scripts/chassis_nav_calibration.json').read_text()))
source = home / 'yahboomcar_ws/src/yahboomcar_nav/launch/library/move_base.launch'
tree = ET.parse(str(source))
node = tree.getroot().find('node')
limits = {'DWAPlannerROS/max_vel_x':profile['navigation_max_linear_mps'], 'DWAPlannerROS/min_vel_x':0.0,
          'DWAPlannerROS/max_vel_y':0.0, 'DWAPlannerROS/min_vel_y':0.0,
          'DWAPlannerROS/max_vel_trans':profile['navigation_max_linear_mps'], 'DWAPlannerROS/min_vel_trans':profile['minimum_stable_linear_mps'],
          'DWAPlannerROS/max_vel_theta':profile['max_angular_rps'], 'DWAPlannerROS/min_vel_theta':0.0,
          'DWAPlannerROS/trans_stopped_vel':0.01, 'DWAPlannerROS/theta_stopped_vel':0.02,
          'DWAPlannerROS/xy_goal_tolerance':profile['xy_goal_tolerance_m'], 'DWAPlannerROS/yaw_goal_tolerance':0.1}
for name, value in limits.items():
    ET.SubElement(node, 'param', name=name, value=str(value))
ET.SubElement(node, 'param', name='recovery_behavior_enabled', value='false')
ET.SubElement(node, 'param', name='clearing_rotation_allowed', value='false')
ET.SubElement(tree.getroot(), 'node', name='map_server', pkg='map_server', type='map_server', args=str(root/'maps/scene_20261028.yaml'))
ET.SubElement(tree.getroot(), 'include', file='$(find yahboomcar_nav)/launch/library/amcl.launch')
ET.SubElement(tree.getroot(), 'param', name='/amcl/update_min_d',value='0.02')
ET.SubElement(tree.getroot(), 'param', name='/amcl/update_min_a',value='0.03')
tree.write(str(root/'demo_navigation.launch'), encoding='utf-8', xml_declaration=True)
