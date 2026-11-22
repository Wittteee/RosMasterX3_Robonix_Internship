from pathlib import Path
import ast
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'ros1/x3_target_navigation'


def test_production_bypass_geometry(tmp_path):
    compiler = shutil.which('g++')
    if not compiler:
        pytest.skip('g++ required for production geometry')
    binary = tmp_path / 'bypass_cases.exe'
    subprocess.run([compiler,'-std=c++14','-Wall','-Wextra','-Werror',
                    '-I',str(PACKAGE/'include'),str(ROOT/'tests/bypass_geometry_cases.cpp'),
                    '-o',str(binary)],check=True,capture_output=True,text=True)
    subprocess.run([str(binary)],check=True,capture_output=True,text=True)


def test_bypass_mode_is_opt_in_and_uses_the_same_base_interlock():
    launch = ET.parse(PACKAGE/'launch/target_navigation.launch').getroot()
    assert launch.find("arg[@name='obstacle_bypass']").get('default') == 'false'
    bypass = ET.parse(PACKAGE/'launch/bottle_bypass.launch').getroot()
    assert bypass.find("arg[@name='execute']").get('default') == 'false'
    args = {a.get('name'):a.get('value') for a in bypass.find('include').findall('arg')}
    assert args['obstacle_bypass'] == 'true'
    assert args['obstacle_stop_approach'] == 'true'
    assert args['start_navigation'] == 'true'
    move_base = ET.parse(PACKAGE/'launch/safe_move_base.launch').getroot()
    load = next(n for n in move_base.iter('rosparam') if n.get('file','').endswith('/bypass_move_base.yaml'))
    assert load.get('if') == '$(arg bypass_holonomic)'
    remap = next(n for n in move_base.iter('remap') if n.get('from') == 'cmd_vel')
    assert '/target_navigation/bypass_nav_velocity' in remap.get('to')
    assert 'else' in remap.get('to')


def test_bypass_planner_keeps_unknown_space_blocked_and_enables_lateral_samples():
    config = yaml.safe_load((PACKAGE/'config/bypass_move_base.yaml').read_text())
    assert config['NavfnROS']['allow_unknown'] is False
    assert config['GlobalPlanner']['allow_unknown'] is False
    assert config['base_global_planner'] == 'global_planner/GlobalPlanner'
    assert config['GlobalPlanner']['old_navfn_behavior'] is False
    assert config['local_costmap']['resolution'] <= .02
    for name in ('local_costmap','global_costmap'):
        assert config[name]['track_unknown_space'] is True
        assert config[name]['footprint_padding'] >= .02
        assert config[name]['obstacle_layer']['scan']['marking'] is True
        assert config[name]['obstacle_layer']['scan']['expected_update_rate'] > 0
    dwa = config['DWAPlannerROS']
    assert dwa['min_vel_y'] < 0 < dwa['max_vel_y']
    assert dwa['vy_samples'] >= 3
    assert dwa['max_vel_trans'] <= .04
    assert dwa['forward_point_distance'] == 0.0
    assert dwa['sim_time'] >= 2.0
    assert dwa['sim_granularity'] <= .005
    assert config['recovery_behavior_enabled'] is False


def test_ros_behavior_test_syntax():
    ast.parse((PACKAGE/'test/test_bypass_behavior.py').read_text(encoding='utf-8'))


def test_live_bypass_uses_thirty_cm_front_gap():
    config = yaml.safe_load((PACKAGE/'config/astra_pro.yaml').read_text())
    assert config['target_task_manager']['bypass_detection_distance_m'] == .30
    assert config['target_task_manager']['obstacle_stop_distance_m'] == .05
    launch = ET.parse(PACKAGE/'test/bypass_behavior.test').getroot()
    param = next(p for p in launch.iter('param') if p.get('name') == 'bypass_detection_distance_m')
    assert float(param.get('value')) == .30


def test_legacy_explicit_params_cannot_override_bypass_profile():
    launch = ET.parse(PACKAGE/'launch/safe_move_base.launch').getroot()
    for name in ('DWAPlannerROS/min_vel_y','DWAPlannerROS/max_vel_y',
                 'DWAPlannerROS/max_vel_x','DWAPlannerROS/max_vel_trans',
                 'DWAPlannerROS/max_vel_theta','DWAPlannerROS/xy_goal_tolerance',
                 'local_costmap/inflation_layer/inflation_radius'):
        param = next(n for n in launch.iter('param') if n.get('name') == name)
        assert param.get('unless') == '$(arg bypass_holonomic)'
