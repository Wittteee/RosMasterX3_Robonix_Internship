"""Run the production geometry and base interlock without ROS or hardware."""
import runpy
import shutil
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'ros1/x3_target_navigation'


def test_compiled_approach_corridor(tmp_path):
    compiler = shutil.which('g++')
    if not compiler:
        pytest.skip('g++ required to run production C++ geometry')
    binary = tmp_path / 'approach_cases.exe'
    subprocess.run([compiler, '-std=c++14', '-Wall', '-Wextra', '-Werror',
                    '-I', str(PACKAGE / 'include'),
                    str(ROOT / 'tests/approach_obstacle_cases.cpp'),
                    '-o', str(binary)], check=True, capture_output=True, text=True)
    subprocess.run([str(binary)], check=True, capture_output=True, text=True)


class Twist:
    def __init__(self):
        self.linear = SimpleNamespace(x=.1, y=0.)
        self.angular = SimpleNamespace(z=0.)


class Stamp:
    def __init__(self, value):
        self.value = value

    def __sub__(self, other):
        return Stamp(self.value - other.value)

    def is_zero(self):
        return self.value == 0.

    def to_sec(self):
        return self.value


@pytest.fixture
def base():
    ros = Mock()
    modules = {
        'rospy': ros,
        'geometry_msgs.msg': SimpleNamespace(Twist=Twist),
        'std_msgs.msg': SimpleNamespace(Header=object),
        'Rosmaster_Lib': SimpleNamespace(Rosmaster=type('Vendor', (), {})),
        'imp': SimpleNamespace(load_source=lambda *a: SimpleNamespace(yahboomcar_driver=Mock())),
    }
    with patch.dict(sys.modules, modules):
        ns = runpy.run_path(str(PACKAGE / 'scripts/x3_base_driver.py'))
    env = ns['_corrected_callback'].__globals__
    clock = SimpleNamespace(now=10.)
    env['monotonic'] = lambda: clock.now
    ros.Time.now.side_effect = lambda: Stamp(clock.now)
    env['_require_navigation_permission'] = True
    driver = Mock()
    env['_state']['driver'] = driver
    return env, driver, clock


def permission(env, clock, frame='ALLOW', stamp=None):
    env['_permission_callback'](SimpleNamespace(
        frame_id=frame, stamp=Stamp(clock.now if stamp is None else stamp)))


def test_startup_rejects_motion_until_navigation_permission(base):
    env, driver, clock = base
    env['_corrected_callback'](driver, Twist())
    driver.car.set_car_motion.assert_called_with(0., 0., 0.)
    permission(env, clock)
    env['_corrected_callback'](driver, Twist())
    driver.car.set_car_motion.assert_called_with(.1, 0., 0.)


def test_obstacle_stop_overrides_continuing_navigation_commands(base):
    env, driver, clock = base
    permission(env, clock)
    env['_corrected_callback'](driver, Twist())
    permission(env, clock, 'STOP')
    driver.car.set_car_motion.assert_called_with(0., 0., 0.)
    for _ in range(5):
        env['_corrected_callback'](driver, Twist())
        driver.car.set_car_motion.assert_called_with(0., 0., 0.)


def test_permission_watchdog_stops_even_with_recent_velocity_command(base):
    env, driver, clock = base
    permission(env, clock)
    env['_corrected_callback'](driver, Twist())
    clock.now += .3
    env['_corrected_callback'](driver, Twist())
    clock.now += .11
    env['_watchdog'](None)
    driver.car.set_car_motion.assert_called_with(0., 0., 0.)
    env['_corrected_callback'](driver, Twist())
    driver.car.set_car_motion.assert_called_with(0., 0., 0.)


@pytest.mark.parametrize('stamp', [0., 8., 11.])
def test_stale_future_or_zero_permission_never_enables_motion(base, stamp):
    env, driver, clock = base
    permission(env, clock, stamp=stamp)
    env['_corrected_callback'](driver, Twist())
    driver.car.set_car_motion.assert_called_with(0., 0., 0.)


def test_monotonic_clock_reversal_stops(base):
    env, driver, clock = base
    permission(env, clock)
    env['_corrected_callback'](driver, Twist())
    clock.now -= 1.
    env['_watchdog'](None)
    driver.car.set_car_motion.assert_called_with(0., 0., 0.)


def test_queued_permission_keeps_its_original_expiry(base):
    env, driver, clock = base
    permission(env, clock, stamp=clock.now - .3)
    env['_corrected_callback'](driver, Twist())
    driver.car.set_car_motion.assert_called_with(.1, 0., 0.)
    clock.now += .11
    env['_watchdog'](None)
    driver.car.set_car_motion.assert_called_with(0., 0., 0.)


def test_real_car_launch_requires_base_interlock():
    import xml.etree.ElementTree as ET
    launch = ET.parse(PACKAGE / 'launch/target_navigation.launch').getroot()
    bringup = next(node for node in launch.iter('include')
                   if node.get('file', '').endswith('/launch/safe_bringup.launch'))
    assert bringup.find("arg[@name='require_navigation_permission']").get('value') == 'true'
    base_launch = ET.parse(PACKAGE / 'launch/safe_bringup.launch').getroot()
    driver = base_launch.find("node[@name='driver_node']")
    assert driver.find("param[@name='require_navigation_permission']").get('value') == '$(arg require_navigation_permission)'


def test_function_one_has_exclusive_velocity_source_and_five_cm_setting():
    import xml.etree.ElementTree as ET
    import yaml
    launch = ET.parse(PACKAGE / 'launch/target_navigation.launch').getroot()
    assert launch.find("arg[@name='obstacle_stop_approach']").get('default') == 'true'
    includes = list(launch.iter('include'))
    bringup = next(n for n in includes if n.get('file', '').endswith('/safe_bringup.launch'))
    topic = bringup.find("arg[@name='velocity_topic']").get('value')
    assert "'/target_navigation/approach_velocity' if arg('obstacle_stop_approach')" in topic
    navigation = next(n for n in includes if n.get('file', '').endswith('/safe_navigation.launch'))
    assert navigation.find("arg[@name='start_move_base']").get('value') == "$(eval not arg('obstacle_stop_approach') or arg('obstacle_bypass'))"
    assert launch.find("arg[@name='obstacle_bypass']").get('default') == 'false'
    safe_nav = ET.parse(PACKAGE / 'launch/safe_navigation.launch').getroot()
    move_base = next(n for n in safe_nav.iter('include') if n.get('file', '').endswith('/safe_move_base.launch'))
    assert move_base.get('if') == '$(arg start_move_base)'
    config = yaml.safe_load((PACKAGE / 'config/astra_pro.yaml').read_text(encoding='utf-8'))
    assert config['target_task_manager']['obstacle_stop_distance_m'] == .05
    assert config['target_task_manager']['approach_speed_mps'] == .04
