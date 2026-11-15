"""Exercise the deployed callback without ROS, serial ports or physical motion."""
import json
import math
import runpy
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

ROOT = Path(__file__).resolve().parents[1]


class Twist:
    def __init__(self):
        self.linear = SimpleNamespace(x=0., y=0., z=0.)
        self.angular = SimpleNamespace(x=0., y=0., z=0.)


class Stamp:
    def __init__(self, value):
        self.value = value

    def __sub__(self, other):
        return Stamp(self.value - other.value)

    def is_zero(self):
        return self.value == 0

    def to_sec(self):
        return self.value


def load_driver(overrides=None):
    ros = Mock()
    params = {'~calibration_profile': str(ROOT / 'configs/chassis_nav_calibration.json')}
    params.update(overrides or {})
    ros.get_param.side_effect = lambda key, default=None: params.get(key, default)
    driver = Mock(car=Mock(ser=SimpleNamespace()))
    driver_class = Mock(return_value=driver)
    modules = {
        'rospy': ros,
        'geometry_msgs': SimpleNamespace(),
        'geometry_msgs.msg': SimpleNamespace(Twist=Twist),
        'nav_msgs': SimpleNamespace(),
        'nav_msgs.msg': SimpleNamespace(Odometry=object),
        'Rosmaster_Lib': SimpleNamespace(Rosmaster=type('Vendor', (), {})),
        'imp': SimpleNamespace(load_source=lambda *args: SimpleNamespace(yahboomcar_driver=driver_class)),
    }
    with patch.object(sys, 'path', [str(ROOT / 'scripts/jetson')] + sys.path), patch.dict(sys.modules, modules):
        namespace = runpy.run_path(str(ROOT / 'scripts/jetson/pinned_chassis_driver.py'))
    driver.car.set_car_motion.reset_mock()
    return namespace, driver, ros


def test_callback_preserves_metric_speed_and_publishes_applied_command():
    ns, driver, ros = load_driver()
    msg = Twist()
    msg.linear.x, msg.angular.z = .09, .15
    ns['corrected_callback'](driver, msg)
    driver.car.set_car_motion.assert_called_once_with(.09, 0., .15)
    applied = ros.Publisher.return_value.publish.call_args.args[0]
    assert (applied.linear.x, applied.angular.z) == (.09, .15)
    ros.Subscriber.assert_not_called()


def test_callback_reports_limit_and_stops_on_bad_command():
    ns, driver, ros = load_driver({'~command_max_linear_mps': .08})
    msg = Twist()
    msg.linear.x = .09
    ns['corrected_callback'](driver, msg)
    driver.car.set_car_motion.assert_called_with(.08, 0., 0.)
    ros.logwarn_throttle.assert_called_once()
    msg.linear.x = float('nan')
    ns['corrected_callback'](driver, msg)
    driver.car.set_car_motion.assert_called_with(0., 0., 0.)


def test_excessive_override_refused():
    with pytest.raises(ValueError, match='range'):
        load_driver({'~command_max_linear_mps': .2})


def test_heading_hold_requires_verified_sign():
    with pytest.raises(ValueError, match='heading sign'):
        load_driver({'~straight_heading_hold': True})


@pytest.fixture
def heading_driver(tmp_path):
    profile = json.loads((ROOT / 'configs/chassis_nav_calibration.json').read_text())
    profile['heading_sign_verified'] = True
    path = tmp_path / 'profile.json'
    path.write_text(json.dumps(profile))
    ns, driver, ros = load_driver({'~calibration_profile': str(path),
                                  '~straight_heading_hold': True,
                                  '~command_max_angular_rps': .05})
    clock = SimpleNamespace(now=10.)
    ns['corrected_callback'].__globals__['monotonic'] = lambda: clock.now
    ros.Time.now.side_effect = lambda: Stamp(clock.now)
    return ns, driver, clock


def odom(yaw, stamp, frame='odom'):
    q = SimpleNamespace(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2))
    return SimpleNamespace(header=SimpleNamespace(frame_id=frame, stamp=Stamp(stamp)),
                           child_frame_id='base_footprint',
                           pose=SimpleNamespace(pose=SimpleNamespace(orientation=q)))


def test_heading_hold_corrects_shortest_error_and_resets_after_manual_turn(heading_driver):
    ns, driver, clock = heading_driver
    cmd = Twist()
    cmd.linear.x = .09
    ns['update_heading'](odom(math.pi-.02, clock.now))
    ns['corrected_callback'](driver, cmd)
    driver.car.set_car_motion.assert_called_with(.09, 0., 0.)
    ns['update_heading'](odom(-math.pi+.03, clock.now))
    ns['corrected_callback'](driver, cmd)
    assert driver.car.set_car_motion.call_args.args == pytest.approx((.09, 0., -.04))
    ns['update_heading'](odom(-math.pi+.2, clock.now))
    ns['corrected_callback'](driver, cmd)
    driver.car.set_car_motion.assert_called_with(.09, 0., -.05)
    cmd.angular.z = .04
    ns['corrected_callback'](driver, cmd)
    driver.car.set_car_motion.assert_called_with(.09, 0., .04)
    cmd.angular.z = 0.
    ns['corrected_callback'](driver, cmd)
    driver.car.set_car_motion.assert_called_with(.09, 0., 0.)


def test_heading_hold_stops_with_stale_or_wrong_frame_feedback(heading_driver):
    ns, driver, clock = heading_driver
    cmd = Twist()
    cmd.linear.x = .09
    ns['update_heading'](odom(0., clock.now, frame='map'))
    ns['corrected_callback'](driver, cmd)
    driver.car.set_car_motion.assert_called_with(0., 0., 0.)
    ns['update_heading'](odom(0., clock.now-.3))
    ns['corrected_callback'](driver, cmd)
    driver.car.set_car_motion.assert_called_with(0., 0., 0.)
    ns['update_heading'](odom(0., clock.now))
    ns['corrected_callback'](driver, cmd)
    driver.car.set_car_motion.assert_called_with(.09, 0., 0.)
    clock.now += .26
    ns['watchdog'](None)
    driver.car.set_car_motion.assert_called_with(0., 0., 0.)
