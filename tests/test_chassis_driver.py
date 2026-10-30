"""Exercise the deployed callback without ROS, serial ports or physical motion."""
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
