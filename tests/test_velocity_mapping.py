import importlib.util
from pathlib import Path
import unittest
spec=importlib.util.spec_from_file_location('velocity_mapping',Path(__file__).resolve().parents[1]/'scripts/jetson/velocity_mapping.py')
mapping=importlib.util.module_from_spec(spec);spec.loader.exec_module(mapping)
policy_spec=importlib.util.spec_from_file_location('nav_calibration_policy',Path(__file__).resolve().parents[1]/'scripts/jetson/nav_calibration_policy.py')
policy=importlib.util.module_from_spec(policy_spec);policy_spec.loader.exec_module(policy)

class VelocityMappingTests(unittest.TestCase):
    def test_metric_commands_are_not_attenuated(self):
        self.assertEqual(mapping.map_velocity(.09,-.01,.15),(.09,-.01,.15))
    def test_zero_and_signed_rotation(self):
        self.assertEqual(mapping.map_velocity(0,0,0),(0,0,0))
        self.assertEqual(mapping.map_velocity(0,0,-.1),(0,0,-.1))
    def test_vector_limit_preserves_direction(self):
        x,y,w=mapping.map_velocity(.3,.4,.5)
        self.assertAlmostEqual((x*x+y*y)**.5,.1)
        self.assertAlmostEqual(x/y,.75)
        self.assertEqual(w,.2)
    def test_invalid_inputs_fail_closed(self):
        for bad in (float('nan'),float('inf')):
            with self.assertRaises(ValueError): mapping.map_velocity(bad,0,0)
        with self.assertRaises(ValueError): mapping.map_velocity(0,0,0,max_linear=-1)
    def test_watchdog_timing(self):
        self.assertFalse(mapping.command_expired(None,1))
        self.assertFalse(mapping.command_expired(1,1.2))
        self.assertTrue(mapping.command_expired(1,1.5))
        self.assertTrue(mapping.command_expired(1,.5))
    def test_unmeasured_calibration_blocks_navigation(self):
        with self.assertRaises(ValueError):policy.require_calibration({'status':'pending_physical_measurement'})
    def test_invalid_speed_and_precision_calibration(self):
        p={'status':'verified','heading_sign_verified':True,'odometry_scale_verified':True,
           'minimum_stable_linear_mps':.05,'navigation_max_linear_mps':.09,
           'measured_position_error_m':.03,'xy_goal_tolerance_m':.05}
        self.assertEqual(policy.require_calibration(p)['minimum_stable_linear_mps'],.05)
        for invalid in ({'minimum_stable_linear_mps':.12},{'xy_goal_tolerance_m':.02},{'heading_sign_verified':False}):
            with self.assertRaises(ValueError): policy.require_calibration(dict(p,**invalid))
