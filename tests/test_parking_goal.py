import math
import unittest
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from x3_nav.vision.parking_goal import target_parking_pose,choose_parking_candidate
class ParkingTests(unittest.TestCase):
    def test_rotation_and_standoff(self):
        goal=target_parking_pose({'x':1,'y':2,'yaw_rad':math.pi/2},.6,.3)
        self.assertAlmostEqual(goal['target']['y'],2.6)
        self.assertAlmostEqual(goal['parking']['y'],2.3)
        self.assertAlmostEqual(goal['parking']['x'],1)
    def test_invalid_distances(self):
        for distance,standoff in ((.3,.3),(.2,.3),(2,.1),(float('nan'),.3)):
            with self.assertRaises(ValueError): target_parking_pose({'x':0,'y':0,'yaw_rad':0},distance,standoff)
    def test_only_safe_matching_candidate(self):
        goal=target_parking_pose({'x':0,'y':0,'yaw_rad':0},.6,.3)
        candidate={'x':.3,'y':0,'radius_m':.3,'option_id':1}
        self.assertEqual(choose_parking_candidate({'ok':True,'options':[candidate]},goal),candidate)
        with self.assertRaises(ValueError): choose_parking_candidate({'ok':False,'options':[candidate]},goal)
        with self.assertRaises(ValueError): choose_parking_candidate({'ok':True,'options':[dict(candidate,x=.5)]},goal)
    def test_minimum_step_preserves_original_parking_tolerance(self):
        goal=target_parking_pose({'x':0,'y':0,'yaw_rad':0},.6,.3)
        goal.update(travel_m=.09,search_radius_m=.1)
        candidate={'x':.31,'y':0,'radius_m':.1,'option_id':1}
        self.assertEqual(choose_parking_candidate({'ok':True,'options':[candidate]},goal),candidate)
        with self.assertRaises(ValueError): choose_parking_candidate({'ok':True,'options':[dict(candidate,x=.34)]},goal)
