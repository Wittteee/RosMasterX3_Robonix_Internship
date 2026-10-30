import json
import tempfile
import unittest
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from x3_nav.vision.camera_observation import centered_qr_bearing,StableQrTrigger

class CameraObservationTests(unittest.TestCase):
    def check(self,data):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'observation.json'
            path.write_text(json.dumps(data))
            return centered_qr_bearing(path,now=100)
    def data(self):
        return {'timestamp':100,'source':'real_camera','frame_ok':True,
                'targets':[{'label':'X3_TARGET_A','center_x':320,'image_width':640}]}
    def test_centered_trigger(self):
        self.assertEqual(self.check(self.data()),0)
    def test_stale_and_future_rejected(self):
        for stamp in (98,101,float('nan')):
            data=self.data();data['timestamp']=stamp
            with self.assertRaises(ValueError):self.check(data)
    def test_duplicate_missing_and_offcenter_rejected(self):
        for targets in ([],self.data()['targets']*2,[{'label':'X3_TARGET_A','center_x':100,'image_width':640}]):
            data=self.data();data['targets']=targets
            with self.assertRaises(ValueError):self.check(data)
    def test_non_camera_rejected(self):
        data=self.data();data['source']='simulation'
        with self.assertRaises(ValueError):self.check(data)
    def test_three_distinct_frames_trigger_only_once(self):
        gate=StableQrTrigger(3)
        data=self.data()
        self.assertFalse(gate.update(data,now=100))
        self.assertFalse(gate.update(data,now=100))
        data['timestamp']=100.2
        self.assertFalse(gate.update(data,now=100.2))
        data['timestamp']=100.4
        self.assertTrue(gate.update(data,now=100.4))
        data['timestamp']=100.6
        self.assertFalse(gate.update(data,now=100.6))
    def test_lost_target_resets_stability(self):
        gate=StableQrTrigger(3)
        self.assertFalse(gate.update(self.data(),now=100))
        bad=self.data();bad['targets']=[]
        self.assertFalse(gate.update(bad,now=100))
        data=self.data();data['timestamp']=100.2
        self.assertFalse(gate.update(data,now=100.2))
        self.assertEqual(gate.count,1)
