from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

from preview_visual_target import demo_frame
from x3_nav.vision.targets import detect_qr_targets, recommend_option, target_bearing_deg
from x3_nav.bridge.robonix_preview import RobonixPreview


class VisualTargetTests(unittest.TestCase):
    def setUp(self):
        self.frame = demo_frame("X3_TARGET_A")
        self.preview = {"ok": True, "session_id": "test", "options": [
            {"option_id": 1, "radius_m": 0.2, "relative_angle_deg": -20},
            {"option_id": 2, "radius_m": 0.2, "relative_angle_deg": 0},
            {"option_id": 3, "radius_m": 0.2, "relative_angle_deg": 20},
        ]}

    def test_real_opencv_decode_and_left_bearing(self):
        targets = detect_qr_targets(self.frame, "X3_TARGET_A")
        self.assertEqual(len(targets), 1)
        angle = target_bearing_deg(targets[0], 500, 320, 0)
        self.assertGreater(angle, 0)
        result = recommend_option(self.preview, angle)
        self.assertEqual(result["selected_option"]["option_id"], 3)
        self.assertFalse(result["navigation_goal_sent"])

    def test_unknown_and_missing_targets(self):
        self.assertEqual(detect_qr_targets(self.frame, "OTHER"), [])
        self.assertEqual(detect_qr_targets(self.frame * 0, "X3_TARGET_A"), [])

    def test_invalid_or_unsafe_candidates_blocked(self):
        for change in ({"radius_m": 0.8}, {"relative_angle_deg": float("nan")},
                       {"radius_m": -1}, {"option_id": 0}):
            preview = dict(self.preview, options=[dict(self.preview["options"][2], **change)])
            self.assertIsNone(recommend_option(preview, 20)["selected_option"])

    def test_failure_expiry_and_out_of_sector(self):
        self.assertIsNone(recommend_option(dict(self.preview, ok=False), 0)["selected_option"])
        expired = dict(self.preview, expires_epoch=time.time() - 1)
        self.assertEqual(recommend_option(expired, 0)["reason"], "provider_session_expired")
        self.assertIsNone(recommend_option(self.preview, 45)["selected_option"])

    def test_bad_calibration_is_rejected(self):
        target = detect_qr_targets(self.frame, "X3_TARGET_A")[0]
        with self.assertRaises(ValueError):
            target_bearing_deg(target, 0, 320, 0)

    def test_multiple_same_label_targets_are_detected(self):
        import numpy as np
        frame = np.full((480, 900, 3), 255, dtype=np.uint8)
        frame[120:360, 50:290] = self.frame[120:360, 100:340]
        frame[120:360, 600:840] = self.frame[120:360, 100:340]
        self.assertEqual(len(detect_qr_targets(frame, "X3_TARGET_A")), 2)

    def test_robonix_adapter_calls_only_preview(self):
        import json
        preview_call = Mock(return_value=SimpleNamespace(
            result_json=SimpleNamespace(data=json.dumps(self.preview))))
        bridge = SimpleNamespace(ros=SimpleNamespace(terminate=Mock()))
        provider = SimpleNamespace(preview_move_options=preview_call, bridge=bridge)
        modules = {
            "rosmaster_x3_bridge.main": provider,
            "rosmaster_x3_bridge_mcp": SimpleNamespace(PreviewMoveOptions_Request=SimpleNamespace),
        }
        with patch("x3_nav.bridge.robonix_preview.importlib.import_module", side_effect=modules.__getitem__):
            api = RobonixPreview()
            self.assertEqual(api.preview(), self.preview)
            self.assertEqual(preview_call.call_args.args[0].max_options, 5)
            api.close()
        bridge.ros.terminate.assert_called_once()
        self.assertIsNone(provider.bridge)


if __name__ == "__main__":
    unittest.main()
