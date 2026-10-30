import json
import os
from pathlib import Path
import sys
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

from run_visual_nav import SimulationProvider
from x3_nav.mission.visual import VisualNavigation
from x3_nav.bridge.robonix_navigation import RobonixNavigation, prepared_execution_gate
from x3_nav.vision.targets import recommend_option


class NavigationTests(unittest.TestCase):
    def setUp(self):
        self.api = Mock(wraps=SimulationProvider())
        self.observe = Mock(return_value=20)
        self.mission = VisualNavigation(self.api, self.observe)

    def run_confirmed(self):
        return self.mission.run(execute=True, confirm=lambda _: "确认执行")

    def test_preview_never_executes(self):
        result = self.mission.run()
        self.assertTrue(result["ok"])
        self.api.execute.assert_not_called()
        self.api.prepare.assert_called_once_with("simulation", 3)

    def test_invalid_chassis_blocks_preview(self):
        self.api.health.return_value = {"ok": False, "telemetry": {"/edition": -1, "/voltage": 0}}
        self.assertFalse(self.run_confirmed()["ok"])
        self.api.preview.assert_not_called()
        self.api.execute.assert_not_called()

    def test_chassis_failure_after_confirmation(self):
        self.api.health.side_effect = [{"ok": True}, {"ok": False}]
        self.assertFalse(self.run_confirmed()["ok"])
        self.api.execute.assert_not_called()

    def test_success_and_no_token_replay(self):
        self.assertTrue(self.run_confirmed()["arrival_verified"])
        self.assertEqual(self.observe.call_count, 3)
        self.api.execute.assert_called_once_with("simulation-token", "simulation")
        with self.assertRaises(RuntimeError):
            self.run_confirmed()

    def test_declined(self):
        self.assertFalse(self.mission.run(execute=True, confirm=lambda _: "yes")["ok"])
        self.api.execute.assert_not_called()

    def test_lost_after_confirmation(self):
        self.observe.side_effect = [20, 20, ValueError("target_not_found")]
        self.assertFalse(self.run_confirmed()["ok"])
        self.api.execute.assert_not_called()

    def test_changed_direction(self):
        self.observe.side_effect = [20, 20, -20]
        self.assertFalse(self.run_confirmed()["ok"])
        self.api.execute.assert_not_called()

    def test_preview_and_prepare_rejection(self):
        self.api.preview.return_value = {"ok": False, "reason": "nav_not_ready"}
        self.assertFalse(self.run_confirmed()["ok"])
        self.api.prepare.assert_not_called()
        self.api.execute.assert_not_called()

    def test_prepare_rejection(self):
        self.api.prepare.return_value = {"ok": False, "reason": "nav_not_ready"}
        self.assertFalse(self.run_confirmed()["ok"])
        self.api.execute.assert_not_called()

    def test_expired_token(self):
        prepared = SimulationProvider().prepare("simulation", 3)
        prepared["expires_at_epoch"] = time.time() - 1
        self.api.prepare.return_value = prepared
        self.assertFalse(self.run_confirmed()["ok"])
        self.api.execute.assert_not_called()

    def test_uncertain_outcome_requests_cancel(self):
        self.api.execute.return_value = {"ok": True, "reason": "navigation_result_timeout"}
        self.assertFalse(self.run_confirmed()["ok"])
        self.api.cancel.assert_called_once()
        self.assertFalse(self.mission.events[-1]["stop_verified"])

    def test_exception_and_interrupt_request_cancel(self):
        for exc in (RuntimeError("transport lost"), KeyboardInterrupt()):
            with self.subTest(exc=type(exc)):
                self.setUp()
                self.api.execute.side_effect = exc
                self.assertFalse(self.run_confirmed()["ok"])
                self.api.cancel.assert_called_once()

    def test_cancel_failure_is_visible(self):
        self.api.execute.side_effect = RuntimeError("lost")
        self.api.cancel.side_effect = RuntimeError("disconnected")
        self.assertFalse(self.run_confirmed()["ok"])
        self.assertEqual(self.mission.events[-1]["phase"], "CANCEL_FAILED")

    def test_event_log_does_not_contain_token(self):
        self.run_confirmed()
        self.assertNotIn("simulation-token", json.dumps(self.mission.events))

    def test_malformed_candidates(self):
        for options in (None, [None], [{"option_id": 1.5, "radius_m": .2, "relative_angle_deg": 0}],
                        [{"option_id": True, "radius_m": .2, "relative_angle_deg": 0}]):
            self.assertIsNone(recommend_option({"ok": True, "session_id": "s", "options": options}, 0)["selected_option"])

    def test_gate_restores_environment_on_exception(self):
        with patch.dict(os.environ, {"X3_ALLOW_NAV_GOAL": "existing"}):
            before = dict(os.environ)
            with self.assertRaises(RuntimeError):
                with prepared_execution_gate():
                    self.assertNotIn("X3_ALLOW_NAV_GOAL", os.environ)
                    self.assertEqual(os.environ["X3_ALLOW_PREPARED_NAV_EXECUTION"], "1")
                    raise RuntimeError("failed")
            self.assertEqual(dict(os.environ), before)

    def test_adapter_contract(self):
        response = SimpleNamespace(result_json=SimpleNamespace(data='{"ok": true}'))
        provider = SimpleNamespace(prepare_selected_move=Mock(return_value=response),
                                   execute_prepared_navigation=Mock(return_value=response),
                                   cancel_nav_goal=Mock(return_value=response))
        bindings = SimpleNamespace(PrepareSelectedMove_Request=SimpleNamespace,
                                   ExecutePreparedNavigation_Request=SimpleNamespace,
                                   CancelNavGoal_Request=SimpleNamespace)
        modules = {"rosmaster_x3_bridge.main": provider, "rosmaster_x3_bridge_mcp": bindings}
        with patch("x3_nav.bridge.robonix_preview.importlib.import_module", side_effect=modules.__getitem__):
            api = RobonixNavigation()
            api.prepare("s", 3)
            api.execute("token", "phrase")
            api.cancel()
        request = provider.prepare_selected_move.call_args.args[0]
        self.assertEqual(vars(request), {"session_id": "s", "option_id": 3})
        request = provider.execute_prepared_navigation.call_args.args[0]
        self.assertEqual(vars(request), {"token": "token", "allow_execute": True, "confirmation_text": "phrase"})

    def test_provider_preview_uses_requested_radius_and_restores_environment(self):
        api = RobonixNavigation.__new__(RobonixNavigation)
        api.max_radius = 0.2
        api.bindings = SimpleNamespace(PreviewMoveOptions_Request=SimpleNamespace)
        def preview(request):
            self.assertEqual(request.max_options, 5)
            self.assertEqual(os.environ["X3_MOVE_OPTIONS_MAX_RADIUS_M"], "0.2")
            return SimpleNamespace(result_json=SimpleNamespace(data='{"ok": false}'))
        api.provider = SimpleNamespace(preview_move_options=preview)
        before = dict(os.environ)
        self.assertFalse(api.preview()["ok"])
        self.assertEqual(dict(os.environ), before)

    def test_chassis_adapter_validates_real_messages_and_unsubscribes(self):
        for edition, voltage, expected in ((1.2, 12, True), (-1, 0, False),
                                           (float("nan"), 12, False)):
            with self.subTest(edition=edition):
                topics = []
                def topic_factory(ros, name, message_type):
                    self.assertEqual(message_type, "std_msgs/Float32")
                    topic = Mock()
                    value = edition if name == "/edition" else voltage
                    topic.subscribe.side_effect = lambda callback: callback({"data": value})
                    topics.append(topic)
                    return topic
                api = RobonixNavigation.__new__(RobonixNavigation)
                api.provider = SimpleNamespace(ensure_bridge=lambda: SimpleNamespace(ros=object()))
                with patch("roslibpy.Topic", side_effect=topic_factory):
                    report = api.health()
                self.assertEqual(report["ok"], expected)
                for topic in topics:
                    topic.unsubscribe.assert_called_once()

    def test_legacy_live_injection_is_blocked(self):
        from x3_nav.config import load_config
        from x3_nav.vision.detect import Detection
        from x3_nav.vision.pipeline import DetectNavPipeline
        with self.assertRaises(RuntimeError):
            DetectNavPipeline(load_config()).run_live(
                Detection("person", .9, 0), human_confirm=True, dispatch=True)


if __name__ == "__main__":
    unittest.main()
