"""Adapter for safe_move eabaec3e75463489481d98658f8918b82afb7c36."""
from contextlib import contextmanager
import json
import os
import math
import threading

from x3_nav.vision.robonix_preview import RobonixPreview


@contextmanager
def prepared_execution_gate():
    values = {"X3_ALLOW_PREPARED_NAV_EXECUTION": "1",
              "X3_PREPARED_NAV_EXECUTION_BUDGET_SEC": "20",
              "X3_PREPARED_NAV_RESULT_TIMEOUT_SEC": "45",
              "X3_ALLOW_NAV_GOAL": None, "X3_MAX_NAV_GOAL_DISTANCE_M": None}
    previous = {name: os.environ.get(name) for name in values}
    try:
        for name, value in values.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value
        yield
    finally:
        for name, value in previous.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value


class RobonixNavigation(RobonixPreview):
    def __init__(self, provider_dir=None, *, max_radius=0.2):
        if not math.isfinite(max_radius) or not 0.1 <= max_radius <= 0.8:
            raise ValueError("Provider radius must be in [0.1, 0.8] metres")
        self.max_radius = max_radius
        super().__init__(provider_dir)

    def preview(self):
        # Ask the provider to PLAN within the requested radius, rather than
        # rejecting all of its default 0.8 m candidates in an open room.
        values = {"X3_MOVE_OPTIONS_MIN_RADIUS_M": "0.1",
                  "X3_MOVE_OPTIONS_MAX_RADIUS_M": str(self.max_radius)}
        previous = {key: os.environ.get(key) for key in values}
        try:
            os.environ.update(values)
            return super().preview()
        finally:
            for key, value in previous.items():
                if value is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value

    def health(self):
        """Require actual positive MCU telemetry, not driver's default zeros."""
        import roslibpy
        bridge = self.provider.ensure_bridge()
        values = {}
        done = threading.Event()
        lock = threading.Lock()
        subscriptions = []

        def receive(name, message):
            try:
                value = float(message["data"])
            except (KeyError, ValueError, TypeError):
                return
            with lock:
                values[name] = value if math.isfinite(value) else None
                if len(values) == 2:
                    done.set()

        try:
            for name in ("/edition", "/voltage"):
                topic = roslibpy.Topic(bridge.ros, name, "std_msgs/Float32")
                subscriptions.append(topic)
                topic.subscribe(lambda message, name=name: receive(name, message))
            done.wait(10)
            with lock:
                report = dict(values)
            return {"ok": len(report) == 2 and all(v is not None and v > 0 for v in report.values()),
                    "telemetry": report}
        finally:
            for topic in subscriptions:
                topic.unsubscribe()

    def _call(self, function, request, **kwargs):
        response = getattr(self.provider, function)(getattr(self.bindings, request)(**kwargs))
        payload = json.loads(response.result_json.data)
        if not isinstance(payload, dict):
            raise ValueError("Invalid Provider response")
        return payload

    def prepare(self, session_id, option_id):
        return self._call("prepare_selected_move", "PrepareSelectedMove_Request",
                          session_id=session_id, option_id=option_id)

    def execute(self, token, confirmation_text):
        with prepared_execution_gate():
            return self._call("execute_prepared_navigation", "ExecutePreparedNavigation_Request",
                              token=token, allow_execute=True, confirmation_text=confirmation_text)

    def cancel(self):
        return self._call("cancel_nav_goal", "CancelNavGoal_Request")
