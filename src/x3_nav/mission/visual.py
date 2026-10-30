"""One QR observation -> official candidate -> single-use prepared execution.

The provider owns localization, planning, token validation and motion monitoring.
No coordinates or velocity commands are sent by this module.
"""
from __future__ import annotations

import math
import time
from typing import Callable

from x3_nav.vision.targets import recommend_option
from x3_nav.safety import CandidateGoal, SafetyController, Phase


class VisualNavigation:
    def __init__(self, api, observe: Callable[[], float], *, max_radius=0.2):
        self.api = api
        self.observe = observe
        self.max_radius = max_radius
        self.events = []
        self.used = False
        self.ctrl = SafetyController(max_candidate_distance_m=max_radius)

    def record(self, phase, **details):
        self.events.append(dict(time_epoch=time.time(), phase=phase, **details))

    def run(self, *, execute=False, confirm=input):
        if self.used:
            raise RuntimeError("Create a new mission; tokens must never be retried")
        self.used = True
        attempted = False
        result = {"ok": False, "execution_attempted": False,
                  "arrival_verified": False, "events": self.events}
        try:
            self._health()
            bearing = self.observe()
            if not math.isfinite(bearing):
                raise ValueError("nonfinite_target_bearing")
            self.record("DETECTED", bearing_deg=bearing)
            preview = self.api.preview()
            recommendation = recommend_option(preview, bearing, self.max_radius)
            option = recommendation["selected_option"]
            if option is None:
                raise ValueError(recommendation["reason"])
            origin = preview.get("current_pose") or {}
            candidate = CandidateGoal(float(option["x"]), float(option["y"]),
                                      math.radians(float(option["yaw_deg"])), "map")
            self.ctrl.start_preview(candidate, robot_xy=(float(origin["x"]), float(origin["y"])))
            self.record("RECOMMENDED", session_id=preview["session_id"], option=option)
            # Provider preview may take seconds; observe again before preparation.
            self._recheck(preview, option)
            prepared = self.api.prepare(preview["session_id"], option["option_id"])
            if prepared.get("ok") is not True:
                raise ValueError("prepare_rejected: " + str(prepared.get("reason")))
            self.record("PREPARED", option_id=option["option_id"],
                        target=prepared.get("target"), expires_at_epoch=prepared.get("expires_at_epoch"))
            local_token = self.ctrl.prepare(True, "official Provider accepted selected candidate")
            if not execute:
                return dict(result, ok=True, reason="preview_only", snapshot=self.ctrl.snapshot())
            phrase = prepared.get("confirmation_text")
            token = prepared.get("token")
            if not isinstance(phrase, str) or not phrase or not isinstance(token, str) or not token:
                raise ValueError("invalid_prepared_response")
            self.ctrl.begin_confirmation()
            answer = confirm("候选 {}，距离 {} m，方向 {}°。输入“确认执行”继续，其他输入取消：".format(
                option["option_id"], option["radius_m"], option["relative_angle_deg"]))
            if answer.strip() != "确认执行":
                self.record("CANCELED_BEFORE_EXECUTION")
                self.ctrl.mark_failed("human_declined; no execution")
                return dict(result, reason="human_declined", snapshot=self.ctrl.snapshot())
            self.ctrl.confirm(local_token, human_ok=True)
            self._health()
            self._recheck(preview, option)
            expiry = float(prepared.get("expires_at_epoch", 0))
            if not math.isfinite(expiry) or expiry <= time.time():
                raise ValueError("prepared_token_expired_or_missing_expiry")
            self.ctrl.mark_executing()
            self.record("EXECUTING")
            attempted = True
            result["execution_attempted"] = True
            outcome = self.api.execute(token, phrase)
            # Execution is synchronous: the official provider monitors the specific
            # goal, watchdog and terminal status before returning this payload.
            success = (outcome.get("ok") is True and outcome.get("arrived") is True
                       and outcome.get("move_base_status_name") == "SUCCEEDED")
            preempted = (outcome.get("move_base_status_name") in {"PREEMPTED", "RECALLED"}
                         and outcome.get("terminal_status_known") is True)
            if success:
                self.ctrl.mark_succeeded()
            elif preempted:
                self.ctrl.mark_preempted("Provider reported terminal cancellation")
            else:
                self.ctrl.mark_failed(str(outcome.get("reason", "unverified_result")))
            self.record(self.ctrl.snapshot()["phase"], outcome=outcome)
            if not success and not preempted:
                self._cancel()
            return dict(result, ok=success or preempted, arrival_verified=success, snapshot=self.ctrl.snapshot(),
                        reason="arrived" if success else "preempted" if preempted else str(outcome.get("reason", "unverified_result")))
        except (Exception, KeyboardInterrupt) as exc:
            self.record("INTERRUPTED" if isinstance(exc, KeyboardInterrupt) else "REJECTED",
                        error=str(exc))
            if attempted:
                self._cancel()
            if self.ctrl.session.phase not in {Phase.SUCCEEDED, Phase.PREEMPTED, Phase.FAILED}:
                self.ctrl.mark_failed(str(exc) or "interrupted; stop not verified")
            return dict(result, snapshot=self.ctrl.snapshot(), reason="interrupted" if isinstance(exc, KeyboardInterrupt) else str(exc))

    def _recheck(self, preview, option):
        # Check the SAME selected option; a changed observation requires a new
        # preview and new human confirmation, never a silent candidate switch.
        selected = dict(preview, options=[option])
        if recommend_option(selected, self.observe(), self.max_radius)["selected_option"] is None:
            raise ValueError("target_changed_or_session_expired")

    def _health(self):
        health = self.api.health()
        self.record("HEALTH", report=health)
        if health.get("ok") is not True:
            raise ValueError("chassis_health_failed")

    def _cancel(self):
        try:
            outcome = self.api.cancel()
            self.record("CANCEL_REQUESTED", outcome=outcome,
                        stop_verified=False)
        except Exception as exc:
            self.record("CANCEL_FAILED", error=str(exc), stop_verified=False)
