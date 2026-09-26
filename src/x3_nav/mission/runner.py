"""完整任务：预览 → 健康检查 → 令牌 → 确认 → 下发 → 监控 / 安全取消。"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any

from x3_nav.bridge.client import RosBridge
from x3_nav.config import load_config, project_root
from x3_nav.logutil import write_json_log
from x3_nav.nav.client import NavClient, pose_xy_from_amcl, pose_xy_from_odometry
from x3_nav.nav.status import GoalStatusMonitor
from x3_nav.safety.controller import CandidateGoal, Phase, SafetyController
from x3_nav.validation.topics import check_topic_rates, summarize_health


@dataclass
class MissionResult:
    ok: bool
    phase: str
    status_name: str | None = None
    detail: str = ""
    snapshot: dict[str, Any] = field(default_factory=dict)
    log_path: str | None = None


class MissionRunner:
    def __init__(self, config: dict[str, Any] | None = None) -> None:
        self.config = config or load_config()
        safety = self.config["safety"]
        self.ctrl = SafetyController(
            token_ttl_sec=float(self.config["mission"].get("token_ttl_sec", 60)),
            max_candidate_distance_m=float(safety.get("candidate_distance_m", 0.80)),
        )
        self.preempt_trigger_m = float(safety.get("preempt_trigger_displacement_m", 0.080))

    def dry_run(self, goal: CandidateGoal, *, human_confirm: bool = False) -> MissionResult:
        """不连 ROS，只验证安全状态机。"""
        self.ctrl.start_preview(goal, robot_xy=(0.0, 0.0), enforce_distance=True)
        token = self.ctrl.prepare(system_ok=True, detail="dry-run")
        if human_confirm:
            self.ctrl.confirm(token, human_ok=True)
            self.ctrl.mark_executing()
        snap = self.ctrl.snapshot()
        return MissionResult(
            ok=True,
            phase=snap["phase"],
            detail="dry-run 完成（未连接 rosbridge）",
            snapshot=snap,
        )

    def run(
        self,
        goal: CandidateGoal,
        *,
        human_confirm: bool,
        dispatch: bool,
        auto_preempt: bool = False,
        timeout_sec: float = 120.0,
        health_duration_sec: float = 2.0,
        save_log: bool = True,
    ) -> MissionResult:
        if not human_confirm and dispatch:
            raise RuntimeError("红线：未人工确认时禁止 dispatch")

        with RosBridge(self.config) as bridge:
            nav = NavClient(bridge)
            status_mon = GoalStatusMonitor()
            nav.subscribe_status(status_mon.callback)

            robot_xy_box: dict[str, Any] = {"xy": None}

            def on_odom(msg: dict[str, Any]) -> None:
                xy = pose_xy_from_odometry(msg)
                if xy is not None:
                    robot_xy_box["xy"] = xy

            def on_amcl(msg: dict[str, Any]) -> None:
                xy = pose_xy_from_amcl(msg)
                if xy is not None:
                    robot_xy_box["xy"] = xy

            sub_odom = nav.subscribe_odom(on_odom)
            sub_amcl = nav.subscribe_amcl(on_amcl)
            time.sleep(0.8)
            start_xy = robot_xy_box["xy"] or (0.0, 0.0)

            # 1) 预览（不下发）
            self.ctrl.start_preview(goal, robot_xy=start_xy, enforce_distance=True)

            # 2) 数据健康检查
            report = check_topic_rates(bridge, duration_sec=health_duration_sec)
            health = summarize_health(report)
            if not health["ok"]:
                self.ctrl.mark_failed(f"话题不健康: {health['missing']}")
                return self._finish(False, "话题不健康", save_log=save_log, extra={"health": health})

            # 3) 准备令牌
            token = self.ctrl.prepare(system_ok=True, detail="topics ok")

            if not human_confirm:
                return self._finish(True, "已停在准备阶段（未确认）", save_log=save_log, extra={"health": health, "token": token})

            # 4) 人工确认
            self.ctrl.confirm(token, human_ok=True)
            ready = self.ctrl.mark_executing()

            if not dispatch:
                return self._finish(True, "已确认但未 dispatch", save_log=save_log, extra={"health": health})

            # 5) 下发目标
            nav.publish_goal(ready)
            self.ctrl.session.log("已发布 PoseStamped goal")

            # 6) 监控
            deadline = time.time() + timeout_sec
            while time.time() < deadline:
                xy = robot_xy_box["xy"]
                if auto_preempt and xy is not None and self.ctrl.should_preempt_by_displacement(xy, self.preempt_trigger_m):
                    nav.cancel()
                    self.ctrl.mark_preempted(
                        f"累计位移达到约 {self.preempt_trigger_m:.3f} m，触发安全取消"
                    )
                    time.sleep(0.5)
                    return self._finish(
                        True,
                        "PREEMPTED（安全取消）",
                        save_log=save_log,
                        extra={"health": health, "status": status_mon.snapshot(), "xy": xy},
                    )

                code = status_mon.last_status_code
                if code == 3:
                    self.ctrl.mark_succeeded()
                    return self._finish(
                        True,
                        "SUCCEEDED",
                        save_log=save_log,
                        extra={"health": health, "status": status_mon.snapshot(), "xy": xy},
                    )
                if code == 2:
                    self.ctrl.mark_preempted("收到 move_base PREEMPTED")
                    return self._finish(
                        True,
                        "PREEMPTED",
                        save_log=save_log,
                        extra={"health": health, "status": status_mon.snapshot(), "xy": xy},
                    )
                if code in {4, 5, 9}:
                    name = status_mon.last_status_name or str(code)
                    self.ctrl.mark_failed(name)
                    return self._finish(
                        False,
                        name,
                        save_log=save_log,
                        extra={"health": health, "status": status_mon.snapshot(), "xy": xy},
                    )
                time.sleep(0.1)

            self.ctrl.mark_failed("监控超时")
            try:
                sub_odom.unsubscribe()
                sub_amcl.unsubscribe()
            except Exception:
                pass
            return self._finish(
                False,
                "监控超时",
                save_log=save_log,
                extra={"health": health, "status": status_mon.snapshot()},
            )

    def _finish(
        self,
        ok: bool,
        detail: str,
        *,
        save_log: bool,
        extra: dict[str, Any] | None = None,
    ) -> MissionResult:
        snap = self.ctrl.snapshot()
        payload = {"detail": detail, "snapshot": snap, "extra": extra or {}}
        log_path = None
        if save_log:
            path = write_json_log(project_root(), "mission", payload)
            log_path = str(path)
        return MissionResult(
            ok=ok,
            phase=snap["phase"],
            status_name=(extra or {}).get("status", {}).get("status_name") if extra else None,
            detail=detail,
            snapshot=snap,
            log_path=log_path,
        )


def offset_goal_from_robot(
    robot_xy: tuple[float, float],
    distance_m: float,
    yaw: float = 0.0,
    frame_id: str = "map",
) -> CandidateGoal:
    """沿当前 yaw（默认 0，即 +x）生成短距离候选点。"""
    x = robot_xy[0] + distance_m * math.cos(yaw)
    y = robot_xy[1] + distance_m * math.sin(yaw)
    return CandidateGoal(x=x, y=y, yaw=yaw, frame_id=frame_id)
