"""安全管控状态机：预览 → 准备(令牌) → 确认 → 执行 → 监控/取消。"""

from __future__ import annotations

import math
import secrets
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any


class Phase(Enum):
    IDLE = auto()
    PREVIEW = auto()
    PREPARE = auto()
    CONFIRM = auto()
    EXECUTING = auto()
    SUCCEEDED = auto()
    PREEMPTED = auto()
    FAILED = auto()


@dataclass
class CandidateGoal:
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = "map"

    def distance_from(self, x0: float, y0: float) -> float:
        return math.hypot(self.x - x0, self.y - y0)


@dataclass
class SafetySession:
    phase: Phase = Phase.IDLE
    candidate: CandidateGoal | None = None
    token: str | None = None
    token_expires_at: float | None = None
    human_confirmed: bool = False
    start_xy: tuple[float, float] | None = None
    notes: list[str] = field(default_factory=list)

    def log(self, msg: str) -> None:
        self.notes.append(f"{time.strftime('%H:%M:%S')} {msg}")


class SafetyController:
    """对应 PPT：预览车静止 → 一次性令牌 → 人工授权 → 单次下发 → 可紧急取消。"""

    def __init__(self, token_ttl_sec: float = 60.0, max_candidate_distance_m: float = 0.80) -> None:
        self.token_ttl_sec = token_ttl_sec
        self.max_candidate_distance_m = max_candidate_distance_m
        self.session = SafetySession()

    def start_preview(
        self,
        candidate: CandidateGoal,
        *,
        robot_xy: tuple[float, float] | None = None,
        enforce_distance: bool = True,
    ) -> CandidateGoal:
        if enforce_distance and robot_xy is not None:
            dist = candidate.distance_from(*robot_xy)
            if dist > self.max_candidate_distance_m + 1e-6:
                raise ValueError(
                    f"候选点距离 {dist:.3f} m 超过配置上限 {self.max_candidate_distance_m:.3f} m"
                )
        self.session = SafetySession(phase=Phase.PREVIEW, candidate=candidate, start_xy=robot_xy)
        self.session.log("预览：已筛选候选点，机器人应保持静止（不下发 goal）")
        return candidate

    def prepare(self, system_ok: bool, detail: str = "") -> str:
        if self.session.phase != Phase.PREVIEW:
            raise RuntimeError("必须先完成预览阶段")
        if not system_ok:
            raise RuntimeError(f"系统状态未通过复核，不能生成令牌: {detail}")
        token = secrets.token_hex(8)
        self.session.token = token
        self.session.token_expires_at = time.time() + self.token_ttl_sec
        self.session.phase = Phase.PREPARE
        self.session.log("准备：全状态复核通过，已生成一次性执行令牌" + (f"（{detail}）" if detail else ""))
        return token

    def confirm(self, token: str, human_ok: bool) -> None:
        if self.session.phase != Phase.PREPARE:
            raise RuntimeError("必须先进入准备阶段")
        if not human_ok:
            raise RuntimeError("未获得人工明确授权")
        if token != self.session.token:
            raise RuntimeError("令牌不匹配")
        if self.session.token_expires_at and time.time() > self.session.token_expires_at:
            raise RuntimeError("令牌已过期，请重新准备")
        self.session.human_confirmed = True
        self.session.phase = Phase.CONFIRM
        self.session.log("确认：人工授权通过，允许单次下发目标")

    def mark_executing(self) -> CandidateGoal:
        if self.session.phase != Phase.CONFIRM or not self.session.human_confirmed:
            raise RuntimeError("未确认前禁止下发目标")
        if self.session.candidate is None:
            raise RuntimeError("无候选目标")
        self.session.token = None
        self.session.phase = Phase.EXECUTING
        self.session.log("执行：已允许下发 PoseStamped（令牌已失效，单次触发）")
        return self.session.candidate

    def should_preempt_by_displacement(self, current_xy: tuple[float, float], threshold_m: float) -> bool:
        if self.session.start_xy is None:
            return False
        dx = current_xy[0] - self.session.start_xy[0]
        dy = current_xy[1] - self.session.start_xy[1]
        return math.hypot(dx, dy) >= threshold_m

    def mark_succeeded(self) -> None:
        self.session.phase = Phase.SUCCEEDED
        self.session.log("结果：SUCCEEDED，到达并稳定停止")

    def mark_preempted(self, reason: str = "安全保护触发") -> None:
        self.session.phase = Phase.PREEMPTED
        self.session.log(f"结果：PREEMPTED（{reason}；主动中止，非导航失败）")

    def mark_failed(self, reason: str) -> None:
        self.session.phase = Phase.FAILED
        self.session.log(f"结果：FAILED（{reason}）")

    def snapshot(self) -> dict[str, Any]:
        c = self.session.candidate
        return {
            "phase": self.session.phase.name,
            "candidate": None if c is None else {"x": c.x, "y": c.y, "yaw": c.yaw, "frame_id": c.frame_id},
            "human_confirmed": self.session.human_confirmed,
            "has_token": self.session.token is not None,
            "start_xy": self.session.start_xy,
            "notes": list(self.session.notes),
        }
