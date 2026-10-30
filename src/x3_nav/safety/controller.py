"""安全管控状态机：预览 → 准备(令牌) → 确认 → 执行 → 监控/取消。"""

from __future__ import annotations

import math
import secrets
import time
import threading
from functools import wraps
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


@dataclass(frozen=True)
class CandidateGoal:
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = "map"

    def __post_init__(self):
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
               for v in (self.x, self.y, self.yaw)):
            raise ValueError("Candidate coordinates and yaw must be finite")
        if not isinstance(self.frame_id, str) or not self.frame_id.strip():
            raise ValueError("Candidate frame_id is required")

    def distance_from(self, x0: float, y0: float) -> float:
        return math.hypot(self.x - x0, self.y - y0)


@dataclass
class SafetySession:
    phase: Phase = Phase.IDLE
    candidate: CandidateGoal | None = None
    token: str | None = None
    token_expires_at: float | None = None
    token_exp: float | None = None
    human_confirmed: bool = False
    start_xy: tuple[float, float] | None = None
    notes: list[str] = field(default_factory=list)

    def log(self, msg: str) -> None:
        self.notes.append(f"{time.strftime('%H:%M:%S')} {msg}")


def _locked(method):
    @wraps(method)
    def call(self, *args, **kwargs):
        with self._lock:
            return method(self, *args, **kwargs)
    return call


class SafetyController:
    """对应 PPT：预览车静止 → 一次性令牌 → 人工授权 → 单次下发 → 可紧急取消。"""

    def __init__(self, token_ttl_sec: float = 60.0, max_candidate_distance_m: float = 0.80, *, clock=time.monotonic) -> None:
        if not math.isfinite(token_ttl_sec) or token_ttl_sec <= 0:
            raise ValueError("Token TTL must be finite and positive")
        if not math.isfinite(max_candidate_distance_m) or not 0 < max_candidate_distance_m <= .8:
            raise ValueError("Candidate limit must be in (0, 0.8]")
        self._clock = clock
        self._lock = threading.RLock()
        self.token_ttl_sec = token_ttl_sec
        self.max_candidate_distance_m = max_candidate_distance_m
        self.session = SafetySession()

    @_locked
    def start_preview(
        self,
        candidate: CandidateGoal,
        *,
        robot_xy: tuple[float, float] | None = None,
        enforce_distance: bool = True,
    ) -> CandidateGoal:
        if self.session.phase != Phase.IDLE:
            raise RuntimeError("One controller handles exactly one mission")
        if not enforce_distance or robot_xy is None or len(robot_xy) != 2 or not all(math.isfinite(v) for v in robot_xy):
            raise ValueError("A finite same-frame preview origin and distance check are mandatory")
        if enforce_distance:
            dist = candidate.distance_from(*robot_xy)
            if dist > self.max_candidate_distance_m + 1e-6:
                raise ValueError(
                    f"候选点距离 {dist:.3f} m 超过配置上限 {self.max_candidate_distance_m:.3f} m"
                )
        self.session = SafetySession(phase=Phase.PREVIEW, candidate=candidate, start_xy=robot_xy)
        self.session.log("预览：已筛选候选点，机器人应保持静止（不下发 goal）")
        return candidate

    @_locked
    def prepare(self, system_ok: bool, detail: str = "") -> str:
        if self.session.phase != Phase.PREVIEW:
            raise RuntimeError("必须先完成预览阶段")
        if system_ok is not True:
            raise RuntimeError(f"系统状态未通过复核，不能生成令牌: {detail}")
        token = secrets.token_hex(24)
        self.session.token = token
        self.session.token_expires_at = time.time() + self.token_ttl_sec
        self.session.token_exp = self._clock() + self.token_ttl_sec
        self.session.phase = Phase.PREPARE
        self.session.log("准备：全状态复核通过，已生成一次性执行令牌" + (f"（{detail}）" if detail else ""))
        return token

    @_locked
    def begin_confirmation(self) -> None:
        if self.session.phase != Phase.PREPARE:
            raise RuntimeError("必须先进入准备阶段")
        self._check_token()
        self.session.phase = Phase.CONFIRM
        self.session.log("CONFIRM：等待人工授权")

    def _check_token(self):
        if not self.session.token or self.session.token_exp is None or self._clock() >= self.session.token_exp:
            self.session.token = None
            raise RuntimeError("令牌不存在或已过期；必须创建新任务")

    @_locked
    def confirm(self, token: str, human_ok: bool) -> None:
        if self.session.phase != Phase.CONFIRM or self.session.human_confirmed:
            raise RuntimeError("必须处于等待人工授权的 CONFIRM 阶段")
        if human_ok is not True:
            raise RuntimeError("未获得人工明确授权")
        self._check_token()
        if not isinstance(token, str) or not secrets.compare_digest(token, self.session.token):
            raise RuntimeError("令牌不匹配")
        self.session.human_confirmed = True
        self.session.phase = Phase.CONFIRM
        self.session.log("确认：人工授权通过，允许单次下发目标")

    @_locked
    def mark_executing(self) -> CandidateGoal:
        if self.session.phase != Phase.CONFIRM or not self.session.human_confirmed:
            raise RuntimeError("未确认前禁止下发目标")
        if self.session.candidate is None:
            raise RuntimeError("无候选目标")
        self._check_token()
        self.session.token = None
        self.session.phase = Phase.EXECUTING
        self.session.log("执行：已允许下发 PoseStamped（令牌已失效，单次触发）")
        return self.session.candidate

    @_locked
    def should_preempt_by_displacement(self, current_xy: tuple[float, float], threshold_m: float) -> bool:
        if self.session.phase != Phase.EXECUTING:
            raise RuntimeError("Displacement protection only applies during execution")
        if not math.isfinite(threshold_m) or threshold_m <= 0 or not all(math.isfinite(v) for v in current_xy):
            raise ValueError("Invalid displacement data")
        if self.session.start_xy is None:
            return False
        dx = current_xy[0] - self.session.start_xy[0]
        dy = current_xy[1] - self.session.start_xy[1]
        return math.hypot(dx, dy) >= threshold_m

    @_locked
    def mark_succeeded(self) -> None:
        self._require_executing()
        self.session.phase = Phase.SUCCEEDED
        self.session.log("结果：SUCCEEDED（导航状态回报，不等于独立物理停车验证）")

    @_locked
    def mark_preempted(self, reason: str = "安全保护触发") -> None:
        self._require_executing()
        self.session.phase = Phase.PREEMPTED
        self.session.log(f"结果：PREEMPTED（{reason}；主动中止，非导航失败）")

    @_locked
    def mark_failed(self, reason: str) -> None:
        if self.session.phase in {Phase.SUCCEEDED, Phase.PREEMPTED, Phase.FAILED}:
            raise RuntimeError("Terminal states cannot be overwritten")
        self.session.token = None
        self.session.phase = Phase.FAILED
        self.session.log(f"结果：FAILED（{reason}）")

    def _require_executing(self):
        if self.session.phase != Phase.EXECUTING:
            raise RuntimeError("Terminal navigation result requires EXECUTING")

    @_locked
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
