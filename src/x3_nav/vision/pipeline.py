"""检测驱动导航管线：detect → candidate → 安全状态机（可接 MissionRunner）。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from x3_nav.mission.runner import MissionResult, MissionRunner
from x3_nav.safety.controller import CandidateGoal, SafetyController
from x3_nav.vision.detect import Detection, MockDetector, detection_to_goal


@dataclass
class DetectNavPlan:
    detection: Detection
    goal: CandidateGoal
    notes: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "detection": self.detection.to_dict(),
            "goal": {
                "x": self.goal.x,
                "y": self.goal.y,
                "yaw": self.goal.yaw,
                "frame_id": self.goal.frame_id,
            },
            "notes": list(self.notes),
        }


class DetectNavPipeline:
    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config
        vision = config.get("vision", {})
        safety = config["safety"]
        self.target_label = str(vision.get("target_label", "person"))
        self.min_confidence = float(vision.get("min_confidence", 0.50))
        self.max_distance_m = float(safety.get("candidate_distance_m", 0.80))
        self.min_distance_m = float(vision.get("min_approach_m", 0.20))
        self.max_bearing_deg = float(vision.get("max_bearing_deg", 30.0))
        self.frame_id = str(config["mission"].get("frame_id", "map"))
        self.ctrl = SafetyController(
            token_ttl_sec=float(config["mission"].get("token_ttl_sec", 60)),
            max_candidate_distance_m=self.max_distance_m,
        )

    def plan_from_detection(
        self,
        det: Detection,
        *,
        robot_xy: tuple[float, float] = (0.0, 0.0),
        robot_yaw: float = 0.0,
    ) -> DetectNavPlan:
        goal = detection_to_goal(
            det,
            robot_xy=robot_xy,
            robot_yaw=robot_yaw,
            max_distance_m=self.max_distance_m,
            min_distance_m=self.min_distance_m,
            max_bearing_deg=self.max_bearing_deg,
            frame_id=self.frame_id,
            target_label=self.target_label,
            min_confidence=self.min_confidence,
        )
        notes = [
            f"标签={det.label} conf={det.confidence:.2f} source={det.source}",
            f"映射距离≈{goal.distance_from(*robot_xy):.3f} m，偏航计入 bearing 限制",
        ]
        return DetectNavPlan(detection=det, goal=goal, notes=notes)

    def dry_run_mock(
        self,
        *,
        label: str | None = None,
        offset_x: float = 0.15,
        size: float = 0.22,
        confidence: float = 0.86,
        human_confirm: bool = False,
    ) -> tuple[DetectNavPlan, MissionResult]:
        """不连车：Mock 检测 + 安全状态机干跑。"""
        det = MockDetector(
            label=label or self.target_label,
            confidence=confidence,
            offset_x=offset_x,
            size=size,
        ).detect()
        assert det is not None
        plan = self.plan_from_detection(det)
        # 预览阶段强制距离校验
        self.ctrl.start_preview(plan.goal, robot_xy=(0.0, 0.0), enforce_distance=True)
        token = self.ctrl.prepare(system_ok=True, detail="detect-nav dry-run")
        if human_confirm:
            self.ctrl.confirm(token, human_ok=True)
            self.ctrl.mark_executing()
        result = MissionResult(
            ok=True,
            phase=self.ctrl.snapshot()["phase"],
            detail="detect-nav dry-run（未连接 rosbridge，未发 goal）",
            snapshot=self.ctrl.snapshot(),
        )
        return plan, result

    def run_live(
        self,
        det: Detection,
        *,
        human_confirm: bool,
        dispatch: bool,
        auto_preempt: bool = False,
        timeout_sec: float | None = None,
    ) -> tuple[DetectNavPlan, MissionResult]:
        """连车：把检测结果交给既有 MissionRunner（仍禁 /cmd_vel）。"""
        plan = self.plan_from_detection(det)
        runner = MissionRunner(self.config)
        timeout = timeout_sec
        if timeout is None:
            timeout = float(self.config["mission"].get("default_timeout_sec", 120))
        result = runner.run(
            plan.goal,
            human_confirm=human_confirm,
            dispatch=dispatch,
            auto_preempt=auto_preempt,
            timeout_sec=timeout,
        )
        return plan, result
