"""把检测框映射为短距离导航候选点（不直接发 /cmd_vel）。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from x3_nav.safety.controller import CandidateGoal


@dataclass
class Detection:
    """图像平面上的一次检测结果（归一化坐标更稳）。"""

    label: str
    confidence: float
    # bbox 中心相对图像中心的归一化偏移：左负右正，范围约 [-1, 1]
    offset_x: float
    # 画面相对大小只决定启发式步长，不代表米制距离
    size: float = 0.2
    source: str = "mock"

    def to_dict(self) -> dict[str, Any]:
        return {
            "label": self.label,
            "confidence": self.confidence,
            "offset_x": self.offset_x,
            "size": self.size,
            "source": self.source,
        }


class MockDetector:
    """离线演示用：模拟「看到」某个标签目标。"""

    def __init__(
        self,
        label: str = "person",
        confidence: float = 0.86,
        offset_x: float = 0.15,
        size: float = 0.22,
    ) -> None:
        self.label = label
        self.confidence = confidence
        self.offset_x = offset_x
        self.size = size

    def detect(self) -> Detection | None:
        return Detection(
            label=self.label,
            confidence=self.confidence,
            offset_x=self.offset_x,
            size=self.size,
            source="mock",
        )


def detection_to_goal(
    det: Detection,
    *,
    robot_xy: tuple[float, float] = (0.0, 0.0),
    robot_yaw: float = 0.0,
    max_distance_m: float = 0.80,
    min_distance_m: float = 0.20,
    max_bearing_deg: float = 30.0,
    frame_id: str = "map",
    target_label: str | None = None,
    min_confidence: float = 0.50,
) -> CandidateGoal:
    """
    规则（刻意简单，便于课题验收）：
    - 置信度 / 标签过滤
    - 水平偏移 → 车头前方偏航角（限制在 ±max_bearing_deg）
    - 目标越大 → 前进距离越短（已靠近则少走）
    - 距离夹在 [min_distance_m, max_distance_m]
    """
    values = (det.confidence, det.offset_x, det.size, *robot_xy, robot_yaw,
              max_distance_m, min_distance_m, max_bearing_deg, min_confidence)
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        raise ValueError("Detection, pose and limits must be finite")
    if not 0 <= det.confidence <= 1 or not 0 <= min_confidence <= 1:
        raise ValueError("Confidence must be in [0,1]")
    if not 0 < min_distance_m <= max_distance_m <= .8 or not 0 <= max_bearing_deg <= 30:
        raise ValueError("Invalid short-step limits")
    if target_label and det.label != target_label:
        raise ValueError(f"标签不匹配：期望 {target_label!r}，得到 {det.label!r}")
    if det.confidence < min_confidence:
        raise ValueError(f"置信度过低：{det.confidence:.2f} < {min_confidence:.2f}")

    # Pixel x grows right; positive ROS yaw points left.
    bearing = -max(-1.0, min(1.0, det.offset_x)) * math.radians(max_bearing_deg)
    size = max(0.0, min(1.0, det.size))
    # size=1 → min_distance；size→0 → max_distance
    distance = max_distance_m - (max_distance_m - min_distance_m) * size
    distance = max(min_distance_m, min(max_distance_m, distance))

    yaw = robot_yaw + bearing
    x = robot_xy[0] + distance * math.cos(yaw)
    y = robot_xy[1] + distance * math.sin(yaw)
    return CandidateGoal(x=x, y=y, yaw=yaw, frame_id=frame_id)
