from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass
from typing import Any

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    import numpy as np


@dataclass(frozen=True)
class Target:
    label: str
    center_x: float
    center_y: float
    corners: list[list[float]]
    image_width: int
    image_height: int

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def detect_qr_targets(frame: np.ndarray, expected_label: str) -> list[Target]:
    import cv2
    if frame is None or frame.size == 0:
        raise ValueError("Empty image")
    if not expected_label.strip():
        raise ValueError("A nonempty target label is required")
    detected, labels, points, _ = cv2.QRCodeDetector().detectAndDecodeMulti(frame)
    if not detected or points is None:
        return []
    height, width = frame.shape[:2]
    targets = []
    for label, corners in zip(labels, points):
        if label != expected_label:
            continue
        center = corners.mean(axis=0)
        targets.append(Target(label, float(center[0]), float(center[1]),
                              corners.astype(float).tolist(), width, height))
    return targets


def target_bearing_deg(target: Target, fx: float, cx: float, camera_yaw_deg: float) -> float:
    if not all(math.isfinite(v) for v in (fx, cx, camera_yaw_deg)) or fx <= 0:
        raise ValueError("Camera calibration must be finite, with fx > 0")
    if not 0 <= cx < target.image_width:
        raise ValueError("cx must be inside the image")
    if not (math.isfinite(target.center_x) and math.isfinite(target.center_y)
            and 0 <= target.center_x < target.image_width
            and 0 <= target.center_y < target.image_height):
        raise ValueError("Target center must be inside the image")
    # Image x grows rightwards; ROS base yaw grows leftwards.
    return camera_yaw_deg + math.degrees(math.atan2(cx - target.center_x, fx))


def recommend_option(preview: dict[str, Any], bearing_deg: float,
                     max_radius_m: float = 0.2, tolerance_deg: float = 12.0) -> dict[str, Any]:
    if not math.isfinite(bearing_deg):
        raise ValueError("Target bearing must be finite")
    if not math.isfinite(max_radius_m) or not 0 < max_radius_m <= 0.8:
        raise ValueError("Preview radius must be in (0, 0.8] metres")
    if not math.isfinite(tolerance_deg) or not 0 < tolerance_deg <= 30:
        raise ValueError("Angle tolerance must be in (0, 30] degrees")
    result = {"preview_only": True, "navigation_goal_sent": False,
              "target_bearing_deg": bearing_deg, "selected_option": None}
    if preview.get("ok") is not True or not preview.get("session_id"):
        return dict(result, reason="provider_preview_unavailable")
    expiry = preview.get("expires_epoch")
    if expiry is not None:
        try:
            expiry = float(expiry)
        except (ValueError, TypeError, OverflowError):
            return dict(result, reason="provider_session_expired")
        if not math.isfinite(expiry) or expiry <= time.time():
            return dict(result, reason="provider_session_expired")
    if abs(bearing_deg) > 30:
        return dict(result, reason="target_outside_forward_sector")
    candidates = []
    options = preview.get("options")
    if not isinstance(options, list):
        return dict(result, reason="no_matching_provider_option")
    ids = [o.get("option_id") for o in options if isinstance(o, dict)]
    for option in options:
        if not isinstance(option, dict):
            continue
        try:
            angle = float(option["relative_angle_deg"])
            radius = float(option["radius_m"])
            option_id = option["option_id"]
            if type(option_id) is not int or ids.count(option_id) != 1:
                continue
        except (KeyError, ValueError, TypeError, OverflowError):
            continue
        if (not all(math.isfinite(v) for v in (angle, radius)) or
                abs(angle) > 30 or not 0 < radius <= max_radius_m or option_id <= 0):
            continue
        error = abs(angle - bearing_deg)
        if error <= tolerance_deg:
            candidates.append((error, radius, option_id, option))
    if not candidates:
        return dict(result, reason="no_matching_provider_option")
    selected = min(candidates, key=lambda item: item[:3])[3]
    return dict(result, reason="candidate_recommended", session_id=preview["session_id"],
                selected_option=dict(selected))


def annotate_targets(frame: np.ndarray, targets: list[Target]) -> np.ndarray:
    import cv2
    import numpy as np
    image = frame.copy()
    for target in targets:
        corners = np.asarray(target.corners, dtype=np.int32)
        cv2.polylines(image, [corners], True, (30, 180, 30), 2)
        cv2.putText(image, target.label, (int(target.center_x), max(25, int(target.center_y) - 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 200), 2)
    return image
