"""导航客户端：只发 PoseStamped goal / cancel，订阅 move_base status。"""

from __future__ import annotations

import math
from typing import Any, Callable

from x3_nav.bridge.client import RosBridge
from x3_nav.safety.controller import CandidateGoal


def yaw_to_quaternion(yaw: float) -> dict[str, float]:
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(yaw / 2.0),
        "w": math.cos(yaw / 2.0),
    }


def pose_xy_from_odometry(msg: dict[str, Any]) -> tuple[float, float] | None:
    try:
        p = msg["pose"]["pose"]["position"]
        xy = float(p["x"]), float(p["y"])
        return xy if all(math.isfinite(v) for v in xy) else None
    except Exception:
        return None


def pose_xy_from_amcl(msg: dict[str, Any]) -> tuple[float, float] | None:
    return pose_xy_from_odometry(msg)


class NavClient:
    def __init__(self, bridge: RosBridge) -> None:
        self.bridge = bridge
        topics = bridge.config["topics"]
        self.goal_topic = topics["move_base_goal"]
        self.cancel_topic = topics["move_base_cancel"]
        self.status_topic = topics["move_base_status"]
        self.odom_topic = topics["odom"]
        self.amcl_topic = topics["amcl_pose"]

    def publish_goal(self, goal: CandidateGoal) -> None:
        q = yaw_to_quaternion(goal.yaw)
        msg = {
            "header": {"frame_id": goal.frame_id},
            "pose": {
                "position": {"x": float(goal.x), "y": float(goal.y), "z": 0.0},
                "orientation": q,
            },
        }
        self.bridge.publish(self.goal_topic, "geometry_msgs/PoseStamped", msg, keep_advertised=False)

    def cancel(self) -> None:
        self.bridge.publish(
            self.cancel_topic,
            "actionlib_msgs/GoalID",
            {"stamp": {"secs": 0, "nsecs": 0}, "id": ""},
            keep_advertised=False,
        )

    def subscribe_status(self, callback: Callable[[dict[str, Any]], None]) -> Any:
        return self.bridge.subscribe(self.status_topic, "actionlib_msgs/GoalStatusArray", callback)

    def subscribe_odom(self, callback: Callable[[dict[str, Any]], None]) -> Any:
        return self.bridge.subscribe(self.odom_topic, "nav_msgs/Odometry", callback)

    def subscribe_amcl(self, callback: Callable[[dict[str, Any]], None]) -> Any:
        return self.bridge.subscribe(
            self.amcl_topic,
            "geometry_msgs/PoseWithCovarianceStamped",
            callback,
        )
