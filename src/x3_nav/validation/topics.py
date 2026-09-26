"""话题频率与基本有效性检查。"""

from __future__ import annotations

import time
from collections import defaultdict
from typing import Any

from x3_nav.bridge.client import RosBridge

TOPIC_TYPES = {
    "scan": "sensor_msgs/LaserScan",
    "odom": "nav_msgs/Odometry",
    "amcl_pose": "geometry_msgs/PoseWithCovarianceStamped",
    "move_base_status": "actionlib_msgs/GoalStatusArray",
}


def check_topic_rates(
    bridge: RosBridge,
    duration_sec: float = 3.0,
    keys: list[str] | None = None,
) -> dict[str, Any]:
    topics_cfg = bridge.config["topics"]
    keys = keys or ["scan", "odom", "amcl_pose", "move_base_status"]
    counts: dict[str, int] = defaultdict(int)
    last_msg: dict[str, Any] = {}
    subs = []

    def make_cb(name: str):
        def _cb(msg: dict[str, Any]) -> None:
            counts[name] += 1
            last_msg[name] = msg

        return _cb

    for key in keys:
        topic = topics_cfg[key]
        msg_type = TOPIC_TYPES[key]
        subs.append(bridge.subscribe(topic, msg_type, make_cb(key)))

    time.sleep(duration_sec)
    for s in subs:
        try:
            s.unsubscribe()
        except Exception:
            pass

    report: dict[str, Any] = {}
    for key in keys:
        n = counts[key]
        report[key] = {
            "topic": topics_cfg[key],
            "count": n,
            "hz_est": round(n / duration_sec, 2) if duration_sec > 0 else 0.0,
            "ok": n > 0,
        }
    return report


def summarize_health(report: dict[str, Any], required: list[str] | None = None) -> dict[str, Any]:
    required = required or ["scan", "odom", "amcl_pose"]
    missing = [k for k in required if not report.get(k, {}).get("ok")]
    return {
        "ok": len(missing) == 0,
        "missing": missing,
        "report": report,
    }
