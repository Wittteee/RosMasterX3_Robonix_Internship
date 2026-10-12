"""Observe ROS goal/velocity topics while calling the actual Provider preview."""
from __future__ import annotations

import importlib
import json
from pathlib import Path
import sys
import time

import roslibpy

ROOT = Path.home() / "rosmaster_x3_deploy"
sys.path.insert(0, str(ROOT / "scripts"))
from move_base_cli import ProviderApi


def main() -> int:
    api = ProviderApi()
    observer = api._provider.ensure_bridge().ros
    counts = {"goal_messages": 0, "nonzero_velocity_messages": 0}
    subs = []

    def goal_callback(_):
        counts["goal_messages"] += 1

    def velocity_callback(message):
        values = [message.get("linear", {}).get(axis, 0) for axis in ("x", "y", "z")]
        values += [message.get("angular", {}).get(axis, 0) for axis in ("x", "y", "z")]
        if any(abs(float(value)) > 1e-6 for value in values):
            counts["nonzero_velocity_messages"] += 1

    try:
        for name, kind, callback in (
            ("/move_base_simple/goal", "geometry_msgs/PoseStamped", goal_callback),
            ("/cmd_vel", "geometry_msgs/Twist", velocity_callback),
        ):
            sub = roslibpy.Topic(observer, name, kind)
            sub.subscribe(callback)
            subs.append(sub)
        time.sleep(0.5)
        preview = api.preview(5)
        time.sleep(0.5)
        result = {"real_provider": True, "rosbridge_connected": observer.is_connected,
                  "preview": preview, "observation": counts,
                  "execution_requested": False}
        result["check_passed"] = (preview.get("reason") in {"nav_not_ready", "amcl_unavailable"} and
                                  observer.is_connected and
                                  preview.get("preview_only") is True and
                                  counts["goal_messages"] == 0 and
                                  counts["nonzero_velocity_messages"] == 0 and
                                  preview.get("navigation_goal_sent") is False)
        log = ROOT / "logs" / "official_preview.json"
        log.parent.mkdir(parents=True, exist_ok=True)
        log.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result["check_passed"] else 2
    finally:
        for sub in subs:
            sub.unsubscribe()
        api.close()


if __name__ == "__main__":
    raise SystemExit(main())
