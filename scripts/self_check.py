"""禁止直接发布 /cmd_vel 的单元自检（不需要连车）。"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from x3_nav.bridge import ForbiddenPublishError, RosBridge
from x3_nav.config import load_config
from x3_nav.mission.runner import MissionRunner
from x3_nav.safety import CandidateGoal


def main() -> int:
    cfg = load_config()
    bridge = RosBridge(cfg)
    try:
        bridge.assert_allowed_publish("/cmd_vel")
        print("FAIL: 应禁止 /cmd_vel")
        return 1
    except ForbiddenPublishError:
        print("PASS: /cmd_vel 禁止发布")

    # goal/cancel 应允许（仅校验白名单逻辑，不真正连接）
    try:
        bridge.assert_allowed_publish(cfg["topics"]["move_base_goal"])
        bridge.assert_allowed_publish(cfg["topics"]["move_base_cancel"])
        print("PASS: goal/cancel 允许发布")
    except ForbiddenPublishError as e:
        print("FAIL:", e)
        return 1

    result = MissionRunner(cfg).dry_run(CandidateGoal(0.8, 0.0), human_confirm=True)
    if result.phase != "EXECUTING":
        print("FAIL: dry-run phase=", result.phase)
        return 1
    print("PASS: 安全状态机 dry-run ->", result.phase)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
