"""干跑安全流程：预览 → 准备 → 确认（默认不连 ROS、不下发）。"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from x3_nav.config import load_config
from x3_nav.mission.runner import MissionRunner
from x3_nav.safety import CandidateGoal


def main() -> int:
    parser = argparse.ArgumentParser(description="安全导航状态机干跑")
    parser.add_argument("--x", type=float, default=0.8, help="候选点 x（map）")
    parser.add_argument("--y", type=float, default=0.0, help="候选点 y（map）")
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--confirm", action="store_true", help="模拟人工确认")
    args = parser.parse_args()

    cfg = load_config()
    frame = cfg["mission"]["frame_id"]
    runner = MissionRunner(cfg)
    goal = CandidateGoal(x=args.x, y=args.y, yaw=args.yaw, frame_id=frame)
    result = runner.dry_run(goal, human_confirm=args.confirm)
    print(json.dumps({
        "ok": result.ok,
        "phase": result.phase,
        "detail": result.detail,
        "snapshot": result.snapshot,
    }, ensure_ascii=False, indent=2))
    return 0 if result.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
