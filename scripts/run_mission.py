"""完整主实验任务：连通 → 健康检查 → 预览/令牌 →（可选）确认下发 → 监控/安全取消。"""

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
    parser = argparse.ArgumentParser(description="安全点位导航完整任务")
    parser.add_argument("--x", type=float, default=None, help="目标 x；默认用 candidate_distance_m")
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--confirm", action="store_true", help="人工确认（否则停在准备阶段）")
    parser.add_argument("--dispatch", action="store_true", help="确认后真下发 goal（实车危险，需同时 --confirm）")
    parser.add_argument(
        "--auto-preempt",
        action="store_true",
        help="按配置位移阈值自动 cancel，用于验证 PREEMPTED",
    )
    parser.add_argument("--timeout", type=float, default=None)
    args = parser.parse_args()

    cfg = load_config()
    dist = float(cfg["safety"]["candidate_distance_m"])
    x = dist if args.x is None else args.x
    goal = CandidateGoal(x=x, y=args.y, yaw=args.yaw, frame_id=cfg["mission"]["frame_id"])
    timeout = args.timeout if args.timeout is not None else float(cfg["mission"].get("default_timeout_sec", 120))

    runner = MissionRunner(cfg)
    result = runner.run(
        goal,
        human_confirm=args.confirm,
        dispatch=args.dispatch,
        auto_preempt=args.auto_preempt,
        timeout_sec=timeout,
    )
    print(json.dumps({
        "ok": result.ok,
        "phase": result.phase,
        "status_name": result.status_name,
        "detail": result.detail,
        "log_path": result.log_path,
        "snapshot": result.snapshot,
    }, ensure_ascii=False, indent=2))
    return 0 if result.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
