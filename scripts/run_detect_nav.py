"""目标检测驱动短距导航（拓展课题最小闭环）。

默认 --mock 不连车：Mock 检测 → 候选点 → 安全预览/令牌干跑。
实车使用 run_visual_nav.py；此旧入口禁止下发坐标。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from x3_nav.config import load_config
from x3_nav.vision.pipeline import DetectNavPipeline


def main() -> int:
    parser = argparse.ArgumentParser(description="检测驱动短距安全导航")
    parser.add_argument("--mock", action="store_true", help="使用 Mock 检测（推荐先跑）")
    parser.add_argument("--label", type=str, default=None, help="目标标签，默认读 configs/robot.yaml")
    parser.add_argument("--offset-x", type=float, default=0.15, help="目标水平偏移 [-1,1]，右为正")
    parser.add_argument("--size", type=float, default=0.22, help="目标相对大小，越大越近")
    parser.add_argument("--confidence", type=float, default=0.86)
    parser.add_argument("--confirm", action="store_true", help="人工确认")
    parser.add_argument("--dispatch", action="store_true", help="真下发 move_base goal（需 --confirm）")
    parser.add_argument("--auto-preempt", action="store_true")
    args = parser.parse_args()
    if args.dispatch:
        parser.error("此入口仅保留离线演示；实车请使用 run_visual_nav.py 的官方候选与令牌流程")

    cfg = load_config()
    pipe = DetectNavPipeline(cfg)

    plan, result = pipe.dry_run_mock(
        label=args.label,
        offset_x=args.offset_x,
        size=args.size,
        confidence=args.confidence,
        human_confirm=args.confirm,
    )
    print(json.dumps({
        "mode": "mock-dry-run",
        "plan": plan.to_dict(),
        "ok": result.ok,
        "phase": result.phase,
        "detail": result.detail,
        "snapshot": result.snapshot,
    }, ensure_ascii=False, indent=2))
    return 0 if result.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
