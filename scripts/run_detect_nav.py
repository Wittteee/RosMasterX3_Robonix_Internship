"""目标检测驱动短距导航（拓展课题最小闭环）。

默认 --mock 不连车：Mock 检测 → 候选点 → 安全预览/令牌干跑。
实车需 rosbridge + 导航栈，并显式 --confirm --dispatch。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from x3_nav.config import load_config
from x3_nav.vision.detect import Detection
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

    cfg = load_config()
    pipe = DetectNavPipeline(cfg)

    if args.mock or not args.dispatch:
        # 默认走 mock 干跑；即使未写 --mock，未 dispatch 时也优先演示映射逻辑
        use_mock = True if args.mock or not args.dispatch else False
    else:
        use_mock = False

    if use_mock and not args.dispatch:
        plan, result = pipe.dry_run_mock(
            label=args.label,
            offset_x=args.offset_x,
            size=args.size,
            confidence=args.confidence,
            human_confirm=args.confirm,
        )
        out = {
            "mode": "mock-dry-run",
            "plan": plan.to_dict(),
            "ok": result.ok,
            "phase": result.phase,
            "detail": result.detail,
            "snapshot": result.snapshot,
        }
        print(json.dumps(out, ensure_ascii=False, indent=2))
        return 0 if result.ok else 1

    # 实车：当前用 MockDetection 注入（后续可换成 YOLO 话题订阅）
    label = args.label or str(cfg.get("vision", {}).get("target_label", "person"))
    det = Detection(
        label=label,
        confidence=args.confidence,
        offset_x=args.offset_x,
        size=args.size,
        source="cli-injection",
    )
    plan, result = pipe.run_live(
        det,
        human_confirm=args.confirm,
        dispatch=args.dispatch,
        auto_preempt=args.auto_preempt,
    )
    out = {
        "mode": "live",
        "plan": plan.to_dict(),
        "ok": result.ok,
        "phase": result.phase,
        "detail": result.detail,
        "log_path": result.log_path,
        "snapshot": result.snapshot,
    }
    print(json.dumps(out, ensure_ascii=False, indent=2))
    return 0 if result.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
