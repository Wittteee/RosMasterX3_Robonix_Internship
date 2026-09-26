"""发送 move_base cancel（安全取消联调）。"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from x3_nav.bridge import RosBridge
from x3_nav.config import load_config
from x3_nav.nav import NavClient


def main() -> int:
    cfg = load_config()
    with RosBridge(cfg) as bridge:
        NavClient(bridge).cancel()
        print("已发布 cancel。若任务被保护性中止，状态应为 PREEMPTED。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
