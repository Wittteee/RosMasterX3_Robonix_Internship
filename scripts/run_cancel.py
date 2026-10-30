"""发送 move_base cancel（安全取消联调）。"""

from __future__ import annotations

import sys
import argparse
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from x3_nav.bridge import RosBridge
from x3_nav.config import load_config
from x3_nav.nav import NavClient


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    cfg = load_config(args.config)
    if args.dry_run:
        from x3_nav.bridge.offline import OfflineBridge
        with OfflineBridge(cfg) as bridge:
            NavClient(bridge).cancel()
            print('dry-run：仅记录内存 cancel，未连接实车')
        return 0
    with RosBridge(cfg) as bridge:
        NavClient(bridge).cancel()
        print("已请求取消；本命令未确认导航终态或物理停车。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
