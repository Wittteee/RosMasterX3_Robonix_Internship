"""检查本机能否连上 rosbridge（通常需先 SSH -L 9090:127.0.0.1:9090）。"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from x3_nav.bridge import ForbiddenPublishError, RosBridge
from x3_nav.config import load_config


def main() -> int:
    cfg = load_config()
    host = cfg["robot"]["host"]
    port = cfg["robot"]["rosbridge_port"]
    print(f"连接 rosbridge {host}:{port} ...")
    try:
        with RosBridge(cfg) as bridge:
            print("连通成功:", bridge.is_connected)
            topics = bridge.get_topics()
            if topics:
                print(f"rosapi 话题数: {len(topics)}")
            else:
                print("提示: 未取到 /rosapi/topics（可忽略，不影响订阅）")
            try:
                bridge.assert_allowed_publish("/cmd_vel")
                print("错误：红线未生效")
                return 2
            except ForbiddenPublishError as e:
                print("红线自检通过:", e)
        return 0
    except Exception as e:
        print("连通失败:", e)
        print("提示: 先 SSH 端口转发，并确认车上 rosbridge 已启动")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
