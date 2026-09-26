"""采样关键话题，检查是否有数据（数据有效性初检）。"""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from x3_nav.bridge import RosBridge
from x3_nav.config import load_config, project_root
from x3_nav.logutil import write_json_log
from x3_nav.validation import check_topic_rates, summarize_health


def main() -> int:
    cfg = load_config()
    with RosBridge(cfg) as bridge:
        report = check_topic_rates(bridge, duration_sec=3.0)
    health = summarize_health(report)
    print(json.dumps(health, ensure_ascii=False, indent=2))
    path = write_json_log(project_root(), "topics", health)
    print("日志:", path)
    print("总体:", "PASS" if health["ok"] else "FAIL（部分话题无数据，请核对车上节点与话题名）")
    return 0 if health["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
