from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any


def ensure_log_dir(root: Path) -> Path:
    log_dir = root / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    return log_dir


def write_json_log(root: Path, name: str, payload: dict[str, Any]) -> Path:
    log_dir = ensure_log_dir(root)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    path = log_dir / f"{stamp}_{name}.json"
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    return path
