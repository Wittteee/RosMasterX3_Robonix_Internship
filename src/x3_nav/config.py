from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "configs" / "robot.yaml"


def project_root() -> Path:
    return ROOT


def load_config(path: str | Path | None = None) -> dict[str, Any]:
    cfg_path = Path(path) if path else DEFAULT_CONFIG
    if not cfg_path.is_file():
        raise FileNotFoundError(f"找不到配置文件: {cfg_path}")
    with cfg_path.open(encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"配置无效: {cfg_path}")
    _validate_config(data)
    return data


def _validate_config(data: dict[str, Any]) -> None:
    for key in ("robot", "topics", "safety", "mission"):
        if key not in data:
            raise KeyError(f"配置缺少字段: {key}")
    for t in ("scan", "odom", "amcl_pose", "move_base_status", "move_base_goal", "move_base_cancel"):
        if t not in data["topics"]:
            raise KeyError(f"topics 缺少: {t}")
    forbidden = data["safety"].get("forbidden_publish") or []
    if "/cmd_vel" not in {x.rstrip("/") for x in forbidden} and "/cmd_vel" not in forbidden:
        # 允许写成带或不带斜杠；若完全没写则补一条提醒性校验失败
        raise ValueError("safety.forbidden_publish 必须包含 /cmd_vel（红线）")
