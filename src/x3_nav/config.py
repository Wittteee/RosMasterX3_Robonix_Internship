from __future__ import annotations

import copy
import math
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
    return validate_config(data)


TOPIC_KEYS = ("scan", "odom", "amcl_pose", "move_base_status", "move_base_goal", "move_base_cancel")
VISION_DEFAULTS = dict(target_label="person", min_confidence=0.5, min_approach_m=0.2,
                       max_approach_m=0.8, max_yaw_deg=30.0,
                       detections_topic="/darknet_ros/bounding_boxes")


def normalize_topic(topic: str) -> str:
    if not isinstance(topic, str) or not topic or topic != topic.strip():
        raise ValueError("Topic must be a nonempty string without whitespace")
    parts = topic.strip("/").split("/")
    if any(not part or not part.replace("_", "a").isascii() or not part.replace("_", "a").isalnum() or part[0].isdigit() for part in parts):
        raise ValueError("Invalid ROS topic: " + topic)
    return "/".join(parts)


def finite_number(value, name, minimum=None, maximum=None, *, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(name + " must be a finite number")
    if positive and value <= 0 or minimum is not None and value < minimum or maximum is not None and value > maximum:
        raise ValueError(name + " is outside its allowed range")
    return float(value)


def validate_config(data: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(data, dict):
        raise ValueError("Configuration must be a mapping")
    cfg = copy.deepcopy(data)
    for key in ("robot", "topics", "safety", "mission"):
        if key not in cfg:
            raise KeyError("Missing section: " + key)
        if not isinstance(cfg[key], dict):
            raise ValueError("Section must be a mapping: " + key)
    host = cfg["robot"]["host"]
    if not isinstance(host, str) or not host.strip() or any(c.isspace() for c in host):
        raise ValueError("robot.host must be a nonempty host name")
    port = cfg["robot"]["rosbridge_port"]
    if type(port) is not int or not 1 <= port <= 65535:
        raise ValueError("robot.rosbridge_port must be an integer in 1..65535")
    for name, default in (("edition_topic", "/edition"), ("voltage_topic", "/voltage")):
        cfg["robot"][name] = "/" + normalize_topic(cfg["robot"].get(name, default))
    topics = cfg["topics"]
    for key in TOPIC_KEYS:
        topics[key] = "/" + normalize_topic(topics[key])
    if len(set(topics[key] for key in TOPIC_KEYS)) != len(TOPIC_KEYS):
        raise ValueError("Configured topics must be distinct")
    safety = cfg["safety"]
    forbidden = safety["forbidden_publish"]
    if not isinstance(forbidden, list) or not forbidden:
        raise ValueError("safety.forbidden_publish must be a nonempty list")
    names = {normalize_topic(name) for name in forbidden}
    if "cmd_vel" not in names:
        raise ValueError("safety.forbidden_publish must contain cmd_vel")
    for key in ("move_base_goal", "move_base_cancel"):
        name = normalize_topic(topics[key])
        if name in names or name.split("/")[-1] == "cmd_vel":
            raise ValueError("Writable topic aliases a forbidden topic: " + topics[key])
    safety["candidate_distance_m"] = finite_number(safety["candidate_distance_m"], "candidate_distance_m", maximum=.8, positive=True)
    safety["preempt_trigger_displacement_m"] = finite_number(safety["preempt_trigger_displacement_m"], "preempt_trigger_displacement_m", maximum=safety["candidate_distance_m"], positive=True)
    mission = cfg["mission"]
    for key in ("token_ttl_sec", "default_timeout_sec"):
        mission[key] = finite_number(mission[key], key, positive=True)
    for key, default in (("health_duration_sec", 2.0), ("pose_max_age_sec", 2.0),
                         ("cancel_timeout_sec", 3.0), ("poll_interval_sec", .1)):
        mission[key] = finite_number(mission.get(key, default), key, positive=True)
    mission["preview_motion_tolerance_m"] = finite_number(mission.get("preview_motion_tolerance_m", .03), "preview_motion_tolerance_m", minimum=0, maximum=.08)
    if not isinstance(mission["frame_id"], str) or not mission["frame_id"].strip():
        raise ValueError("mission.frame_id is required")
    for key in ("require_human_confirm", "one_shot_token"):
        if mission.get(key, True) is not True:
            raise ValueError(key + " cannot be disabled")
    if not isinstance(cfg.get("vision", {}), dict):
        raise ValueError("vision must be a mapping")
    vision = dict(VISION_DEFAULTS, **cfg.get("vision", {}))
    if "max_bearing_deg" in vision and "max_yaw_deg" not in cfg.get("vision", {}):
        vision["max_yaw_deg"] = vision["max_bearing_deg"]
    if not isinstance(vision["target_label"], str) or not vision["target_label"].strip():
        raise ValueError("vision.target_label must be nonempty")
    finite_number(vision["min_confidence"], "min_confidence", 0, 1)
    finite_number(vision["min_approach_m"], "min_approach_m", positive=True)
    finite_number(vision["max_approach_m"], "max_approach_m", vision["min_approach_m"], .8)
    finite_number(vision["max_yaw_deg"], "max_yaw_deg", 0, 30)
    normalize_topic(vision["detections_topic"])
    cfg["vision"] = vision
    return cfg


def _validate_config(data):
    validate_config(data)
