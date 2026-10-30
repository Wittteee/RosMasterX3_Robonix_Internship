import copy
import pytest
from x3_nav.config import load_config, validate_config


@pytest.mark.parametrize("section", ["robot", "topics", "safety", "mission"])
def test_missing_section(section):
    cfg = load_config()
    del cfg[section]
    with pytest.raises(KeyError):
        validate_config(cfg)


@pytest.mark.parametrize("name", ["cmd_vel", "/cmd_vel", "/cmd_vel/"])
def test_blacklist_normalization(name):
    cfg = load_config()
    cfg["safety"]["forbidden_publish"] = [name]
    validate_config(cfg)


@pytest.mark.parametrize("change", [
    ("robot", "rosbridge_port", True), ("robot", "host", ""),
    ("safety", "forbidden_publish", []), ("safety", "candidate_distance_m", float("nan")),
    ("safety", "candidate_distance_m", .81), ("mission", "token_ttl_sec", 0),
    ("mission", "one_shot_token", False), ("topics", "move_base_goal", "/robot/cmd_vel/"),
    ("vision", "min_confidence", 2), ("vision", "max_approach_m", .1),
])
def test_bad_values(change):
    cfg = load_config()
    cfg[change[0]][change[1]] = change[2]
    with pytest.raises(ValueError):
        validate_config(cfg)


def test_required_topics_and_vision_defaults():
    cfg = load_config()
    del cfg["vision"]
    original = copy.deepcopy(cfg)
    assert validate_config(cfg)["vision"]["target_label"] == "person"
    assert cfg == original
    del cfg["topics"]["scan"]
    with pytest.raises(KeyError):
        validate_config(cfg)
