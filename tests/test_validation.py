from unittest.mock import Mock
import pytest
from x3_nav.config import load_config
from x3_nav.validation import check_topic_rates, summarize_health


@pytest.mark.parametrize("missing", [None, "scan", "odom", "amcl_pose", "move_base_status"])
def test_counts_and_required_topics(missing):
    cfg = load_config()
    subscriptions = []
    def subscribe(topic, msg_type, callback):
        sub = Mock()
        subscriptions.append(sub)
        if missing is None or topic != cfg["topics"][missing]:
            callback({})
        return sub
    bridge = Mock(config=cfg, subscribe=subscribe)
    report = check_topic_rates(bridge, 2, sleep=lambda _: None)
    assert summarize_health(report)["ok"] == (missing in (None, "move_base_status"))
    assert report["scan"]["hz_est"] == (0 if missing == "scan" else .5)
    for sub in subscriptions:
        sub.unsubscribe.assert_called_once()


def test_partial_subscription_cleanup():
    first = Mock()
    bridge = Mock(config=load_config())
    bridge.subscribe.side_effect = [first, RuntimeError("lost")]
    with pytest.raises(RuntimeError):
        check_topic_rates(bridge, sleep=lambda _: None)
    first.unsubscribe.assert_called_once()
