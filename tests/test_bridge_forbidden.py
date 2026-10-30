from unittest.mock import Mock, patch
import pytest
from x3_nav.bridge import RosBridge, ForbiddenPublishError
from x3_nav.config import load_config


@pytest.mark.parametrize("topic", ["cmd_vel", "/cmd_vel", "/cmd_vel/", "//cmd_vel//", "/robot/cmd_vel", "/other"])
def test_publish_blocked_before_transport(topic):
    bridge = RosBridge(load_config())
    with patch("x3_nav.bridge.client.roslibpy.Topic") as topic_class:
        with pytest.raises(ForbiddenPublishError):
            bridge.publish(topic, "geometry_msgs/Twist", {})
        topic_class.assert_not_called()


def test_configuration_cannot_remap_goal_to_velocity():
    cfg = load_config()
    cfg["topics"]["move_base_goal"] = "/cmd_vel/"
    with pytest.raises(ValueError):
        RosBridge(cfg)


def test_publish_cleanup_even_on_error():
    b = RosBridge()
    b._client = Mock()
    with patch("x3_nav.bridge.client.roslibpy.Topic") as factory:
        pub = factory.return_value
        pub.publish.side_effect = RuntimeError("lost")
        with pytest.raises(RuntimeError):
            b.publish("/move_base_simple/goal", "geometry_msgs/PoseStamped", {})
        pub.unadvertise.assert_called_once()


def test_connection_timeout_passed_to_transport():
    b = RosBridge()
    with patch("x3_nav.bridge.client.roslibpy.Ros") as factory:
        factory.return_value.run.side_effect = TimeoutError()
        with pytest.raises(TimeoutError, match="SSH"):
            b.connect(timeout=.1)
        factory.return_value.run.assert_called_once_with(timeout=.1)
        factory.return_value.terminate.assert_called_once()


def test_rosapi_failure_returns_empty():
    b = RosBridge()
    b._client = Mock()
    with patch("x3_nav.bridge.client.roslibpy.Service", side_effect=RuntimeError("missing")):
        assert b.get_topics() == []
