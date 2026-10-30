"""与车上 rosbridge 的安全桥接：只允许订阅状态 + 发布 goal/cancel。"""

from __future__ import annotations

import time
from typing import Any, Callable

import roslibpy

from x3_nav.config import load_config, validate_config, normalize_topic


class ForbiddenPublishError(RuntimeError):
    """试图发布被红线禁止的话题。"""


class RosBridge:
    """roslibpy 封装；强制禁止 /cmd_vel 等底层速度话题。"""

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        self.config = validate_config(config) if config is not None else load_config()
        robot = self.config["robot"]
        self.host = str(robot["host"])
        self.port = int(robot["rosbridge_port"])
        self.forbidden = {normalize_topic(t) for t in self.config["safety"]["forbidden_publish"]}
        self.allowed = {normalize_topic(self.config["topics"][k]) for k in ("move_base_goal", "move_base_cancel")}
        self._client: roslibpy.Ros | None = None
        self._advertised: dict[str, roslibpy.Topic] = {}
        self._subscriptions = []

    @property
    def client(self) -> roslibpy.Ros:
        if self._client is None:
            raise RuntimeError("尚未 connect()")
        return self._client

    @property
    def is_connected(self) -> bool:
        return bool(self._client is not None and self._client.is_connected)

    def connect(self, timeout: float = 10.0) -> None:
        if self._client is not None and self._client.is_connected:
            return
        self._client = roslibpy.Ros(host=self.host, port=self.port)
        try:
            self._client.run(timeout=timeout)
            if not self._client.is_connected:
                raise TimeoutError("Transport did not connect")
        except Exception as exc:
            self.close()
            raise TimeoutError(f"无法连接 rosbridge {self.host}:{self.port}（检查 SSH 隧道与车上 rosbridge）") from exc

    def close(self) -> None:
        for topic in self._subscriptions:
            try:
                topic.unsubscribe()
            except Exception:
                pass
        self._subscriptions.clear()
        for topic in list(self._advertised.values()):
            try:
                topic.unadvertise()
            except Exception:
                pass
        self._advertised.clear()
        if self._client is not None:
            try:
                self._client.terminate()
            except Exception:
                pass
            self._client = None

    def __enter__(self) -> "RosBridge":
        self.connect()
        return self

    def __exit__(self, *args: object) -> None:
        self.close()

    def assert_allowed_publish(self, topic: str) -> None:
        key = normalize_topic(topic)
        if key == "cmd_vel" or key.split("/")[-1] == "cmd_vel" or key in self.forbidden or key not in self.allowed:
            raise ForbiddenPublishError(
                f"红线禁止发布 {topic}。Provider 只能发导航目标/取消，不能发底层速度。"
            )

    def subscribe(
        self,
        topic: str,
        msg_type: str,
        callback: Callable[[dict[str, Any]], None],
    ) -> roslibpy.Topic:
        t = roslibpy.Topic(self.client, topic, msg_type)
        t.subscribe(callback)
        self._subscriptions.append(t)
        return t

    def publish(self, topic: str, msg_type: str, message: dict[str, Any], *, keep_advertised: bool = False) -> None:
        self.assert_allowed_publish(topic)
        topic = "/" + normalize_topic(topic)
        expected_type = ("geometry_msgs/PoseStamped" if topic == self.config["topics"]["move_base_goal"] else "actionlib_msgs/GoalID")
        if msg_type != expected_type:
            raise ForbiddenPublishError("Unexpected message type for controlled topic")
        if topic in self._advertised:
            pub = self._advertised[topic]
        else:
            pub = roslibpy.Topic(self.client, topic, msg_type)
            pub.advertise()
            if keep_advertised:
                self._advertised[topic] = pub
        try:
            pub.publish(roslibpy.Message(message))
        finally:
            if not keep_advertised and topic not in self._advertised:
                time.sleep(0.05)
                try:
                    pub.unadvertise()
                except Exception:
                    pass

    def get_topics(self) -> list[str]:
        """拉取当前 ROS 话题列表（需要 rosapi）。失败则返回空列表。"""
        result: list[str] = []
        done = {"ok": False}

        def _cb(msg: dict[str, Any]) -> None:
            topics = msg.get("topics") or []
            result.extend(str(t) for t in topics)
            done["ok"] = True

        try:
            service = roslibpy.Service(self.client, "/rosapi/topics", "rosapi/Topics")
            service.call(roslibpy.ServiceRequest({}), _cb, _cb)
            deadline = time.time() + 5.0
            while time.time() < deadline and not done["ok"]:
                time.sleep(0.05)
        except Exception:
            return []
        return result
