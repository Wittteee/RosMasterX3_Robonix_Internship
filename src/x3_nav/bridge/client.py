"""与车上 rosbridge 的安全桥接：只允许订阅状态 + 发布 goal/cancel。"""

from __future__ import annotations

import time
from typing import Any, Callable

import roslibpy

from x3_nav.config import load_config


class ForbiddenPublishError(RuntimeError):
    """试图发布被红线禁止的话题。"""


class RosBridge:
    """roslibpy 封装；强制禁止 /cmd_vel 等底层速度话题。"""

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        self.config = config or load_config()
        robot = self.config["robot"]
        self.host = str(robot["host"])
        self.port = int(robot["rosbridge_port"])
        self.forbidden = {str(t).rstrip("/") for t in self.config["safety"].get("forbidden_publish", [])}
        self._client: roslibpy.Ros | None = None
        self._advertised: dict[str, roslibpy.Topic] = {}

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
        self._client.run()
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._client.is_connected:
                return
            time.sleep(0.1)
        self.close()
        raise TimeoutError(f"无法连接 rosbridge {self.host}:{self.port}（检查 SSH 隧道与车上 rosbridge）")

    def close(self) -> None:
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
        key = topic.rstrip("/")
        if key in self.forbidden or topic in self.forbidden:
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
        return t

    def publish(self, topic: str, msg_type: str, message: dict[str, Any], *, keep_advertised: bool = False) -> None:
        self.assert_allowed_publish(topic)
        if topic in self._advertised:
            pub = self._advertised[topic]
        else:
            pub = roslibpy.Topic(self.client, topic, msg_type)
            pub.advertise()
            if keep_advertised:
                self._advertised[topic] = pub
        pub.publish(roslibpy.Message(message))
        if not keep_advertised and topic not in self._advertised:
            # 给 rosbridge 一点时间发出去再取消 advertise
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
