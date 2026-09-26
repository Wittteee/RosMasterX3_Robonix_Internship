"""move_base GoalStatus 解析。"""

from __future__ import annotations

import threading
from typing import Any

# actionlib_msgs/GoalStatus
STATUS_NAMES = {
    0: "PENDING",
    1: "ACTIVE",
    2: "PREEMPTED",
    3: "SUCCEEDED",
    4: "ABORTED",
    5: "REJECTED",
    6: "PREEMPTING",
    7: "RECALLING",
    8: "RECALLED",
    9: "LOST",
}


class GoalStatusMonitor:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.last_status_code: int | None = None
        self.last_status_name: str | None = None
        self.last_text: str = ""
        self.raw: dict[str, Any] | None = None

    def callback(self, msg: dict[str, Any]) -> None:
        statuses = msg.get("status_list") or []
        if not statuses:
            return
        item = statuses[-1]
        code = int(item.get("status", -1))
        with self._lock:
            self.last_status_code = code
            self.last_status_name = STATUS_NAMES.get(code, f"UNKNOWN({code})")
            self.last_text = str(item.get("text") or "")
            self.raw = msg

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "status_code": self.last_status_code,
                "status_name": self.last_status_name,
                "text": self.last_text,
            }

    def is_terminal(self) -> bool:
        return self.last_status_code in {2, 3, 4, 5, 8, 9}
