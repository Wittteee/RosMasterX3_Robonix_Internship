"""Observe only a new goal accepted after dispatch, ignoring retained statuses."""
from __future__ import annotations
import threading

STATUS_NAMES = {0: 'PENDING', 1: 'ACTIVE', 2: 'PREEMPTED', 3: 'SUCCEEDED',
                4: 'ABORTED', 5: 'REJECTED', 6: 'PREEMPTING', 7: 'RECALLING',
                8: 'RECALLED', 9: 'LOST'}


class GoalStatusMonitor:
    def __init__(self):
        self._lock = threading.RLock()
        self._seen = set()
        self._armed = False
        self.goal_id = None
        self.last_status_code = None
        self.last_status_name = None
        self.last_text = ''
        self.ambiguous = False
        self.raw = None
        self._active = set()

    def arm(self):
        with self._lock:
            self._armed = True
            self._baseline = set(self._seen)
            self.goal_id = None
            self.last_status_code = self.last_status_name = None
            self.last_text = ''
            self.ambiguous = False

    def callback(self, msg):
        statuses = msg.get('status_list', [])
        if not isinstance(statuses, list):
            return
        valid = []
        for item in statuses:
            try:
                gid, code = item['goal_id']['id'], item['status']
                if not isinstance(gid, str) or not gid or type(code) is not int or code not in STATUS_NAMES:
                    continue
                valid.append((gid, code, str(item.get('text', ''))))
            except (TypeError, KeyError):
                continue
        with self._lock:
            self._active = {gid for gid, code, _ in valid if code in {0, 1, 6, 7}}
            self._seen.update(gid for gid, _, _ in valid)
            if not self._armed:
                return
            new_ids = {gid for gid, code, _ in valid if gid not in self._baseline and code in {0, 1}}
            if self.goal_id:
                if new_ids - {self.goal_id}:
                    self.ambiguous = True
            elif len(new_ids) == 1:
                self.goal_id = new_ids.pop()
            elif len(new_ids) > 1:
                self.ambiguous = True
            for gid, code, text in valid:
                if gid == self.goal_id and not self.ambiguous:
                    self.last_status_code = code
                    self.last_status_name = STATUS_NAMES[code]
                    self.last_text = text
            self.raw = msg

    @property
    def has_active_goal(self):
        with self._lock:
            return bool(self._active)

    def snapshot(self):
        with self._lock:
            return dict(status_code=self.last_status_code, status_name=self.last_status_name,
                        text=self.last_text, goal_id=self.goal_id, ambiguous=self.ambiguous)

    def is_terminal(self):
        return self.last_status_code in {2, 3, 4, 5, 8, 9}
