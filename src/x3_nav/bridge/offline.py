"""Deterministic in-memory ROS substitute; never opens a socket."""
import math
from types import SimpleNamespace
from x3_nav.bridge.client import RosBridge


class SimClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class OfflineBridge(RosBridge):
    def __init__(self, config, clock=None, *, missing=(), outcome=3):
        super().__init__(config)
        self.clock = clock or SimClock()
        self.missing = set(missing)
        self.outcome = outcome
        self.callbacks = {}
        self.publications = []
        self.xy = (0.0, 0.0)
        self.goal = None
        self.code = None
        self.connected = False
        self.closed = False

    @property
    def is_connected(self):
        return self.connected

    def connect(self, timeout=10):
        self.connected = True

    def close(self):
        self.connected = False
        self.closed = True
        self.callbacks.clear()

    def subscribe(self, topic, msg_type, callback):
        self.callbacks.setdefault(topic, []).append(callback)
        self._emit_sensors()
        def unsubscribe():
            if callback in self.callbacks.get(topic, []):
                self.callbacks[topic].remove(callback)
        return SimpleNamespace(unsubscribe=unsubscribe)

    def emit(self, key, msg):
        if key not in self.missing:
            for cb in list(self.callbacks.get(self.config['topics'][key], [])):
                cb(msg)

    def _emit_sensors(self):
        for source in ('amcl_pose', 'odom'):
            frame = self.config['mission']['frame_id'] if source == 'amcl_pose' else 'odom'
            msg = dict(header=dict(frame_id=frame), pose=dict(pose=dict(
                position=dict(x=self.xy[0], y=self.xy[1]),
                orientation=dict(x=0, y=0, z=0, w=1))))
            self.emit(source, msg)
        self.emit('scan', dict(ranges=[1.0]))
        self.emit('move_base_status', dict(status_list=[] if self.code is None else [
            dict(goal_id=dict(id='offline-goal-1'), status=self.code, text='simulation only')]))

    def publish(self, topic, msg_type, message, **kwargs):
        self.assert_allowed_publish(topic)
        self.publications.append(dict(topic=topic, type=msg_type, message=message))
        if topic == self.config['topics']['move_base_goal']:
            p = message['pose']['position']
            self.goal = p['x'], p['y']
            self.code = 1
        else:
            self.code = 2 if self.goal is not None else None
        self._emit_sensors()

    def sleep(self, duration):
        self.clock.now += duration
        if self.code == 1 and self.outcome is not None:
            dx, dy = self.goal[0]-self.xy[0], self.goal[1]-self.xy[1]
            distance = math.hypot(dx, dy)
            step = min(.04, distance)
            if distance:
                self.xy = (self.xy[0] + step*dx/distance, self.xy[1] + step*dy/distance)
            if distance <= .04 + 1e-9:
                self.code = self.outcome
        self._emit_sensors()
