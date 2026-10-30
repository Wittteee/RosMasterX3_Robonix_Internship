"""Keep coordinate frames separate; AMCL wins only when fresh and in-frame."""
import math
import threading
import time
from dataclasses import dataclass
from x3_nav.nav.client import pose_xy_from_amcl


@dataclass(frozen=True)
class RobotPose:
    x: float
    y: float
    yaw: float
    frame_id: str
    received_at: float
    source: str

    @property
    def xy(self):
        return self.x, self.y


class PoseStore:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.values = {}
        self.latest = None
        self.lock = threading.Lock()

    def update(self, source, msg):
        try:
            xy = pose_xy_from_amcl(msg)
            q = msg['pose']['pose']['orientation']
            frame = msg['header']['frame_id']
            a, b, c, d = (float(q[k]) for k in ('x', 'y', 'z', 'w'))
            norm = math.sqrt(a*a + b*b + c*c + d*d)
            if xy is None or not frame or not math.isfinite(norm) or norm < 1e-9:
                return
            a, b, c, d = (v/norm for v in (a, b, c, d))
            yaw = math.atan2(2*(d*c+a*b), 1-2*(b*b+c*c))
            pose = RobotPose(*xy, yaw, frame, self.clock(), source)
        except (KeyError, TypeError, ValueError, OverflowError):
            return
        with self.lock:
            self.values[source] = pose
            self.latest = pose

    def get(self, frame_id, max_age):
        with self.lock:
            for source in ('amcl', 'odom'):
                pose = self.values.get(source)
                if pose and pose.frame_id == frame_id and 0 <= self.clock()-pose.received_at <= max_age:
                    return pose
        return None
