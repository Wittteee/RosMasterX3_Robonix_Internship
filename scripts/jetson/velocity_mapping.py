"""Body-frame metric commands passed to the vendor API without hidden gains."""
import math
import time

if hasattr(time,'monotonic'):
    monotonic=time.monotonic
else:
    import ctypes
    class _Timespec(ctypes.Structure):
        _fields_=[('seconds',ctypes.c_long),('nanoseconds',ctypes.c_long)]
    _clock=ctypes.CDLL(None,use_errno=True).clock_gettime
    _clock.argtypes=[ctypes.c_int,ctypes.POINTER(_Timespec)]
    _clock.restype=ctypes.c_int
    def monotonic():
        stamp=_Timespec()
        if _clock(1,ctypes.byref(stamp))!=0:
            raise RuntimeError('CLOCK_MONOTONIC unavailable')
        return stamp.seconds+stamp.nanoseconds/1e9

def map_velocity(vx,vy,wz,max_linear=.1,max_angular=.2):
    values=[float(v) for v in (vx,vy,wz,max_linear,max_angular)]
    vx,vy,wz,max_linear,max_angular=values
    if any(math.isnan(v) or math.isinf(v) for v in values) or max_linear<=0 or max_angular<=0:
        raise ValueError('Invalid velocity or command limits')
    vx,vy,wz=values[:3]
    norm=math.hypot(vx,vy)
    scale=min(1.,max_linear/norm) if norm else 1.
    return vx*scale,vy*scale,max(-max_angular,min(max_angular,wz))

def command_expired(last_command,now,timeout=.4):
    return last_command is not None and (now-last_command>timeout or now<last_command)


class StraightHeadingHold(object):
    """Hold the initial odom yaw for one uninterrupted forward command."""
    max_age_sec = .25

    def __init__(self):
        self.sample = None
        self.target_yaw = None

    def observe(self, orientation, sample_time):
        x, y, z, w = orientation
        norm_squared = x*x + y*y + z*z + w*w
        if math.isnan(norm_squared) or math.isinf(norm_squared) or norm_squared < 1e-12:
            self.sample = None
            return
        yaw = math.atan2(2*(w*z + x*y), norm_squared - 2*(y*y + z*z))
        self.sample = (yaw, sample_time)

    def has_fresh_heading(self, now):
        return self.sample is not None and 0 <= now - self.sample[1] <= self.max_age_sec

    def reset(self):
        self.target_yaw = None

    def correct(self, command, now, max_angular):
        vx, vy, wz = command
        if vx <= 0 or vy != 0 or wz != 0:
            self.reset()
            return command
        if not self.has_fresh_heading(now):
            self.reset()
            raise ValueError('Fresh odom heading is required')
        yaw = self.sample[0]
        if self.target_yaw is None:
            self.target_yaw = yaw
        error = self.target_yaw - yaw
        error = math.atan2(math.sin(error), math.cos(error))
        limit = min(.08, max_angular)
        return vx, vy, max(-limit, min(limit, .8 * error))
