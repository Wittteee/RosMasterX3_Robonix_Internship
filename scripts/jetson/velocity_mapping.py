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
