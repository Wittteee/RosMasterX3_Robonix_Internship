"""Use a fresh centered QR observation as a forward-action trigger."""
import json
import math
import time
from pathlib import Path

def centered_qr_bearing(path, label='X3_TARGET_A', now=None):
    data=json.loads(Path(path).read_text(encoding='utf-8'))
    return validate_centered_qr(data,label,now)

def validate_centered_qr(data,label='X3_TARGET_A',now=None):
    now=time.time() if now is None else now
    stamp=float(data['timestamp'])
    if not math.isfinite(stamp) or not -.2 <= now-stamp <= 1.0:
        raise ValueError('camera_observation_stale')
    if data.get('source')!='real_camera' or data.get('frame_ok') is not True:
        raise ValueError('real_camera_unavailable')
    targets=[target for target in data.get('targets',[]) if target.get('label')==label]
    if len(targets)!=1:
        raise ValueError('target_not_unique_or_missing')
    width=float(targets[0]['image_width']); center=float(targets[0]['center_x'])
    if not math.isfinite(width) or width<=0 or not math.isfinite(center) or not 0<=center<width:
        raise ValueError('invalid_image_geometry')
    if abs(center/width-.5)>.1:
        raise ValueError('target_not_centered')
    return 0.0

class StableQrTrigger:
    def __init__(self,required_frames=3):
        self.required_frames=required_frames
        self.count=0
        self.last_stamp=None
        self.used=False

    def update(self,data,now=None):
        if self.used:
            return False
        try:
            validate_centered_qr(data,now=now)
        except (ValueError,KeyError,TypeError):
            self.count=0
            self.last_stamp=None
            return False
        stamp=float(data['timestamp'])
        if self.last_stamp is not None and stamp<=self.last_stamp:
            return False
        self.last_stamp=stamp
        self.count+=1
        if self.count>=self.required_frames:
            self.used=True
            return True
        return False
