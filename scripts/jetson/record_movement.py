import cv2
import time
from pathlib import Path
root=Path.home()/'rosmaster_x3_deploy/logs'
cap=cv2.VideoCapture(0)
writer=None
deadline=time.monotonic()+100
try:
    while time.monotonic()<deadline:
        ok,frame=cap.read()
        if not ok: break
        if writer is None:
            cv2.imwrite(str(root/'movement_before.jpg'),frame)
            writer=cv2.VideoWriter(str(root/'movement.avi'),cv2.VideoWriter_fourcc(*'MJPG'),10,(frame.shape[1],frame.shape[0]))
        writer.write(frame)
        cv2.imwrite(str(root/'movement_latest.jpg'),frame)
        time.sleep(.1)
finally:
    cap.release()
    if writer:writer.release()
