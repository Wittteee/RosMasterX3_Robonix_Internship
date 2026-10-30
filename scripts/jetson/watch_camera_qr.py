"""Read real camera frames and expose a timestamped QR observation."""
import cv2
import json
import time
from pathlib import Path
root=Path.home()/'rosmaster_x3_deploy/logs'
cap=cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
detector=cv2.QRCodeDetector()
deadline=time.monotonic()+180
try:
    while time.monotonic()<deadline:
        ok,frame=cap.read()
        result={'timestamp':time.time(),'source':'real_camera','frame_ok':ok,'targets':[]}
        if ok:
            found,texts,points,_=detector.detectAndDecodeMulti(frame)
            if not found or not any(texts):
                gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
                large=cv2.resize(gray,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC)
                label,corners,_=detector.detectAndDecode(large)
                if label and corners is not None:
                    found=True
                    texts=[label]
                    points=corners/2.0
            if found:
                for label,corners in zip(texts,points):
                    if label=='X3_TARGET_A':
                        result['targets'].append({'label':label,'center_x':float(corners[:,0].mean()),'image_width':int(frame.shape[1])})
            cv2.imwrite(str(root/'camera_qr_current.tmp.jpg'),frame)
            (root/'camera_qr_current.tmp.jpg').replace(root/'camera_qr_current.jpg')
        tmp=root/'camera_qr.tmp'
        tmp.write_text(json.dumps(result));tmp.replace(root/'camera_qr.json')
        time.sleep(.2)
finally:
    cap.release()
