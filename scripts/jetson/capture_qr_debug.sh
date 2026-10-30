#!/usr/bin/env bash
set -eo pipefail
old=$(pgrep -f '^python3 /home/jetson/rosmaster_x3_deploy/scripts/watch_camera_qr.py$' || true)
if [ -n "$old" ]; then kill "$old"; sleep 1; fi
python3 - <<'PY'
import cv2,json
cap=cv2.VideoCapture(0)
try:
 for _ in range(8): ok,frame=cap.read()
 if not ok: raise RuntimeError('No camera frame')
 cv2.imwrite('/home/jetson/rosmaster_x3_deploy/logs/qr_debug.png',frame)
 detector=cv2.QRCodeDetector()
 found,labels,points,_=detector.detectAndDecodeMulti(frame)
 print(json.dumps({'found':found,'labels':list(labels),'points':points.tolist() if points is not None else None}))
finally: cap.release()
PY
