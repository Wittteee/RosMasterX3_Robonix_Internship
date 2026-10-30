#!/usr/bin/env bash
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
rosnode list
timeout 5 rostopic echo -n 1 /vel_raw
timeout 5 rostopic echo -n 1 /imu/imu_raw
python3 - <<'PY'
import cv2,json
cap=cv2.VideoCapture(0)
ok,frame=cap.read()
print(json.dumps({'camera_open':cap.isOpened(),'frame_ok':ok,'shape':list(frame.shape) if ok else None}))
if ok: cv2.imwrite('/home/jetson/rosmaster_x3_deploy/logs/camera_current.jpg',frame)
cap.release()
PY
