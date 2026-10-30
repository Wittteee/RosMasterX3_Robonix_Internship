#!/usr/bin/env bash
python3 - <<'PY'
import json,time
from pathlib import Path
path=Path.home()/'rosmaster_x3_deploy/logs/camera_qr.json'
for _ in range(8):
 data=json.loads(path.read_text())
 print(json.dumps({'age':time.time()-data['timestamp'],'observation':data}),flush=True)
 time.sleep(.3)
PY
tail -10 "$HOME/rosmaster_x3_deploy/logs/camera_qr.log"
