set -e
TASK_DIR=/home/jetson/rosmaster_x3_runtime/obstacle1_20261121_5cm
python3 - <<'PY'
import json, time
path='/home/jetson/rosmaster_x3_runtime/obstacle1_20261121_5cm/latest_status.json'
try:
    row=json.load(open(path))
    if row.get('scan',{}).get('front_points'):
        row['scan']['front_points']=row['scan']['front_points'][:3]
    row['observation_age_sec']=time.time()-row['time']
    print(json.dumps(row,indent=2))
except Exception as error:
    print(error)
PY
ps -eo pid,args | grep -E 'car_observe.py|target_task_manager|rgbd_target_node' | grep -v grep
tail -n 4 "$TASK_DIR/observer.log"
