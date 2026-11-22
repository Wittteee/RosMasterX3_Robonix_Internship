set -e
python3 - <<'PY'
import yaml,json
data=yaml.safe_load(open('/home/jetson/yahboomcar_ws/src/yahboomcar_bringup/param/robot_localization.yaml'))
print(json.dumps(dict((k,v)for k,v in data.items()if k.startswith(('imu','odom','base','world'))),indent=2))
PY
