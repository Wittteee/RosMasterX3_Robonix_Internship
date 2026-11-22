set -e
TASK=/home/jetson/rosmaster_x3_runtime/obstacle2_20261121
python3 - <<'PY'
import os,json,hashlib
root='/home/jetson/rosmaster_x3_runtime/obstacle2_20261121'
active=[]
for entry in os.listdir('/proc'):
 if not entry.isdigit():continue
 try:args=open('/proc/'+entry+'/cmdline','rb').read().split(b'\0')
 except IOError:continue
 if any(os.path.basename(arg) in (b'x3_base_driver.py',b'Mcnamu_driver.py',b'live_trial.py')for arg in args):active.append([entry,[a.decode('utf-8','replace')for a in args]])
assert not active,active
print('No chassis driver or trial supervisor remains active')
print(json.dumps(json.load(open(root+'/hardware_left_01/operator_stop.json')),indent=2))
PY
sha256sum -c "$TASK/function1_binary.sha256"
tar -czf "$TASK/hardware_left_01_evidence.tar.gz" -C "$TASK/hardware_left_01" .
echo 'Hardware evidence archived; no trial restarted'
