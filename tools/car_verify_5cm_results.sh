set -e
TASK=/home/jetson/rosmaster_x3_runtime/obstacle2_20261121
python3 - <<'PY'
import json,xml.etree.ElementTree as ET,os,time,hashlib
root='/home/jetson/rosmaster_x3_runtime/obstacle2_20261121'
base=root+'/build_ws/build/test_results/x3_target_navigation'
results={}
for scenario in ['right','left','full','no_path','side_blocked','dynamic','server_delayed','wrong_direction','brake_rebound']:
 p=base+'/rosunit-bypass_behavior_'+scenario+'.xml'
 e=ET.parse(p).getroot()
 results[scenario]={'cases':len(list(e.iter('testcase'))),'failures':len(list(e.iter('failure'))),'errors':len(list(e.iter('error')))}
 assert results[scenario]=={'cases':1,'failures':0,'errors':0},(scenario,results[scenario])
active=[]
for entry in os.listdir('/proc'):
 if not entry.isdigit():continue
 try:args=open('/proc/'+entry+'/cmdline','rb').read().split(b'\0')
 except IOError:continue
 if any(os.path.basename(a)in(b'x3_base_driver.py',b'Mcnamu_driver.py',b'live_trial.py')for a in args):active.append(entry)
assert not active,active
report={'time':time.time(),'hardware_running':False,'new_hardware_trial':False,'front_trigger_m':.05,
 'bottle_goal_assertions':'initial preview and final rejoin checked against same bottle point',
 'ros_scenarios':results,'candidate_binary_sha256':hashlib.sha256(open(root+'/build_ws/devel/lib/x3_target_navigation/target_task_manager','rb').read()).hexdigest()}
json.dump(report,open(root+'/five_cm_validation.json','w'),indent=2)
print(json.dumps(report,indent=2))
PY
sha256sum -c "$TASK/function1_binary.sha256"
