"""Navigate to a safe parking candidate derived from the user-confirmed target."""
import importlib.util
import json
import math
import os
import sys
import time
from pathlib import Path
root=Path.home()/'rosmaster_x3_deploy'
def load(name,path):
    spec=importlib.util.spec_from_file_location(name,str(path))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module
cli=load('reference_cli',root/'scripts/move_base_cli.py')
geometry=load('parking_goal',root/'scripts/parking_goal.py')
policy=load('nav_calibration_policy',root/'scripts/nav_calibration_policy.py')
policy.require_calibration(json.loads((root/'scripts/chassis_nav_calibration.json').read_text()))
resume='--resume' in sys.argv
previous=json.loads((root/'logs/target_parking.json').read_text()) if resume else None
radius='0.1' if resume else '0.3'
os.environ['X3_MOVE_OPTIONS_MIN_RADIUS_M']=radius
os.environ['X3_MOVE_OPTIONS_MAX_RADIUS_M']=radius
api=cli.ProviderApi()
report={'mode':'official_target_parking','target_source':'user_confirmed_distance',
        'target_distance_m':.6,'standoff_m':.3,'execution_attempted':False,'arrival_verified':False}
try:
    preview=api.preview(5)
    report['preview']=preview
    if preview.get('ok') is not True: raise RuntimeError('preview_rejected: '+str(preview.get('reason')))
    if resume:
        goal=previous['geometry']
        pose=preview['current_pose']
        remaining=math.hypot(pose['x']-goal['parking']['x'],pose['y']-goal['parking']['y'])
        if not .05<=remaining<=.12: raise RuntimeError('remaining_distance_requires_new_plan')
        goal=dict(goal,travel_m=remaining,search_radius_m=.1)
        report['resumes_original_parking_point']=True
    else:
        goal=geometry.target_parking_pose(preview['current_pose'],.6,.3)
    report['geometry']=goal
    selected=geometry.choose_parking_candidate(preview,goal)
    report['selected_candidate']=selected
    prepared=api.prepare(preview['session_id'],selected['option_id'])
    report['prepare_ok']=prepared.get('ok')
    if prepared.get('ok') is not True: raise RuntimeError('prepare_rejected: '+str(prepared.get('reason')))
    report['prepared_target']=prepared.get('target')
    guard=json.loads((root/'logs/guard_30cm.json').read_text())
    if not guard['ready'] or guard['goal'] is not None or time.monotonic()-guard['heartbeat']>.5:
        raise RuntimeError('independent_guard_not_ready')
    report['execution_attempted']=True
    with cli.prepared_execution_gate():
        outcome=api.execute(prepared['token'],prepared['confirmation_text'])
    report['result']=outcome
    report['arrival_verified']=(outcome.get('ok') is True and outcome.get('arrived') is True
                                and outcome.get('move_base_status_name')=='SUCCEEDED')
except Exception as exc:
    report['error']=str(exc)
finally:
    if report['execution_attempted']: report['cancel']=api.cancel()
    report['guard']=json.loads((root/'logs/guard_30cm.json').read_text())
    report['physical_completion_verified']=False
    filename='target_parking_resume.json' if resume else 'target_parking.json'
    (root/'logs'/filename).write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k not in ('preview','result')},ensure_ascii=False))
    if 'result' in report: print(json.dumps({k:report['result'].get(k) for k in ('ok','reason','arrived','move_base_status_name','message','odom_observation','cmd_vel_observation')},ensure_ascii=False))
    bridge=getattr(api._provider,'bridge',None)
    if bridge: bridge.ros.terminate()
