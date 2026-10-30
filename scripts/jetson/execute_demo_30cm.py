"""Execute exactly one forward 30 cm reference candidate with an independent guard."""
import importlib.util
import json
import os
import time
from pathlib import Path
root=Path.home()/'rosmaster_x3_deploy'
spec=importlib.util.spec_from_file_location('reference_cli',str(root/'scripts/move_base_cli.py'))
cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)
os.environ['X3_MOVE_OPTIONS_MIN_RADIUS_M']='0.3'
os.environ['X3_MOVE_OPTIONS_MAX_RADIUS_M']='0.3'
api=cli.ProviderApi()
report={'requested_distance_m':0.3,'execution_attempted':False}
try:
    preview=api.preview(5)
    report['preview']=preview
    if not preview.get('ok'): raise RuntimeError('preview_rejected')
    options=[o for o in preview['options'] if abs(float(o['radius_m'])-.3)<1e-5 and abs(float(o['relative_angle_deg']))<.1]
    if len(options)!=1: raise RuntimeError('no_forward_30cm_candidate')
    prepared=api.prepare(preview['session_id'],options[0]['option_id'])
    if not prepared.get('ok'): raise RuntimeError('prepare_rejected: '+str(prepared.get('reason')))
    guard=json.loads((root/'logs/guard_30cm.json').read_text())
    if not guard['ready'] or guard['goal'] is not None or time.monotonic()-guard['heartbeat']>.5:
        raise RuntimeError('independent_guard_not_ready')
    report['execution_attempted']=True
    with cli.prepared_execution_gate():
        report['result']=api.execute(prepared['token'],prepared['confirmation_text'])
except Exception as exc:
    report['error']=str(exc)
finally:
    if report['execution_attempted']: report['cancel']=api.cancel()
    report['guard']=json.loads((root/'logs/guard_30cm.json').read_text())
    report['physical_arrival_verified']=False
    (root/'logs/demo_30cm.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k not in ('preview','result')},ensure_ascii=False))
    if 'result' in report: print(json.dumps({k:report['result'].get(k) for k in ('ok','reason','arrived','move_base_status_name','message','odom_observation','cmd_vel_observation')},ensure_ascii=False))
    bridge=getattr(api._provider,'bridge',None)
    if bridge: bridge.ros.terminate()
