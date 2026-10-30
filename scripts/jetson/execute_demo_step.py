"""One human-authorized 0.10 m forward goal through the reference Provider."""
import importlib.util
import json
import math
import os
import threading
import time
from pathlib import Path
import roslibpy

raise RuntimeError('Experimental execution disabled: watchdog timing and telemetry require correction; no automatic retry authorized.')

root = Path.home() / 'rosmaster_x3_deploy'
spec = importlib.util.spec_from_file_location('reference_cli', str(root/'scripts/move_base_cli.py'))
cli = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cli)
os.environ['X3_MOVE_OPTIONS_MIN_RADIUS_M'] = '0.1'
os.environ['X3_MOVE_OPTIONS_MAX_RADIUS_M'] = '0.1'
api = cli.ProviderApi()
report = {'authorized_distance_m':0.1, 'goal_sent':False}
samples = []
subscription = None
timer = None
try:
    preview = api.preview(5)
    report['preview'] = preview
    if not preview.get('ok'):
        raise RuntimeError('preview_rejected')
    options = [o for o in preview.get('options', [])
               if abs(float(o['radius_m'])-0.1) < 0.00001
               and abs(float(o['relative_angle_deg'])) < 0.1]
    if len(options) != 1:
        raise RuntimeError('no_unique_forward_10cm_option')
    bridge = api._provider.ensure_bridge()
    def observe(msg):
        pos = msg['pose']['pose']['position']
        samples.append((time.monotonic(), float(pos['x']), float(pos['y'])))
    subscription = roslibpy.Topic(bridge.ros, '/odom', 'nav_msgs/Odometry')
    subscription.subscribe(observe)
    limit = time.monotonic()+4
    while not samples and time.monotonic()<limit:
        time.sleep(0.05)
    if not samples:
        raise RuntimeError('odom_missing')
    prepared = api.prepare(preview['session_id'], options[0]['option_id'])
    report['prepare_ok'] = prepared.get('ok')
    if not prepared.get('ok'):
        raise RuntimeError('prepare_rejected: '+str(prepared.get('reason')))
    start = samples[-1]
    aborted = threading.Event()
    def watchdog():
        while not aborted.wait(0.05):
            latest = samples[-1]
            distance = math.hypot(latest[1]-start[1], latest[2]-start[2])
            if time.monotonic()-start[0]>9 or time.monotonic()-latest[0]>1 or distance>0.15:
                report['watchdog_cancel'] = True
                api.cancel()
                return
    threading.Thread(target=watchdog, daemon=True).start()
    report['goal_sent'] = True
    try:
        with cli.prepared_execution_gate():
            report['result'] = api.execute(prepared['token'], prepared['confirmation_text'])
    finally:
        aborted.set()
    time.sleep(0.5)
    end = samples[-1]
    report['odom_displacement_m'] = math.hypot(end[1]-start[1],end[2]-start[2])
    report['physical_arrival_verified'] = False
except Exception as exc:
    report['error'] = str(exc)
finally:
    if report.get('goal_sent'):
        report['cancel_after_test'] = api.cancel()
    if subscription:
        subscription.unsubscribe()
    bridge = getattr(api._provider, 'bridge', None)
    if bridge:
        bridge.ros.terminate()
        api._provider.bridge = None
    path = root/'logs/demo_step_20261028.json'
    path.write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps(report,ensure_ascii=False,indent=2))
