"""One real centered QR triggers the validated bounded direct base movement."""
import fcntl
import importlib.util
import json
import multiprocessing
import sys
import time
from pathlib import Path
from Rosmaster_Lib import Rosmaster

root=Path.home()/'rosmaster_x3_deploy'
spec=importlib.util.spec_from_file_location('camera_observation',str(root/'demo_project/src/x3_nav/vision/camera_observation.py'))
observation=importlib.util.module_from_spec(spec)
spec.loader.exec_module(observation)
StableQrTrigger=observation.StableQrTrigger
centered_qr_bearing=observation.centered_qr_bearing

def read_observation():
    return json.loads((root/'logs/camera_qr.json').read_text())

def run():
    lock=open('/tmp/x3_qr_auto_move.lock','w')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    report={'mode':'real_qr_direct_base_demo','label':'X3_TARGET_A','motion_started':False,
            'uses_map':False,'speed_mps':.12,'maximum_duration_sec':4.0,'events':[]}
    bot=None
    stopper=None
    zero=bytearray([255,252,7,16,0,0,0,0,23])
    def stop_independently():
        time.sleep(4.3)
        for _ in range(40):
            bot.ser.write(zero)
            time.sleep(.05)
    def record(phase,**details):
        report['events'].append(dict(phase=phase,timestamp=time.time(),**details))
        print(phase,flush=True)
    try:
        record('WAITING_FOR_REAL_QR')
        gate=StableQrTrigger(3)
        deadline=time.monotonic()+45
        while time.monotonic()<deadline:
            try:
                data=read_observation()
                if gate.update(data):
                    report['trigger_observation']=data
                    break
            except (ValueError,OSError,KeyError,TypeError):
                gate.count=0
            time.sleep(.05)
        else:
            raise RuntimeError('No stable centered real QR within 45 seconds')
        record('QR_CONFIRMED')
        bot=Rosmaster(car_type=1,com='/dev/ttyUSB1')
        bot.ser.write_timeout=.2
        bot.set_car_type(1)
        bot.create_receive_threading()
        time.sleep(1)
        if bot.get_version()<=0:
            raise RuntimeError('Control board not responding')
        centered_qr_bearing(root/'logs/camera_qr.json')
        stopper=multiprocessing.get_context('fork').Process(target=stop_independently)
        stopper.start()
        record('MOVING')
        report['motion_started']=True
        start=time.monotonic()
        # Detection is a one-shot start condition, not a continuous-follow task.
        while time.monotonic()-start<4.0:
            bot.set_car_motion(.12,0,0)
            time.sleep(.05)
        report['motion_elapsed_sec']=time.monotonic()-start
        record('TIME_COMPLETED')
    except (Exception,KeyboardInterrupt) as exc:
        report['error']=str(exc)
        record('REJECTED_OR_INTERRUPTED')
    finally:
        if bot:
            for _ in range(10):
                bot.set_car_motion(0,0,0)
                bot.ser.write(zero)
                time.sleep(.05)
            report['stop_commands_sent']=True
            if stopper:
                stopper.join(8)
                report['independent_stop_exit']=stopper.exitcode
            bot.ser.close()
            record('STOP_SENT')
        report['physical_completion_verified']=False
        (root/'logs/qr_auto_move.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print(json.dumps(report,ensure_ascii=False),flush=True)
        lock.close()

if __name__=='__main__':
    run()
