"""Isolated vendor motor diagnostic, bounded 0.6 s pulse with independent zero writer."""
import json
import multiprocessing
import sys
import time
from pathlib import Path
import serial

def frame(values):
    data=[0xff,0xfc,7,0x10]+[v & 255 for v in values]
    return bytearray(data+[(sum(data)+5)&255])
zero=frame((0,0,0,0))
link=serial.Serial('/dev/ttyUSB1',115200,timeout=.05,write_timeout=.2)
def independent_stop():
    time.sleep(.75)
    for _ in range(30):
        link.write(zero);time.sleep(.05)
    link.close()
report={'mode':'isolated_vendor_motor_diagnostic','pwm_percent':40,'pulse_seconds':.6}
stopper=None
try:
    link.write(zero)
    time.sleep(2)
    link.reset_input_buffer()
    link.write(bytearray.fromhex('fffc05505100a6'))
    received=bytearray()
    deadline=time.monotonic()+3
    while time.monotonic()<deadline:
        received.extend(link.read(256))
        if b'\xff\xfb\x05\x51' in received: break
    report['version_reply_verified']=b'\xff\xfb\x05\x51' in received
    if not report['version_reply_verified']:
        raise RuntimeError('No version reply; motor pulse refused')
    stopper=multiprocessing.get_context('fork').Process(target=independent_stop)
    stopper.start()
    start=time.monotonic()
    link.write(frame((40,40,40,40)))
    received=bytearray()
    while time.monotonic()-start<.6:
        received.extend(link.read(256))
    report['elapsed_until_zero_sec']=time.monotonic()-start
    link.write(zero)
    report['received_hex']=received.hex()
finally:
    for _ in range(5):
        link.write(zero);time.sleep(.05)
    link.close()
    if stopper:
        stopper.join(timeout=5)
        report['independent_stop_exit']=stopper.exitcode
    path=Path.home()/'rosmaster_x3_deploy/logs/motor_pulse.json'
    path.write_text(json.dumps(report,indent=2))
    print(json.dumps(report))
