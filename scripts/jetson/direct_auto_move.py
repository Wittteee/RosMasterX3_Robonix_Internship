"""User-requested bounded base movement, independent of ROS navigation."""
import json
import multiprocessing
import time
from pathlib import Path
from Rosmaster_Lib import Rosmaster

bot=Rosmaster(car_type=1,com='/dev/ttyUSB1')
bot.ser.write_timeout=.2
zero=bytearray([255,252,7,16,0,0,0,0,23])
def independent_stop():
    time.sleep(4.3)
    for _ in range(40):
        bot.ser.write(zero)
        time.sleep(.05)
report={'mode':'bounded_direct_base_movement','port':'/dev/ttyUSB1',
        'requested_forward_mps':.12,'duration_sec':4.0,'stop_commands_sent':False}
stopper=None
try:
    bot.set_car_type(1)
    bot.create_receive_threading()
    time.sleep(1)
    if bot.get_version()<=0:
        raise RuntimeError('Control board not responding; no movement sent')
    stopper=multiprocessing.get_context('fork').Process(target=independent_stop)
    stopper.start()
    start=time.monotonic()
    while time.monotonic()-start<4.0:
        bot.set_car_motion(.12,0,0)
        time.sleep(.05)
    report['elapsed_sec']=time.monotonic()-start
finally:
    for _ in range(10):
        bot.set_car_motion(0,0,0)
        bot.ser.write(zero)
        time.sleep(.05)
    report['stop_commands_sent']=True
    if stopper:
        stopper.join(5)
        report['independent_stop_exit']=stopper.exitcode
    bot.ser.close()
    (Path.home()/'rosmaster_x3_deploy/logs/direct_auto_move.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report))
