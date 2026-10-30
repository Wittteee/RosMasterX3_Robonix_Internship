#!/usr/bin/env bash
ls -l /dev/myserial /dev/ttyUSB*
python3 - <<'PY'
import sys,inspect,time,json
import serial
from Rosmaster_Lib import Rosmaster
print(sys.executable,sys.version)
print(inspect.getfile(Rosmaster),inspect.getfile(serial.Serial))
print(inspect.getsource(Rosmaster.create_receive_threading))
bot=Rosmaster(com='/dev/myserial')
bot.create_receive_threading()
try:
 time.sleep(2)
 print(json.dumps({'version':bot.get_version(),'voltage':bot.get_battery_voltage(),'encoders':bot.get_motor_encoder(),'car_type':bot.get_car_type_from_machine()}))
finally: bot.ser.close()
PY
