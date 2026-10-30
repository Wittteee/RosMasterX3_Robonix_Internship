#!/usr/bin/env bash
python3 - <<'PY'
import inspect
from Rosmaster_Lib import Rosmaster
for name in ('__init__','__del__','set_motor','get_car_type_from_machine','get_motor_pid','get_uart_servo_value'):
 if hasattr(Rosmaster,name): print(inspect.getsource(getattr(Rosmaster,name)))
print(inspect.getsource(Rosmaster._Rosmaster__parse_data))
PY
