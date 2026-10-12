"""Read firmware version from the unused CH340 port; no motion command."""
import json
import time
from Rosmaster_Lib import Rosmaster

bot = Rosmaster(com="/dev/ttyUSB2")
bot.create_receive_threading()
try:
    versions = []
    for attempt in range(10):
        versions.append(bot.get_version())
        time.sleep(0.1)
    print(json.dumps({"port": "/dev/ttyUSB2", "versions": versions,
                      "battery_voltage": bot.get_battery_voltage()}))
finally:
    bot.ser.close()
