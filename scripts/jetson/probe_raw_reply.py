"""Capture version-request replies without sending motor or servo commands."""
import inspect
import json
import time
from Rosmaster_Lib import Rosmaster
import serial

print(inspect.getsource(Rosmaster._Rosmaster__request_data))
for port in ("/dev/ttyUSB1", "/dev/ttyUSB2"):
    # Version query frame follows the installed vendor library protocol.
    frame = [0xFF, 0xFC, 0x05, 0x50, 0x51, 0x00]
    frame.append(sum(frame, 257 - 0xFC) & 0xFF)
    link = serial.Serial(port, 115200, timeout=0.1)
    received = bytearray()
    try:
        for attempt in range(5):
            link.write(bytearray(frame))
            time.sleep(0.1)
            received.extend(link.read(256))
        print(json.dumps({"port": port, "query_hex": bytearray(frame).hex(),
                          "received_bytes": len(received), "received_hex": received.hex()}))
    finally:
        link.close()
