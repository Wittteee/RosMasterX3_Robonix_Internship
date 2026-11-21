#!/usr/bin/env python
"""Refuse to start when MakerControl or another driver already owns the car."""
from __future__ import print_function

import os
import subprocess
import sys


def fail(message):
    print(message, file=sys.stderr)
    raise SystemExit(2)


def _argv(pid):
    try:
        raw = open("/proc/%s/cmdline" % pid, "rb").read()
    except (IOError, OSError):
        return []
    if not isinstance(raw, str):
        raw = raw.decode("utf-8", "replace")
    return [part for part in raw.split("\0") if part]


def running(name):
    """Match a driver process, not a shell whose command text mentions the name."""
    shells = set(["bash", "sh", "dash", "pgrep", "grep", "sed", "cat"])
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        args = _argv(entry)
        if not args or os.path.basename(args[0]) in shells:
            continue
        for arg in args:
            if os.path.basename(arg) == name or name in os.path.basename(arg):
                return True
    return False


def busy(path):
    return subprocess.call(["fuser", path], stdout=subprocess.PIPE,
                           stderr=subprocess.PIPE) == 0


def main():
    bringup = len(sys.argv) > 1 and sys.argv[1] == "true"
    astra = len(sys.argv) > 2 and sys.argv[2] == "true"
    if not bringup and not astra:
        print("skipped")
        return 0
    if running("rosmaster_main.py") or running("MakerControl"):
        fail("MakerControl is running; close it before starting the car")
    if running("Mcnamu_driver.py") or running("x3_base_driver.py"):
        fail("A chassis driver is already running")
    devices = []
    if bringup:
        devices.extend(["/dev/x3_base", "/dev/x3_lidar"])
    if astra:
        devices.append("/dev/x3_camera_rgb")
    for path in devices:
        if not os.path.exists(path):
            fail("Missing %s. Install config/99-x3-devices.rules and replug USB." % path)
        if busy(path):
            fail("%s is already open" % path)
    print("devices-free")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
