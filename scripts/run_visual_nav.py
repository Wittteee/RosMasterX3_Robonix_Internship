"""QR -> official candidate -> preparation -> interactive confirmation -> result.

Defaults to no motion. --demo is entirely offline, even with --execute.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from x3_nav.vision.frames import QRObserver
from x3_nav.config import load_config
from x3_nav.mission.visual import VisualNavigation
from x3_nav.vision.targets import detect_qr_targets, target_bearing_deg


from x3_nav.bridge.simulation_provider import SimulationProvider


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--demo", action="store_true")
    source.add_argument("--camera", help="Camera index or device path on this host")
    parser.add_argument("--provider-dir")
    parser.add_argument("--config")
    parser.add_argument("--label", default="X3_TARGET_A")
    parser.add_argument("--fx", type=float)
    parser.add_argument("--cx", type=float)
    parser.add_argument("--camera-yaw-deg", type=float)
    parser.add_argument("--image-width", type=int, help="Width used during camera calibration")
    parser.add_argument("--image-height", type=int, help="Height used during camera calibration")
    parser.add_argument("--max-radius", type=float, default=0.2)
    parser.add_argument("--execute", action="store_true", help="Allow interactive confirmation")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "logs" / "vision")
    args = parser.parse_args(argv)
    if args.demo and args.provider_dir:
        parser.error("Simulation must not connect to a live Provider")
    if not args.demo and (not args.provider_dir or any(value is None for value in
            (args.fx, args.cx, args.camera_yaw_deg, args.image_width, args.image_height))):
        parser.error("Camera mode requires Provider directory, fx/cx/yaw and calibration image dimensions")
    api = None
    mission = None
    mode = "simulation" if args.demo else "live"
    try:
        cfg = load_config(args.config)
        if not 0 < args.max_radius <= cfg['safety']['candidate_distance_m']:
            raise ValueError('Radius exceeds configured candidate distance limit')
        if args.demo:
            api = SimulationProvider()
            calibration = (500, 320, 0)
        else:
            from x3_nav.bridge.robonix_navigation import RobonixNavigation
            api = RobonixNavigation(args.provider_dir, max_radius=args.max_radius, config=cfg)
            calibration = (args.fx, args.cx, args.camera_yaw_deg)

        observe = QRObserver(args.label, camera=args.camera, demo=args.demo,
                             calibration=calibration, image_size=(args.image_height, args.image_width))
        mission = VisualNavigation(api, observe, max_radius=args.max_radius)
        result = mission.run(execute=args.execute)
    except (Exception, KeyboardInterrupt) as exc:
        result = {"ok": False, "reason": str(exc), "events": mission.events if mission else []}
    finally:
        if api is not None:
            try:
                api.close()
            except Exception as exc:
                print("Provider disconnect failed: " + str(exc), file=sys.stderr)
    result["mode"] = mode
    result["physical_arrival_verified"] = mode == "live" and result.get("arrival_verified", False)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    path = args.output_dir / ("mission-" + str(time.time_ns()) + ".json")
    text = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)
    path.write_text(text, encoding="utf-8")
    print(text)
    print("任务记录：" + str(path))
    return 0 if result["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
