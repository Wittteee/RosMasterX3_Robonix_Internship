"""QR -> official candidate -> preparation -> interactive confirmation -> result.

Defaults to no motion. --demo is entirely offline, even with --execute.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from x3_nav.vision.navigation import VisualNavigation
from x3_nav.vision.targets import detect_qr_targets, target_bearing_deg


class SimulationProvider:
    """No transport, no ROS imports, no physical actuation."""
    def health(self):
        return {"ok": True, "simulation": True}

    def preview(self):
        return {"ok": True, "session_id": "simulation", "expires_epoch": time.time() + 60,
                "options": [{"option_id": n, "radius_m": 0.2, "relative_angle_deg": angle}
                            for n, angle in enumerate((-20, 0, 20), 1)]}

    def prepare(self, session_id, option_id):
        return {"ok": True, "token": "simulation-token", "confirmation_text": "simulation",
                "expires_at_epoch": time.time() + 30}

    def execute(self, token, confirmation_text):
        return {"ok": True, "arrived": True, "move_base_status_name": "SUCCEEDED",
                "simulation": True}

    def cancel(self):
        return {"ok": True, "simulation": True}

    def close(self):
        pass


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--demo", action="store_true")
    source.add_argument("--camera", help="Camera index or device path on this host")
    source.add_argument("--observation-file", help="Fresh JSON from the real-camera QR observer; centered trigger only")
    parser.add_argument("--provider-dir")
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
    if not args.demo and not args.observation_file and (not args.provider_dir or any(value is None for value in
            (args.fx, args.cx, args.camera_yaw_deg, args.image_width, args.image_height))):
        parser.error("Camera mode requires Provider directory, fx/cx/yaw and calibration image dimensions")
    if args.observation_file and not args.provider_dir:
        parser.error("Real observation mode requires Provider directory")
    api = None
    mission = None
    mode = "simulation" if args.demo else "live"
    try:
        if args.demo:
            api = SimulationProvider()
            calibration = (500, 320, 0)
        else:
            from x3_nav.vision.robonix_navigation import RobonixNavigation
            api = RobonixNavigation(args.provider_dir, max_radius=args.max_radius)
            calibration = (args.fx, args.cx, args.camera_yaw_deg)

        def observe():
            if args.observation_file:
                from x3_nav.vision.camera_observation import centered_qr_bearing
                return centered_qr_bearing(args.observation_file, args.label)
            from preview_visual_target import demo_frame, read_frame
            frame = demo_frame(args.label) if args.demo else read_frame(args.camera, True)
            if not args.demo and frame.shape[:2] != (args.image_height, args.image_width):
                raise ValueError("camera_resolution_differs_from_calibration")
            targets = detect_qr_targets(frame, args.label)
            if len(targets) != 1:
                raise ValueError("target_not_found" if not targets else "ambiguous_target")
            return target_bearing_deg(targets[0], *calibration)

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
