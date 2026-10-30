"""Detect one QR target and optionally recommend a Robonix navigation option."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import cv2
import numpy as np

from x3_nav.vision.targets import (
    annotate_targets, detect_qr_targets, recommend_option, target_bearing_deg,
)


from x3_nav.vision.frames import read_frame, demo_frame


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--image")
    source.add_argument("--camera", help="Camera index, device path or stream URL")
    source.add_argument("--demo", action="store_true")
    parser.add_argument("--label", default="X3_TARGET_A")
    parser.add_argument("--fx", type=float, help="Calibrated horizontal focal length in pixels")
    parser.add_argument("--cx", type=float, help="Calibrated horizontal principal point in pixels")
    parser.add_argument("--camera-yaw-deg", type=float, help="Camera yaw relative to robot base")
    parser.add_argument("--provider-dir", help="Parent directory of installed rosmaster_x3_bridge package")
    parser.add_argument("--max-radius", type=float, default=0.2)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "logs" / "vision")
    args = parser.parse_args()
    if args.demo and args.provider_dir:
        parser.error("Demo images cannot be used with a live Provider")
    calibration = (args.fx, args.cx, args.camera_yaw_deg)
    if args.provider_dir and any(value is None for value in calibration):
        parser.error("Provider preview requires --fx, --cx and --camera-yaw-deg")
    api = None
    try:
        frame = demo_frame(args.label) if args.demo else read_frame(args.camera or args.image, bool(args.camera))
        targets = detect_qr_targets(frame, args.label)
        result = {"mode": "simulation" if args.demo else "observation",
                  "preview_only": True, "navigation_goal_sent": False,
                  "detections": [target.to_dict() for target in targets]}
        if len(targets) != 1:
            result["reason"] = "target_not_found" if not targets else "ambiguous_target"
        else:
            if args.demo:
                calibration = (500.0, 320.0, 0.0)
            if any(value is None for value in calibration):
                result["reason"] = "detected_calibration_required_for_navigation"
            else:
                bearing = target_bearing_deg(targets[0], *calibration)
                result["target_bearing_deg"] = bearing
                if args.demo:
                    preview = {"ok": True, "session_id": "simulation-only", "options": [
                        {"option_id": i + 1, "relative_angle_deg": angle, "radius_m": 0.2}
                        for i, angle in enumerate((-20.0, 0.0, 20.0))
                    ]}
                    result["recommendation"] = recommend_option(preview, bearing, args.max_radius)
                    result["reason"] = result["recommendation"]["reason"]
                elif args.provider_dir:
                    from x3_nav.bridge.robonix_preview import RobonixPreview
                    api = RobonixPreview(args.provider_dir)
                    result["recommendation"] = recommend_option(api.preview(), bearing, args.max_radius)
                    result["reason"] = result["recommendation"]["reason"]
                else:
                    result["reason"] = "detected_provider_not_connected"
        args.output_dir.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(args.output_dir / "annotated.png"), annotate_targets(frame, targets)):
            raise RuntimeError("Could not write annotated image")
        if args.demo:
            cv2.imwrite(str(args.output_dir / "target_scene.png"), frame)
        text = json.dumps(result, ensure_ascii=False, indent=2)
        (args.output_dir / "result.json").write_text(text, encoding="utf-8")
        print(text)
        return 0 if len(targets) == 1 and result["reason"] not in {
            "provider_preview_unavailable", "provider_session_expired",
            "no_matching_provider_option", "target_outside_forward_sector",
        } else 2
    except (ValueError, RuntimeError, ImportError, OSError) as exc:
        print(json.dumps({"ok": False, "error": str(exc), "navigation_goal_sent": False}))
        return 1
    finally:
        if api is not None:
            api.close()


if __name__ == "__main__":
    raise SystemExit(main())
