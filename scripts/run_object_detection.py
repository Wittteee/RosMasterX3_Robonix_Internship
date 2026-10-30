"""Detect COCO objects in one image/camera frame. No robot connection or motion."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
import cv2
from download_yolox import DEFAULT_MODEL
from x3_nav.vision.frames import read_frame
from x3_nav.vision.yolox import COCO_CLASSES, YOLOXDetector, annotate, select_unique


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--image")
    source.add_argument("--camera", help="Camera index, e.g. 0, or local device path")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--confidence", type=float, default=0.5)
    parser.add_argument("--label", choices=COCO_CLASSES, help="Require one matching object for candidate output")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "logs" / "detection")
    args = parser.parse_args(argv)
    result = {"ok": False, "navigation_goal_sent": False, "mode": "detection_only"}
    try:
        detector = YOLOXDetector(args.model, confidence=args.confidence)
        frame = read_frame(args.image if args.image else args.camera, camera=args.camera is not None)
        started = time.perf_counter()
        boxes = detector.detect(frame)
        result.update(ok=True, inference_ms=round((time.perf_counter()-started)*1000, 2),
                      detections=[box.to_dict() for box in boxes])
        args.output_dir.mkdir(parents=True, exist_ok=True)
        stem = "detection-" + str(time.time_ns())
        path = args.output_dir / (stem + ".jpg")
        if not cv2.imwrite(str(path), annotate(frame, boxes)):
            raise OSError("Cannot write annotated image")
        result["annotated_image"] = str(path.resolve())
        if args.label:
            result["candidate_detection"] = select_unique(boxes, args.label).as_detection().to_dict()
        record = args.output_dir / (stem + ".json")
    except Exception as exc:
        result.update(ok=False, reason=str(exc))
        args.output_dir.mkdir(parents=True, exist_ok=True)
        record = args.output_dir / ("error-" + str(time.time_ns()) + ".json")
    payload = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)
    record.write_text(payload, encoding="utf-8")
    print(payload)
    return 0 if result["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
