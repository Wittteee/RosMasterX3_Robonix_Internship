"""YOLOX-s COCO inference with OpenCV DNN (CPU); no robot dependencies.

Model format/preprocessing: OpenCV Zoo object_detection_yolox (Apache-2.0).
https://github.com/opencv/opencv_zoo/tree/main/models/object_detection_yolox
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
import re
import time
import cv2
import numpy as np

from .detect import Detection
from .frames import read_frame
from .targets import Target, target_bearing_deg

COCO_CLASSES = tuple((
    "person,bicycle,car,motorcycle,airplane,bus,train,truck,boat,traffic light,"
    "fire hydrant,stop sign,parking meter,bench,bird,cat,dog,horse,sheep,cow,"
    "elephant,bear,zebra,giraffe,backpack,umbrella,handbag,tie,suitcase,frisbee,"
    "skis,snowboard,sports ball,kite,baseball bat,baseball glove,skateboard,"
    "surfboard,tennis racket,bottle,wine glass,cup,fork,knife,spoon,bowl,banana,"
    "apple,sandwich,orange,broccoli,carrot,hot dog,pizza,donut,cake,chair,couch,"
    "potted plant,bed,dining table,toilet,tv,laptop,mouse,remote,keyboard,"
    "cell phone,microwave,oven,toaster,sink,refrigerator,book,clock,vase,scissors,"
    "teddy bear,hair drier,toothbrush"
).split(","))


@dataclass(frozen=True)
class ObjectBox:
    label: str
    confidence: float
    xyxy: tuple[float, float, float, float]
    image_width: int
    image_height: int

    def to_dict(self):
        return asdict(self)

    def as_detection(self):
        x1, y1, x2, y2 = self.xyxy
        return Detection(self.label, self.confidence,
                         (x1 + x2) / self.image_width - 1,
                         (x2 - x1) * (y2 - y1) / (self.image_width * self.image_height),
                         source="yolox-opencv")

    def as_target(self):
        x1, y1, x2, y2 = self.xyxy
        return Target(self.label, (x1+x2)/2, (y1+y2)/2,
                      [[x1,y1],[x2,y1],[x2,y2],[x1,y2]], self.image_width, self.image_height)


def select_unique(boxes, label):
    matches = [box for box in boxes if box.label == label]
    if len(matches) != 1:
        raise ValueError(f"Target {label!r}: expected one object, found {len(matches)}")
    return matches[0]


class YOLOXDetector:
    def __init__(self, model, confidence=0.5, nms_threshold=0.45):
        if not 0 < confidence <= 1 or not 0 < nms_threshold <= 1:
            raise ValueError("Confidence and NMS thresholds must be in (0, 1]")
        if not Path(model).is_file():
            raise FileNotFoundError("Model missing; run scripts/download_yolox.py first")
        self.confidence = confidence
        self.nms_threshold = nms_threshold
        self.net = cv2.dnn.readNetFromONNX(str(model))
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    def detect(self, frame):
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3 or frame.size == 0:
            raise ValueError("Expected a nonempty BGR image")
        h, w = frame.shape[:2]
        ratio = min(640 / h, 640 / w)
        resized = cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB),
                             (max(1, int(w * ratio)), max(1, int(h * ratio))))
        canvas = np.full((640, 640, 3), 114, dtype=np.float32)
        canvas[:resized.shape[0], :resized.shape[1]] = resized
        self.net.setInput(np.ascontiguousarray(canvas.transpose(2, 0, 1)[None]))
        return self.decode(self.net.forward(), w, h, ratio,
                           self.confidence, self.nms_threshold)

    @staticmethod
    def decode(output, width, height, ratio, confidence=0.5, nms_threshold=0.45):
        if output.shape != (1, 8400, 85) or not np.isfinite(output).all():
            raise ValueError("Expected finite YOLOX output [1,8400,85]")
        rows = output[0]
        scores = rows[:, 4:5] * rows[:, 5:]
        classes = scores.argmax(axis=1)
        scores = scores.max(axis=1)
        valid = scores >= confidence
        grids, strides = [], []
        for stride in (8, 16, 32):
            yy, xx = np.mgrid[:640//stride, :640//stride]
            grids.append(np.stack((xx, yy), axis=-1).reshape(-1, 2))
            strides.append(np.full((xx.size, 1), stride))
        grid = np.concatenate(grids)[valid]
        stride = np.concatenate(strides)[valid]
        rows, scores, classes = rows[valid], scores[valid], classes[valid]
        if not len(rows):
            return []
        centers = (rows[:, :2] + grid) * stride / ratio
        with np.errstate(over="ignore"):
            sizes = np.exp(rows[:, 2:4]) * stride / ratio
        if not np.isfinite(sizes).all():
            raise ValueError("Invalid YOLOX box size")
        starts = centers - sizes / 2
        ends = centers + sizes / 2
        starts = np.clip(starts, [0, 0], [width, height])
        ends = np.clip(ends, [0, 0], [width, height])
        boxes = np.concatenate((starts, ends-starts), axis=1)
        result = []
        # Per-class NMS also supports OpenCV versions without NMSBoxesBatched.
        for class_id in np.unique(classes):
            indices = np.flatnonzero((classes == class_id) & (boxes[:, 2] > 0) & (boxes[:, 3] > 0))
            keep = cv2.dnn.NMSBoxes(boxes[indices].tolist(), scores[indices].tolist(), confidence, nms_threshold)
            for local in np.asarray(keep).reshape(-1):
                i = indices[int(local)]
                result.append(ObjectBox(COCO_CLASSES[int(class_id)], float(scores[i]),
                                        tuple(map(float, (*starts[i], *ends[i]))), width, height))
        return sorted(result, key=lambda box: box.confidence, reverse=True)


class ObjectObserver:
    """Acquire again on each mission check; reject missing/ambiguous targets."""
    def __init__(self, detector, camera, label, calibration, image_size):
        if not (str(camera).isdigit() or re.fullmatch(r"/dev/video\d+", str(camera))):
            raise ValueError("Navigation requires a local camera index or /dev/videoN, not a recorded video")
        self.detector, self.camera, self.label = detector, camera, label
        self.calibration, self.image_size = calibration, image_size
        self.last_boxes = []

    def __call__(self):
        started = time.monotonic()
        frame = read_frame(self.camera, camera=True)
        if frame.shape[:2] != self.image_size:
            raise ValueError("Camera frame dimensions differ from calibration")
        self.last_boxes = self.detector.detect(frame)
        if time.monotonic() - started > 2.0:
            raise ValueError("Camera acquisition/inference exceeded the 2-second observation window")
        return target_bearing_deg(select_unique(self.last_boxes, self.label).as_target(), *self.calibration)


def annotate(frame, boxes):
    result = frame.copy()
    for box in boxes:
        x1, y1, x2, y2 = map(int, box.xyxy)
        cv2.rectangle(result, (x1, y1), (x2, y2), (0, 200, 0), 2)
        cv2.putText(result, f"{box.label} {box.confidence:.2f}", (x1, max(16, y1-5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 1)
    return result
