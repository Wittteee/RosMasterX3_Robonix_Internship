"""Image acquisition and offline QR fixtures; no navigation side effects."""
import cv2
import numpy as np
from x3_nav.vision.targets import detect_qr_targets, target_bearing_deg


class QRObserver:
    """Produce a bearing from a new frame; reject ambiguity and calibration mismatch."""
    def __init__(self, label, *, camera=None, demo=False, calibration=None, image_size=None):
        self.label = label
        self.camera = camera
        self.demo = demo
        self.calibration = (500, 320, 0) if demo else calibration
        self.image_size = image_size

    def __call__(self):
        frame = demo_frame(self.label) if self.demo else read_frame(self.camera, True)
        if not self.demo and frame.shape[:2] != self.image_size:
            raise ValueError('camera_resolution_differs_from_calibration')
        targets = detect_qr_targets(frame, self.label)
        if len(targets) != 1:
            raise ValueError('target_not_found' if not targets else 'ambiguous_target')
        return target_bearing_deg(targets[0], *self.calibration)

def read_frame(source: str, camera: bool) -> np.ndarray:
    if not camera:
        frame = cv2.imread(source)
        if frame is None:
            raise ValueError("Cannot read image: " + source)
        return frame
    capture = cv2.VideoCapture(int(source) if source.isdigit() else source)
    try:
        if not capture.isOpened():
            raise RuntimeError("Cannot open camera: " + source)
        ok, frame = capture.read()
        if not ok or frame is None:
            raise RuntimeError("Camera did not return a frame")
        return frame
    finally:
        capture.release()


def demo_frame(label: str) -> np.ndarray:
    qr = cv2.QRCodeEncoder_create().encode(label)
    qr = cv2.copyMakeBorder(qr, 4, 4, 4, 4, cv2.BORDER_CONSTANT, value=255)
    qr = cv2.resize(qr, (240, 240), interpolation=cv2.INTER_NEAREST)
    image = np.full((480, 640, 3), 255, dtype=np.uint8)
    image[120:360, 100:340] = cv2.cvtColor(qr, cv2.COLOR_GRAY2BGR)
    return image
