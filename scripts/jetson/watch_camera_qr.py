"""Read real camera frames and expose a timestamped QR observation."""
import json
import time
from pathlib import Path

import cv2


def decode_targets(detector, image, image_width):
    found, labels, points, _ = detector.detectAndDecodeMulti(image)
    if not found or points is None:
        return []
    scale_x = image_width / image.shape[1]
    return [
        {'label': label, 'center_x': float(corners[:, 0].mean()) * scale_x,
         'image_width': image_width}
        for label, corners in zip(labels, points) if label == 'X3_TARGET_A'
    ]


def detect_targets(frame, detector, contrast):
    width = frame.shape[1]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if width > 960:
        gray = cv2.resize(gray, (960, round(frame.shape[0] * 960 / width)),
                          interpolation=cv2.INTER_AREA)
    targets = decode_targets(detector, gray, width)
    if not targets:
        # Retry the entire frame with local contrast enhancement for uneven lighting.
        targets = decode_targets(detector, contrast.apply(gray), width)
    return targets


def main():
    root = Path.home() / 'rosmaster_x3_deploy/logs'
    cv2.setNumThreads(2)
    detector = cv2.QRCodeDetector()
    contrast = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    cap = cv2.VideoCapture(0)
    try:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        deadline = time.monotonic() + 180
        while time.monotonic() < deadline:
            ok, frame = cap.read()
            result = {'timestamp': time.time(), 'source': 'real_camera',
                      'frame_ok': ok, 'targets': []}
            if ok:
                result['targets'] = detect_targets(frame, detector, contrast)
                frame_path = root / 'camera_qr_current.tmp.jpg'
                if cv2.imwrite(str(frame_path), frame):
                    frame_path.replace(root / 'camera_qr_current.jpg')
            result['processing_latency_sec'] = time.time() - result['timestamp']
            tmp = root / 'camera_qr.tmp'
            tmp.write_text(json.dumps(result))
            tmp.replace(root / 'camera_qr.json')
            time.sleep(.2)
    finally:
        cap.release()


if __name__ == '__main__':
    main()
