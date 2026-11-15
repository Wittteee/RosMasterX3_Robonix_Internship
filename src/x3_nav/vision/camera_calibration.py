"""Measured RGB camera model for converting a detection pixel to a bearing."""
import json
import math
from pathlib import Path

import cv2
import numpy as np


class CameraCalibration:
    def __init__(self, profile, mounting_yaw_deg=None):
        width, height = profile['image_width'], profile['image_height']
        if any(type(value) is not int or value <= 0 for value in (width, height)):
            raise ValueError('Camera calibration requires positive integer image dimensions')
        self.image_size = (height, width)
        self.matrix = np.asarray(profile['camera_matrix'], dtype=np.float64)
        self.distortion = np.asarray(profile['distortion_coefficients'], dtype=np.float64)
        if (self.matrix.shape != (3, 3) or not np.isfinite(self.matrix).all()
                or not np.allclose(self.matrix[2], [0, 0, 1])
                or not np.allclose([self.matrix[0, 1], self.matrix[1, 0]], [0, 0])
                or not (self.matrix[0, 0] > 0 and self.matrix[1, 1] > 0)
                or not (0 <= self.matrix[0, 2] < width and 0 <= self.matrix[1, 2] < height)):
            raise ValueError('Invalid pinhole camera matrix')
        if (self.distortion.ndim != 1 or self.distortion.size not in (4, 5, 8, 12, 14)
                or not np.isfinite(self.distortion).all()):
            raise ValueError('Invalid OpenCV distortion coefficients')
        self.capture_fourcc = profile.get('capture_fourcc')
        if self.capture_fourcc is not None and self.capture_fourcc not in ('MJPG', 'YUYV'):
            raise ValueError('Camera capture format must be MJPG or YUYV')
        measured_yaw = profile.get('camera_yaw_deg')
        if mounting_yaw_deg is not None and measured_yaw is not None:
            raise ValueError('Use one camera mounting yaw source')
        self.yaw_deg = measured_yaw if mounting_yaw_deg is None else mounting_yaw_deg
        if (type(self.yaw_deg) not in (int, float) or not math.isfinite(self.yaw_deg)
                or not -180 <= self.yaw_deg <= 180):
            raise ValueError('Measured camera mounting yaw is required')

    @classmethod
    def load(cls, path, mounting_yaw_deg=None):
        return cls(json.loads(Path(path).read_text(encoding='utf-8')), mounting_yaw_deg)

    def bearing_deg(self, target):
        if (target.image_height, target.image_width) != self.image_size:
            raise ValueError('Target resolution differs from camera calibration')
        height, width = self.image_size
        if not (0 <= target.center_x < width and 0 <= target.center_y < height):
            raise ValueError('Target pixel is outside the calibrated image')
        pixel = np.array([[[target.center_x, target.center_y]]], dtype=np.float64)
        ray = cv2.undistortPoints(pixel, self.matrix, self.distortion)
        x = float(ray[0, 0, 0])
        if not math.isfinite(x):
            raise ValueError('Camera undistortion produced an invalid ray')
        return self.yaw_deg + math.degrees(math.atan2(-x, 1.0))
