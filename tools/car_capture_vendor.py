import socket
import urllib.request
import cv2
import numpy as np
import time

connection = socket.create_connection(('10.206.170.35', 6000), timeout=5)
try:
    connection.sendall(bytes([36]) + b'010F040115#')
    connection.recv(1024)
    with urllib.request.urlopen('http://127.0.0.1:6500/video_feed', timeout=10) as stream:
        buffer = b''
        frames = 0
        while frames < 8:
            chunk = stream.read(4096)
            if not chunk:
                raise RuntimeError('Video stream ended')
            buffer += chunk
            while True:
                start = buffer.find(bytes([255, 216]))
                end = buffer.find(bytes([255, 217]), start + 2) if start >= 0 else -1
                if end < 0:
                    break
                jpeg = buffer[start:end + 2]
                buffer = buffer[end + 2:]
                frames += 1
                if frames == 8:
                    break
        image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError('No image')
        output = '/tmp/x3_obstacle1_before_20261121.png'
        if not cv2.imwrite(output, image):
            raise RuntimeError('Image write failed')
        print(output, image.shape, 'captured_unix', time.time())
finally:
    connection.close()
