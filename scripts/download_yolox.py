"""Download the official OpenCV Zoo YOLOX-s weights with SHA-256 verification."""
import argparse
import hashlib
from pathlib import Path
import urllib.request

MODEL = "object_detection_yolox_2022nov.onnx"
URL = "https://media.githubusercontent.com/media/opencv/opencv_zoo/main/models/object_detection_yolox/" + MODEL
SHA256 = "c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063"
SIZE = 35858002
DEFAULT_MODEL = Path(__file__).resolve().parents[1] / "models" / MODEL


def download(destination=DEFAULT_MODEL):
    destination = Path(destination)
    if destination.exists() and hashlib.sha256(destination.read_bytes()).hexdigest() == SHA256:
        return destination
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(".onnx.part")
    try:
        with urllib.request.urlopen(URL, timeout=60) as response, temporary.open("wb") as output:
            digest, size = hashlib.sha256(), 0
            while chunk := response.read(1024 * 1024):
                size += len(chunk)
                if size > SIZE:
                    raise ValueError("Model exceeds expected size")
                digest.update(chunk)
                output.write(chunk)
        if size != SIZE or digest.hexdigest() != SHA256:
            raise ValueError("Model SHA-256/size mismatch")
        temporary.replace(destination)
    finally:
        temporary.unlink(missing_ok=True)
    return destination


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_MODEL)
    print(download(parser.parse_args().output))
