"""Generate a screen-filling target QR with a clean quiet zone."""
from pathlib import Path
import cv2
root=Path(__file__).resolve().parents[1]/'logs/demo_20261028/qr'
root.mkdir(parents=True,exist_ok=True)
qr=cv2.QRCodeEncoder_create().encode('X3_TARGET_A')
qr=cv2.copyMakeBorder(qr,4,4,4,4,cv2.BORDER_CONSTANT,value=255)
qr=cv2.resize(qr,(990,990),interpolation=cv2.INTER_NEAREST)
assert cv2.QRCodeDetector().detectAndDecode(qr)[0]=='X3_TARGET_A'
cv2.imwrite(str(root/'X3_TARGET_A.png'),qr)
