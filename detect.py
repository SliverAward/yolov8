from ultralytics import YOLO
from ultralytics.yolo.v8.detect.predict import DetectionPredictor
import cv2

model = YOLO("C:\\yolov8\\runs\\detect\\train5\\weights\\best.pt")


results = model.predict(source="1", show=True)

print(*results)