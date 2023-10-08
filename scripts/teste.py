import cv2
from ultralytics import YOLO

model = YOLO('/home/dallagnol/work_ws/src/theta_object_detect/last.pt') 

results = model.predict(source= '0', show=True)

print(results)