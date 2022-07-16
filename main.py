import numpy as np
import cv2
from tracker import *
from webcamStream import *

# Create tracker object
tracker = EuclideanDistTracker()

# cap = cv2.VideoCapture(0)

cap = WebcamStream(src=0).Start()

# Object detection from Stable camera - varThreshold: higher = less false positives
objectDetector = cv2.createBackgroundSubtractorMOG2(history = 50, varThreshold = 20)

while True:
    frame = cap.ReadFrame()
    
    # Create mask of only moving objects in the frame
    mask = objectDetector.apply(frame)
    _, mask = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)    # Remove all mask that is not completly white
    
    # Grab all contours of moving objects
    contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    detections = []
    
    for i in contours:
        # Calculate area and remove all contours that are too small
        area = cv2.contourArea(i)
        if area > 30000 :
#             cv2.drawContours(frame, [i], -1, (0, 255, 0), 2)
            x, y, w, h = cv2.boundingRect(i)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)    # (x,y), (bottom right, top left), color, thickness
#             detections.append([x, y, w, h])
    
    cv2.imshow("frame", frame)
#     cv2.imshow("mask", mask)
    
    # Track center point of each object detected
#     boxes_ids = tracker.update(detections)
#     print(boxes_ids)
    
    key = cv2.waitKey(1)
    if key == 27:
        break
    
cap.Stop()
cv2.destroyAllWindows()