import numpy as np
import cv2
from tracker import *
from webcamStream import *
from multiprocessing import Process
from multiprocessing import Queue
import time
import argparse
import os


def ObjectTracker(queue):

    # Create tracker object
    tracker = EuclideanDistTracker()

    # Open multi-threaded web cam stream
    cap = WebcamStream(src=0).Start()
    
    # Initialize the HOG descriptor/person detector
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())


    framesCount = 0
    startTime = time.time()
    while True:
        frame = cap.ReadFrame()
        #cv2.imshow("preFrame", frame)
        
        # Detect people in the image
        # Parameters:
        # winStride: step size in x and y directions of the sliding window
        # padding: padding around the image
        # scale: image pyramid scale factor
        boxes, weights = hog.detectMultiScale(frame, winStride=(8, 8), padding=(16, 16), scale=1.05)
        
            # Draw bounding boxes on the image
        for (x, y, w, h) in boxes:
            # Draw a rectangle around each detected person
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
        # Optionally, apply Non-Maximum Suppression to reduce overlapping boxes
        # Uncomment the following lines if you want to use NMS
        # from imutils.object_detection import non_max_suppression
        # import numpy as np
        # rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])
        # pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)
        # for (xA, yA, xB, yB) in pick:
        #     cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)
        
        
        # show the output image
        cv2.imshow("frame", frame)
        key = cv2.waitKey(1)
        if key == 27:
            break
        
        
        
        
        
        # # Create mask of only moving objects in the frame
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # gray = cv2.GaussianBlur(gray, (21, 21), 0)
        
        # mask = objectDetector.apply(gray)
        # mask = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)[1]    # Remove all mask that is not completly white
        # #cv2.imshow("Thresh", mask)
        # mask = cv2.dilate(mask, None, iterations=2)
        # cv2.imshow("Dilate", mask)
        
        # # Grab all contours of moving objects
        # contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # detections = []
        
        # for i in contours:
        #     # Calculate area and remove all contours that are too small
        #     area = cv2.contourArea(i)
        #     # Min and maximum area of a valid detected object
        #     if area > 7000 and area < 35000:
        #         x, y, w, h = cv2.boundingRect(i)
        #         cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)    # (x,y), (bottom right, top left), color, thickness
        #         detections.append([x, y, w, h])
        
        # # Track center point of first object detected
        # cx, cy, validCnt, hystLatched = tracker.update(detections)
        # if hystLatched:
        #     cv2.circle(frame, (cx, cy), 20, (0, 0, 255), -1)
        #     centerPoint = [cx, cy]
        #     queue.put(centerPoint)

        # cv2.imshow("frame", frame)
        
        # framesCount += 1

        # print("FRAME RATE: ")
        # print(1 / (time.time() - startTime))

        # startTime = time.time()

        # key = cv2.waitKey(1)
        # if key == 27:
        #     break
    
    cap.Stop()
    cv2.destroyAllWindows()

def Consumer(queue):
    # consume work
    while True:
        # get a unit of work
        item = queue.get()
        # check for stop
        # report
        print(f'>got {item}', flush=True)
        
    # all done
    print('Consumer: Done', flush=True)

# main:
queue = Queue()

cv2.setUseOptimized(True)
cv2.setNumThreads(1)

# start the consumer
consumer_process = Process(target=Consumer, args=(queue,))
consumer_process.start()

# start the producer
producer_process = Process(target=ObjectTracker, args=(queue,))
producer_process.start()

# wait for all processes to finish
producer_process.join()
consumer_process.join()
