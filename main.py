import numpy as np
import cv2
from tracker import *
from webcamStream import *
from multiprocessing import Process
from multiprocessing import Queue
import time


def ObjectTracker(queue):

    # Create tracker object
    tracker = EuclideanDistTracker()

    # cap = cv2.VideoCapture(0)

    cap = WebcamStream(src=0).Start()

    # Object detection from Stable camera - varThreshold: higher = less false positives
    #createBackgroundSubtractorMOG2(history = 100, varThreshold = 50)
    #createBackgroundSubtractorKNN(history = 100, dist2Threshold = 50)
    objectDetector = cv2.createBackgroundSubtractorKNN(history = 100, dist2Threshold = 100)

    framesCount = 0
    startTime = time.time()
    while True:
        frame = cap.ReadFrame()
        
        # Create mask of only moving objects in the frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)
        
        mask = objectDetector.apply(gray)
        mask = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)[1]    # Remove all mask that is not completly white
        #cv2.imshow("Thresh", mask)
        mask = cv2.dilate(mask, None, iterations=2)
        cv2.imshow("Dilate", mask)
        
        # Grab all contours of moving objects
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = []
        
        for i in contours:
            # Calculate area and remove all contours that are too small
            area = cv2.contourArea(i)
            if area > 7000 and area < 35000:
    #             cv2.drawContours(frame, [i], -1, (0, 255, 0), 2)
                x, y, w, h = cv2.boundingRect(i)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)    # (x,y), (bottom right, top left), color, thickness
                detections.append([x, y, w, h])
        
        # Track center point of first object detected
        cx, cy, validCnt, hystLatched = tracker.update(detections)
        if hystLatched:
#            print(cx, cy, validCnt, hystLatched)
            cv2.circle(frame, (cx, cy), 20, (0, 0, 255), -1)
            centerPoint = [cx, cy]
            queue.put(centerPoint)

        cv2.imshow("frame", frame)
        
        framesCount += 1

        print("FRAME RATE: ")
        print(1 / (time.time() - startTime))

        startTime = time.time()

        key = cv2.waitKey(1)
        if key == 27:
            break
    
    cap.Stop()
    cv2.destroyAllWindows()

def Consumer(queue):
#    print('Consumer: Running', flush=True)
    # consume work
    while True:
        # get a unit of work
        item = queue.get()
        # check for stop
#        if item is None:
#            break
        # report
        print(f'>got {item}', flush=True)
        
    # all done
    print('Consumer: Done', flush=True)

# main:
queue = Queue()

# start the consumer
consumer_process = Process(target=Consumer, args=(queue,))
consumer_process.start()

# start the producer
producer_process = Process(target=ObjectTracker, args=(queue,))
producer_process.start()

# wait for all processes to finish
producer_process.join()
consumer_process.join()
