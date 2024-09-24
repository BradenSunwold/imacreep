import numpy as np
import cv2
from tracker_opt import *
from webcamStream import *
from multiprocessing import Process
from multiprocessing import Queue
import time
import time
import argparse
import os


def ObjectTracker(queue):
    
    # Create tracker object
    tracker = EuclideanDistTracker()
    
    # Open multithreaded camera stream
    cap = WebcamStream(src=0).Start()
    
    # Load the Haar cascade XML file
    cascade_path = 'haarcascade_frontalface_default.xml'  # 2.6 FPS
    #cascade_path = 'haarcascade_frontalface_alt2.xml'      # 1.4 FPS
    #cascade_path = 'haarcascade_frontalface_alt.xml'        # 1.6 FPS
    #cascade_path = 'haarcascade_smile.xml'                  # crazy amount of false detects
    #cascade_path = 'haarcascade_lefteye_2splits.xml'        #2.6 FPS 
    #cascade_path = 'haarcascade_righteye_2splits.xml'        # 2.6 FPS detects riht side eye / both frontal
    
    #cascade_path = 'haarcascade_profileface.xml'
    
    #cascade_path = 'haarcascade_fullbody.xml'                  # 3 FPS, works alright
    #cascade_path = 'haarcascade_upperbody.xml'              # 1.4 - 2 FPS - need high max size and 
    #cascade_path = 'haarcascade_lowerbody.xml' 
    
    #cascade_path = 'haarcascade_frontalcatface.xml'
    #cascade_path = 'haarcascade_frontalcatface_extended.xml'

    face_cascade = cv2.CascadeClassifier(cascade_path)
    
    framesCount = 0
    startTime = time.time()
    while True:
        frame = cap.ReadFrame()
        resize_frame = cv2.resize(frame, (640, 480))
        # gray_frame = cv2.cvtColor(resize_frame, cv2.COLOR_BGR2GRAY)
        #cv2.imshow("preFrame", frame)
        
        # Detect faces in the frame
        
        # Use this tuning for full body detection - Kind of works
        # faces = face_cascade.detectMultiScale(frame, scaleFactor=1.005, minNeighbors=6, minSize=(80, 80), maxSize=(480, 640))
        
        # Use this tuning for faces
        faces = face_cascade.detectMultiScale(frame, scaleFactor=1.11, minNeighbors=3, maxSize=(480, 640))
        
        # Use this tuning for eyes
        #faces = face_cascade.detectMultiScale(frame, scaleFactor=1.1, minNeighbors=3)

        detections = []
        
        # Draw rectangles around detected faces
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            detections.append([x, y, w, h])
        
         # Track center point of first object detected
        cx, cy, validCnt, hystLatched = tracker.update(detections)
    
        if hystLatched:
            #            print(cx, cy, validCnt, hystLatched)
            cv2.circle(frame, (cx, cy), 20, (0, 0, 255), -1)
            centerPoint = [cx, cy]
            queueTime = time.time()
            queueTimeStr = str(queueTime)
            queue.put(centerPoint)
            # print("Enqueueing: " + queueTimeStr)
            # print(centerPoint)
            # print()
        
        framesCount += 1
        print("FRAME RATE: ")
        print(1 / (time.time() - startTime))

        startTime = time.time()
        
        key = cv2.waitKey(1)
        if key == 27:
            break

    cap.Stop()
    cv2.destroyAllWindows()

