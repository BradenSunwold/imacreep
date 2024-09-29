import numpy as np
import cv2
from tracker_opt import *
from webcamStream import *
from multiprocessing import Process
from multiprocessing import Queue
import time
import serial


def ObjectTracker(queue):
    
    # Create tracker object
    tracker = EuclideanDistTracker()

    # Init serial stream to object detector computer
    serial_port = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=1)
    
    framesCount = 0
    startTime = time.time()
    while True:      

        detections = []

        # Check if data is waiting  
        if serial_port.in_waiting > 0: 
            uart_message = serial_port.readline().decode('utf-8').strip()

            data = {}
            # Remove brackets and split by comma
            for item in uart_message[1:-1].split(',') :  
                key, value = item.split(':')
                data[key] = int(value)  # Store the parsed values as integers
            print(f"Parsed data: {data}")

            detections.append([data['x'], data['y'], data['w'], data['h']])


            # detections.append([x, y, w, h])
        
    
            # Track center point of first object detected
            cx, cy, validCnt, hystLatched = tracker.update(detections)
            if hystLatched:
                #            print(cx, cy, validCnt, hystLatched)
                #cv2.circle(frame, (cx, cy), 20, (0, 0, 255), -1)
                centerPoint = [cx, cy]
                # queueTime = time.time()
                # queueTimeStr = str(queueTime)
                queue.put(centerPoint)
                # print("Enqueueing: " + queueTimeStr)
                # print(centerPoint)
                # print()
            
        # framesCount += 1
        # print("FRAME RATE: ")
        # print(1 / (time.time() - startTime))

        # startTime = time.time()
        
        key = cv2.waitKey(1)
        if key == 27:
            break

    cv2.destroyAllWindows()

