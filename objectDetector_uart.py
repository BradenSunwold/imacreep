import numpy as np
import cv2
from tracker_opt import *
from webcamStream import *
from multiprocessing import Process
from multiprocessing import Queue
import time
import serial

def ParseUart(data_str):
    # Split the incoming string by '|' to separate each message
    messages = data_str.strip().split("|")
    print("Split Message:   ")
    print(messages)
    parsed_data = []

    for message in messages:
        if not message.strip():  # Skip empty messages
            continue

        data_pairs = message.split(",")  # Split by comma
        # print("Data Pairs:   ")
        # print(data_pairs)
        data = {}

        for item in data_pairs:
            # Ensure that the item contains key-value pairs split by ':'
            if ':' in item:
                key, value = item.split(':')
                key = key.strip()  # Remove any extra spaces
                value = value.strip()  # Remove any extra spaces
                
                # Attempt to store the value as a float
                try:
                    data[key] = float(value)
                    # print("Data:   ")
                    # print(data)
                except ValueError:
                    print(f"Error: Unable to convert {value} to float.")
                    continue  # Skip to the next item

        if data:  # Only append non-empty data
            parsed_data.append(data)

    return parsed_data


def ObjectTracker(queue):
    
    # Create tracker object
    tracker = EuclideanDistTracker()

    # Init serial stream to object detector computer
    serial_port = serial.Serial(port='/dev/ttyS0', baudrate=115200, timeout=1)
    
    framesCount = 0
    startTime = time.time()
    count = 0
    buffer = ""
    while True:      

        detections = []

        # Check if data is waiting  
        if serial_port.in_waiting > 0: 
            # uart_message = serial_port.read().decode('utf-8').strip()

            # print(uart_message)

            # #data = {}
            # # Remove brackets and split by comma
            # #for item in uart_message[1:-1].split(',') :  
            # #    key, value = item.split(':')
            # #    data[key] = float(value)  # Store the parsed values as integers
            # #print(f"Parsed data: {data}")

            # data = ParseUart(uart_message)
            # detections = np.array([[d['x'], d['y'], d['w'], d['h']] for d in data])
            # # print("Detections array: ")
            # # print(data)
            # #detections.append([data['x'], data['y'], data['w'], data['h']])


            # # detections.append([x, y, w, h])
            uart_message = serial_port.read(serial_port.in_waiting).decode('utf-8')
            buffer += uart_message  # Append new data to the buffer

            # Split by '|' which denotes the end of one data frame
            if "|" in buffer:
                # Split by '|', keep the last partial frame (if any) in the buffer
                messages = buffer.split("|")
                complete_messages = messages[:-1]  # Complete messages
                buffer = messages[-1]  # Keep the incomplete part in the buffer

                for message in complete_messages:
                    # Parse and process each complete message
                    data = ParseUart(message)
                    if data:
                        detections = np.array([[d['x'], d['y'], d['w'], d['h']] for d in data])
                        count += 1
                        print(f"Message count: {count}")
                        print(f"Detections: {detections}")
        
        
                        # Calculate center of person
                        cx = (detections[0,0] + detections[0,0] + detections[0,2]) // 2
                        cy = ((detections[0,1] + detections[0,1] + detections[0,3]) // 2) + int(detections[0,3] * 0.25) # Eyes take in opposite sign as TrackerTuning.py displays
                        center = [cx, cy]
                        queue.put(center)


            # # Track center point of first object detected
            # cx, cy, validCnt, hystLatched = tracker.update(detections)
            # if hystLatched:
            #     #            print(cx, cy, validCnt, hystLatched)
            #     #cv2.circle(frame, (cx, cy), 20, (0, 0, 255), -1)
            #     centerPoint = [cx, cy]
            #     # queueTime = time.time()
            #     # queueTimeStr = str(queueTime)
            #     queue.put(centerPoint)
            #     # print("Enqueueing: " + queueTimeStr)
            #     # print(centerPoint)
            #     # print()
            
        # framesCount += 1
        # print("FRAME RATE: ")
        # print(1 / (time.time() - startTime))

        # startTime = time.time()
        
        key = cv2.waitKey(1)
        if key == 27:
            break

    cv2.destroyAllWindows()

