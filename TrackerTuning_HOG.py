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
    
    # Load in coco class lables that our yolo model was trained on and the weights
    labelsPath = "/home/bradensunwold/Documents/yolo-coco/coco.names"
    LABELS = open(labelsPath).read().strip().split("\n")
    
    weightsPath = "/home/bradensunwold/Documents/yolo-coco/yolov3-tiny.weights"
    configPath = "/home/bradensunwold/Documents/yolo-coco/yolov3-tiny.cfg"
    
    # Object detection from Stable camera - varThreshold / dist2Threshold: higher = less false positives
    # Test both these algorithms to determine which works best in your environment

    #createBackgroundSubtractorMOG2(history = 100, varThreshold = 50)
    objectDetector = cv2.createBackgroundSubtractorKNN(history = 90, dist2Threshold = 25)
    
    # Choose random colors to represent different objects
    np.random.seed(42)
    COLORS = np.random.randint(0, 255, size=(len(LABELS), 3), dtype="uint8")

    # load YOLO object detector 
    print("[INFO] loading YOLO from disk...")
    net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)

    # determine only the *output* layer names that we need from YOLO
    ln = net.getLayerNames()
    ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]


    framesCount = 0
    startTime = time.time()
    while True:
        frame = cap.ReadFrame()
        #cv2.imshow("preFrame", frame)
        
        # construct a blob from the input image and then perform a forward
        # pass of the YOLO object detector, giving us our bounding boxes and
        # associated probabilities
        blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (320, 320), swapRB=True, crop=False)
        net.setInput(blob)
        start = time.time()
        layerOutputs = net.forward(ln)
        end = time.time()
        # show timing information on YOLO
        print("[INFO] YOLO took {:.6f} seconds".format(end - start))
        
        # initialize our lists of detected bounding boxes, confidences, and
        # class IDs, respectively
        boxes = []
        confidences = []
        classIDs = []
        
        # loop over each of the layer outputs
        for output in layerOutputs:
            # loop over each of the detections
            for detection in output:
                # extract the class ID and confidence (i.e., probability) of
                # the current object detection
                scores = detection[5:]
                classID = np.argmax(scores)
                confidence = scores[classID]
                # filter out weak predictions by ensuring the detected
                # probability is greater than the minimum probability
                if confidence > .75:
                    # scale the bounding box coordinates back relative to the
                    # size of the image, keeping in mind that YOLO actually
                    # returns the center (x, y)-coordinates of the bounding
                    # box followed by the boxes' width and height
                    box = detection[0:4] * np.array([480, 640, 640, 480])
                    (centerX, centerY, width, height) = box.astype("int")
                    # use the center (x, y)-coordinates to derive the top and
                    # and left corner of the bounding box
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))
                    # update our list of bounding box coordinates, confidences,
                    # and class IDs
                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(confidence))
                    classIDs.append(classID)
        
        # apply non-maxima suppression to suppress weak, overlapping bounding
        # boxes
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, .75, .3)
        
        # ensure at least one detection exists
        if len(idxs) > 0:
            print("found something")
            # loop over the indexes we are keeping
            for i in idxs.flatten():
                # extract the bounding box coordinates
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])
                # draw a bounding box rectangle and label on the image
                color = [int(c) for c in COLORS[classIDs[i]]]
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                text = "{}: {:.4f}".format(LABELS[classIDs[i]], confidences[i])
                cv2.putText(frame, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, color, 2)
        # show the output image
        cv2.imshow("Image", frame)
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
