from threading import Thread
import cv2
from imutils.video import VideoStream


class WebcamStream:
    def __init__ (self, src = 0):
        # init video stream and read first frame - Set to low res 720p
        self.stream = cv2.VideoCapture(src)
        #self.stream = VideoStream(src=0, resolution=(320, 240)).start()
#        self.stream.set(3, 1280)
#        self.stream.set(4, 720)
        self.stream.set(3, 640)
        self.stream.set(4, 480)
        self.stream.set(cv2.CAP_PROP_FPS, 1)
        
        (self.grabbed, self.frame) = self.stream.read()
        
        #init var used to indicate if thread should end
        self.stopped = False
    
    def Start(self):
        # Start thread
        Thread(target=self.Update, args=()).start()
        return self
    
    def Update(self):
        # Keep grabbing frames until thread is stopped
        count = 0
        while True:
            if self.stopped:
                self.stream.release()
                return
            
            # Read the frame only every n frames
            if count % 2 == 0:  # Adjust the skip rate
                (self.grabbed, self.frame) = self.stream.read()
            count += 1
            
    def ReadFrame(self):
        # Return most recent frame
        return self.frame
    
    def Stop(self):
        # Flag stopped
        self.stopped = True
        self.stream.release()
