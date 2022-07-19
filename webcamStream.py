from threading import Thread
import cv2

class WebcamStream:
    def __init__ (self, src = 0):
        # init video stream and read first frame - Set to low res 720p
        self.stream = cv2.VideoCapture(src)
        self.stream.set(3, 1280)
        self.stream.set(4, 720)
        (self.grabbed, self.frame) = self.stream.read()
        
        #init var used to indicate if thread should end
        self.stopped = False
    
    def Start(self):
        # Start thread
        Thread(target=self.Update, args=()).start()
        return self
    
    def Update(self):
        # Keep grabbing frames until thread is stopped
        while True:
            # If thread indicator var is set, stop the thread
            if self.stopped:
                self.stream.release()
                return
            
            # Else read next frame
            (self.grabbed, self.frame) = self.stream.read()
            
    def ReadFrame(self):
        # Return most recent frame
        return self.frame
    
    def Stop(self):
        # Flag stopped
        self.stopped = True
