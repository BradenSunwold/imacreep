import subprocess
import cv2
import time

def start_libcamera_vid(width=640, height=480):
    # Start libcamera-vid with a pipe to continuously stream video frames
    cmd = [
        'libcamera-vid',
        '--width', str(width),
        '--height', str(height),
        '--framerate', '30',  # Set frame rate for video
        '--timeout', '0',  # Run indefinitely
        '--output', '/dev/shm/stream.mjpeg',  # Use shared memory for faster access
        '--codec', 'mjpeg'
    ]
    
    # Start the subprocess to run the video stream
    return subprocess.Popen(cmd)

def clear_and_recreate_shm_file():
    # Remove the file if it exists
    subprocess.run(['rm', '-f', '/dev/shm/stream.mjpeg'], check=True)
    
    # Create an empty file
    #subprocess.run(['touch', '/dev/shm/stream.mjpeg'], check=True)
    
def clear_ffmpeg_buffer():
    # Command to clear the buffer by discarding all old frames
    cmd = [
        'ffmpeg',
        '-f', 'mjpeg',
        '-i', '/dev/shm/stream.mjpeg',  # Input stream
        '-f', 'null', '-'  # Discard output
    ]
    
    # Run the command
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)

def initialize_capture(libcamera_process=None):
    # If the libcamera-vid process is running, terminate it
    if libcamera_process is not None:
        libcamera_process.terminate()
        #clear_and_recreate_shm_file
        clear_ffmpeg_buffer
    
    # Start a new libcamera-vid process
    libcamera_process = start_libcamera_vid()

    # Open the MJPEG stream using OpenCV
    cap = cv2.VideoCapture('/dev/shm/stream.mjpeg')
    
    if not cap.isOpened():
        print("Failed to open stream.")
    
    return cap, libcamera_process

# Start the video stream (this runs in the background)
libcamera_process = start_libcamera_vid()

# Wait a short time to let the stream initialize
time.sleep(2)

# Initialize OpenCV video capture
cap, libcamera_process = initialize_capture()

while True:
    ret, frame = cap.read()
    
    if ret:
        # Display the live video feed
        cv2.imshow("Live Stream", frame)
    else:
        print("Failed to read frame. Reinitializing stream...")
        # Reinitialize OpenCV capture and restart libcamera-vid if the frame reading fails
        cap.release()
        cap, libcamera_process = initialize_capture(libcamera_process)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Stop the libcamera-vid process
libcamera_process.terminate()

# Release the video capture and close OpenCV windows
cap.release()
cv2.destroyAllWindows()
