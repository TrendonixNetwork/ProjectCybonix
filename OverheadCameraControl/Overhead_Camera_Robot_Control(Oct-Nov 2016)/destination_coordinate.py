# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2



# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
m=-1
n=-1
drawing=False
def draw_circle(event,x,y,flags,param):
    global image,m,n
    if event==cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(image,(x,y),50,(255,0,0),1)
        m,n=x,y



# allow the camera to warmup
time.sleep(0.1)
cv2.namedWindow('image')
cv2.setMouseCallback('image',draw_circle)
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array
    cv2.circle(image,(m,n),50,(255,0,0),1)
    # show the frame
    cv2.imshow('image', image)
    key = cv2.waitKey(1) & 0xFF
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
