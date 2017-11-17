from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import cv
import numpy as np

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 50
camera.hflip = True
camera.vflip = True
rawCapture = PiRGBArray(camera, size=(640, 480))

time.sleep(0.1)

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array
    image = cv2.GaussianBlur(image,(5,5),0)
    image = cv2.medianBlur(image,5)
    g_img = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    gray = cv2.adaptiveThreshold(g_img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
            cv2.THRESH_BINARY,11,3.5)
    kernel = np.ones((2.6,2.7),np.uint8)
    g_img = cv2.erode(g_img,kernel,iterations = 1)
	# gray = erosion
	
    g_img = cv2.dilate(g_img,kernel,iterations = 1)
	# gray = dilation
    circles = cv2.HoughCircles(g_img,cv.CV_HOUGH_GRADIENT,1,20,
                        param1=50,param2=30,minRadius=0,maxRadius=0)
    circles = np.uint16(np.around(circles))
    for i in circles[0,:]:
           # draw the outer circle
           cv2.circle(image,(i[0],i[1]),i[2],(0,255,0),2)
           # draw the center of the circle
           cv2.circle(image,(i[0],i[1]),2,(0,0,255),3)

    # show the frame
    cv2.imshow("Frame", image)
    key = cv2.waitKey(1) & 0xFF
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break

cv2.destroyAllWindows()
