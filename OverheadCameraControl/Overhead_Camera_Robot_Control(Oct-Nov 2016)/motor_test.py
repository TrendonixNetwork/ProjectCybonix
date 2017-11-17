# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
from time import sleep
 
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 50
camera.hflip = True
camera.vflip = True
GPIO.setmode(GPIO.BOARD)
 
Motor1A = 16
Motor1B = 18
Motor1E = 22
rawCapture = PiRGBArray(camera, size=(640, 480))

def motor_on():
     GPIO.setmode(GPIO.BOARD)
     GPIO.setup(Motor1A,GPIO.OUT)
     GPIO.setup(Motor1B,GPIO.OUT)
     GPIO.setup(Motor1E,GPIO.OUT)
     print "Turning motor on"
     GPIO.output(Motor1A,GPIO.HIGH)
     GPIO.output(Motor1B,GPIO.LOW)
     GPIO.output(Motor1E,GPIO.HIGH)
     sleep(5)
     print "Stopping motor"
     GPIO.output(Motor1E,GPIO.LOW)
     GPIO.cleanup()
 
# allow the camera to warmup
sleep(0.1)
 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
        image = frame.array

        image = cv2.GaussianBlur(image,(5,5),0)
        blur = cv2.medianBlur(image, 5)

        #hsv to complicate things, or stick with BGR
        hsv = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)
        #thresh = cv2.inRange(hsv,np.array((0, 200, 200)), np.array((20, 255, 255)))

        lower = np.array([110,50,50])
        #upper = np.array([225,88,50], dtype="uint8")
        upper = np.array([130,255,255])

        thresh = cv2.inRange(blur, lower, upper)
        thresh2 = thresh.copy()

        # find contours in the threshold image
        contours,hierarchy = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

        # finding contour with maximum area and store it as best_cnt
        max_area = 0
        best_cnt = 1
        for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > max_area:
                        max_area = area
                        best_cnt = cnt

        # finding centroids of best_cnt and draw a circle there
        M = cv2.moments(best_cnt)
        cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
        #print cx
        #print cy
        #if best_cnt>1:
        cv2.circle(blur,(cx,cy),10,(0,0,255),-1)
        # show the frame
        cv2.imshow("Frame", blur)
        motor_on()
        #cv2.imshow('thresh',thresh2)
        #cv2.imshow("Thresh", thresh)
        key = cv2.waitKey(1) & 0xFF
 
	# clear the stream in preparation for the next frame
        rawCapture.truncate(0)
 
	# if the `q` key was pressed, break from the loop
        if key == ord("q"):
        	break
