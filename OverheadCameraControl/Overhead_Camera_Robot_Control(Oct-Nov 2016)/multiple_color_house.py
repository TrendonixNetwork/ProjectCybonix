# -*- coding: utf-8 -*-
# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
 
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 50
camera.hflip = True
camera.vflip = True

rawCapture = PiRGBArray(camera, size=(640, 480))
 
# allow the camera to warmup
time.sleep(0.1)
 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
        image = frame.array

        blur = cv2.blur(image, (3,3))

        #hsv to complicate things, or stick with BGR
        hsv = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)
        #thresh = cv2.inRange(hsv,np.array((0, 200, 200)), np.array((20, 255, 255)))
        red_lower=np.array([136,87,111],np.uint8)
        red_upper=np.array([180,255,255],np.uint8)

        blue_lower=np.array([99,115,150],np.uint8)
        blue_upper=np.array([110,255,255],np.uint8)

        yellow_lower=np.array([22,60,200],np.uint8)
        yellow_upper=np.array([60,255,255],np.uint8)

        kernal= np.ones((5 ,5), 'uint8') #kernal is created of 5×5
        red=cv2.dilate(red, kernal)
        res=cv2.bitwise_and(image, image, mask = red)
        blue=cv2.dilate(blue,kernal)
        res1=cv2.bitwise_and(image, image, mask = blue)
        yellow=cv2.dilate(yellow,kernal)
        res2=cv2.bitwise_and(image, image, mask = yellow)
        #lower = np.array([76,31,4],dtype="uint8")
        #upper = np.array([225,88,50], dtype="uint8")
        #upper = np.array([210,90,70], dtype="uint8")

        thresh = cv2.inRange(blur, lower, upper)
        thresh2 = thresh.copy()

        # find contours in the threshold image
        contours,hierarchy = cv2.findContours(res2,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

        # finding contour with maximum area and store it as best_cnt
        max_area = 0
        best_cnt = 1
        for cnt in enumerate(contours):
                area = cv2.contourArea(cnt)
                if area > 300:
                        x_r,y_r,w,h=cv2.boundingRect(contour)
                        cv2.rectangle(image,(x_r,y_r),(x_r+w,y_r+h),(0,0,255),2)

        # finding centroids of best_cnt and draw a circle there
        M = cv2.moments(best_cnt)
        cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
        print cx
        print cy
        #if best_cnt>1:
        cv2.circle(blur,(cx,cy),10,(0,0,255),-1)
        # show the frame
        cv2.imshow("Frame", blur)
        #cv2.imshow('thresh',thresh2)
        key = cv2.waitKey(1) & 0xFF
 
	# clear the stream in preparation for the next frame
        rawCapture.truncate(0)
 
	# if the `q` key was pressed, break from the loop
        if key == ord("q"):
        	break
