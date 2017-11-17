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
        #thresh = cv2.inRange(hsv,np.array((0, 200, 200)), np.array((20, 255, 255))
        red_lower=np.array([136,87,111],np.uint8)
        red_upper=np.array([180,255,255],np.uint8)
        blue_lower=np.array([99,115,150],np.uint8)
        blue_upper=np.array([110,255,255],np.uint8)
      

        thresh_r = cv2.inRange(blur, red_lower, red_upper)
        thresh_b = cv2.inRange(blur, blue_lower, blue_upper)
        
        # find contours in the threshold image
        contours_r,h_r = cv2.findContours(thresh_r,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        contours_b,h_b = cv2.findContours(thresh_b,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

        # finding contour with maximum area and store it as best_cnt
        max_area_r = 0
        best_cnt_r = 1
        for cnt_r in contours_r:
                area_r = cv2.contourArea(cnt_r)
                if area_r > max_area_r:
                        max_area_r = area_r
                        best_cnt_r = cnt_r

        # finding centroids of best_cnt and draw a circle there
        M_r = cv2.moments(best_cnt_r)
        cx_r,cy_r = int(M_r['m10']/M_r['m00']), int(M_r['m01']/M_r['m00'])
        print cx_r
        print cy_r
        #if best_cnt>1:
        cv2.circle(image,(cx_r,cy_r),10,(0,0,255),-1)
        # finding contour with maximum area and store it as best_cnt
        max_area_b = 0
        best_cnt_b = 1
        for cnt_b in contours_b:
                area_b = cv2.contourArea(cnt_b)
                if area_b > max_area_b:
                        max_area_b = area_b
                        best_cnt_b = cnt_b

        # finding centroids of best_cnt and draw a circle there
        M_b = cv2.moments(best_cnt_b)
        cx_b,cy_b = int(M_b['m10']/M_b['m00']), int(M_b['m01']/M_b['m00'])
        print cx_b
        print cy_b
        #if best_cnt>1:
        cv2.circle(image,(cx_b,cy_b),10,(0,0,255),-1)
        
        # show the frame
        cv2.imshow("Frame", image)
        #cv2.imshow('thresh',thresh2)
        key = cv2.waitKey(1) & 0xFF
 
	# clear the stream in preparation for the next frame
        rawCapture.truncate(0)
 
	# if the `q` key was pressed, break from the loop
        if key == ord("q"):
        	break
