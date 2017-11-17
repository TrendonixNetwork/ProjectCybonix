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
    img=frame.array
    image = cv2.GaussianBlur(img,(5,5),0)
    blur = cv2.medianBlur(img, 5)
    hsv=cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)

    #definig the range of red color
    red_lower=np.array([136,87,111],np.uint8)
    red_upper=np.array([180,255,255],np.uint8)

    #defining the Range of Blue color
    blue_lower=np.array([99,115,150],np.uint8)
    blue_upper=np.array([110,255,255],np.uint8)

    #finding the range of red,blue and yellow color in the image
    red=cv2.inRange(hsv, red_lower, red_upper)
    blue=cv2.inRange(hsv,blue_lower,blue_upper)

    #Morphological transformation, Dilation         
    kernel = np.ones((5 ,5), "uint8")

    red=cv2.dilate(red, kernel)
    res=cv2.bitwise_and(img, img, mask = red)

    blue=cv2.dilate(blue,kernel)
    res1=cv2.bitwise_and(img, img, mask = blue)

    #Tracking the red Color
    (contours,hierarchy)=cv2.findContours(red,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
    for  pic,contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area>300):
                    x,y,w,h = cv2.boundingRect(contour)
                    #print x
                    #cv2.circle(img,(x,y),10,(0,255,0),1)
                    cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)

    #Tracking the Blue Color
    (contours,hierarchy)=cv2.findContours(blue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    for pic,contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area>300):
                    x,y,w,h = cv2.boundingRect(contour)
                    #print x
                    #cv2.circle(img,(x,y),10,(0,255,0),1)
                    cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)

    cv2.imshow("Test",img)

    key = cv2.waitKey(1) & 0xFF
    rawCapture.truncate(0)
    if key == ord('q'):
         cap.release()
         cv2.destroyAllWindows()
         break
