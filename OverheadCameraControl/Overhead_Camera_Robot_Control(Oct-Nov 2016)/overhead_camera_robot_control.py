# import the necessary packages
#import division from _future_ 
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import math
 
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 50
camera.hflip = True
camera.vflip = True

rawCapture = PiRGBArray(camera, size=(640, 480))
m=-1
n=-1
drawing=False
def draw_circle(event,x,y,flags,param):
    global image,m,n
    if event==cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(img,(x,y),50,(255,0,0),1)
        m,n=x,y
def nothing(x):
    pass
 
# allow the camera to warmup
time.sleep(0.1)

#float x1,y1,x2,y2,x3,y3;
#float p1,p2,q1,q2;
#ang_rad=0.0
#angle=0.0
theta_ang=0.0

cv2.namedWindow('Color 1')
cv2.namedWindow('Color 2')
cv2.createTrackbar('H1 Lower','Color 1',0,180,nothing)
cv2.createTrackbar('H1 Higher','Color 1',0,180,nothing)
cv2.createTrackbar('S1 Lower','Color 1',0,255,nothing)
cv2.createTrackbar('S1 Higher','Color 1',0,255,nothing)
cv2.createTrackbar('V1 Lower','Color 1',0,255,nothing)
cv2.createTrackbar('V1 Higher','Color 1',0,255,nothing)
cv2.createTrackbar('H2 Lower','Color 2',0,180,nothing)
cv2.createTrackbar('H2 Higher','Color 2',0,180,nothing)
cv2.createTrackbar('S2 Lower','Color 2',0,255,nothing)
cv2.createTrackbar('S2 Higher','Color 2',0,255,nothing)
cv2.createTrackbar('V2 Lower','Color 2',0,255,nothing)
cv2.createTrackbar('V2 Higher','Color 2',0,255,nothing)
cv2.namedWindow('Test')
cv2.setMouseCallback('Test',draw_circle)
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):      
        #converting frame(img i.e BGR) to HSV (hue-saturation-value)
        img = frame.array
        

        hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        

        #definig the range of red color
        h1_l=cv2.getTrackbarPos('H1 Lower','Color 1')
        s1_l=cv2.getTrackbarPos('S1 Lower','Color 1')
        v1_l=cv2.getTrackbarPos('V1 Lower','Color 1')
        red_lower=np.array([h1_l,s1_l,v1_l],np.uint8)
        h1_h=cv2.getTrackbarPos('H1 Higher','Color 1')
        s1_h=cv2.getTrackbarPos('S1 Higher','Color 1')
        v1_h=cv2.getTrackbarPos('V1 Higher','Color 1')
        red_upper=np.array([h1_h,s1_h,v1_h],np.uint8)

        #defining the Range of Blue color
        h2_l=cv2.getTrackbarPos('H2 Lower','Color 2')
        s2_l=cv2.getTrackbarPos('S2 Lower','Color 2')
        v2_l=cv2.getTrackbarPos('V2 Lower','Color 2')
        blue_lower=np.array([h2_l,s2_l,v2_l],np.uint8)
        h2_h=cv2.getTrackbarPos('H2 Higher','Color 2')
        s2_h=cv2.getTrackbarPos('S2 Higher','Color 2')
        v2_h=cv2.getTrackbarPos('V2 Higher','Color 2')
        blue_upper=np.array([h2_h,s2_h,v2_h],np.uint8)
        
        #defining the Range of yellow color
        #yellow_lower=np.array([22,60,200],np.uint8)
        #yellow_upper=np.array([60,255,255],np.uint8)

        #finding the range of red,blue and yellow color in the image
        red=cv2.inRange(hsv, red_lower, red_upper)
        blue=cv2.inRange(hsv,blue_lower,blue_upper)
        #yellow=cv2.inRange(hsv,yellow_lower,yellow_upper)
        
        #Morphological transformation, Dilation         
        kernel = np.ones((5 ,5), "uint8")

        red=cv2.dilate(red, kernel)
        res=cv2.bitwise_and(img, img, mask = red)
        

        blue=cv2.dilate(blue,kernel)
        res1=cv2.bitwise_and(img, img, mask = blue)

        #yellow=cv2.dilate(yellow,kernel)
        #res2=cv2.bitwise_and(img, img, mask = yellow)    


        #Tracking the Red Color
        (contours,hierarchy)=cv2.findContours(red,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        max_area_r=0
        best_c_r=1
        for  pic,contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if area > max_area_r:
                        max_area_r = area
                        best_c_r = contour
                        x,y,w,h = cv2.boundingRect(best_c_r)
                        #print x
                        #cv2.circle(img,(x,y),10,(0,255,0),1)
                        cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)

        M1 = cv2.moments(best_c_r)
        cx1,cy1 = int(M1['m10']/M1['m00']), int(M1['m01']/M1['m00'])
        #cx1_float,cy1_float = (M1['m10']/M1['m00']),(M1['m01']/M1['m00'])
        #print cx1
        #print cy1
        #if best_cnt>1:
        cv2.circle(img,(cx1,cy1),10,(0,0,255),-1)
               
                        
        #Tracking the Blue Color
        (contours,hierarchy)=cv2.findContours(blue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        max_area_b=0
        best_c_b=1
        for  pic,contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if area > max_area_b:
                        max_area_b = area
                        best_c_b = contour
                        x,y,w,h = cv2.boundingRect(best_c_b)
                        #print x
                        #cv2.circle(img,(x,y),10,(0,255,0),1)
                        cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)

        M2 = cv2.moments(best_c_b)
        cx2,cy2 = int(M2['m10']/M2['m00']), int(M2['m01']/M2['m00'])
        #cx2_float,cy2_float = (M2['m10']/M2['m00']),(M2['m01']/M2['m00'])
        #print cx2
        #print cy2
        #if best_cnt>1:
        cv2.circle(img,(cx2,cy2),10,(255,0,0),-1)

        cv2.circle(img,(m,n),50,(0,255,0),5)
        cv2.line(img,(cx1,cy1),(cx2,cy2),(0,255,255),5)
        cv2.line(img,(cx2,cy2),(m,n),(0,255,255),5)

        try:
            dx21=cx2-cx1;dx2d=cx2-m;dy21=cy2-cy1;dy2d=cy2-n;
            m21=math.sqrt(dx21*dx21 + dy21*dy21)
            m2d=math.sqrt(dx2d*dx2d + dy2d*dy2d)
            theta_rad=np.rad2deg(math.acos((dx21*dx2d + dy21*dy2d)/((math.sqrt(dx21*dx21 + dy21*dy21))*(math.sqrt(dx2d*dx2d + dy2d*dy2d)))))
            #theta_ang=np.rad2deg(theta_rad)
            print theta_ang
        except ZeroDivisionError:
            print "N.A."
            

        #dist2=math.sqrt((cx2-m)*(cx2-m) + (cy2-n)*(cy2-n))
        #dist1=math.sqrt((cx2-cx1)*(cx2-cx1) + (cy2-cy1)*(cy2-cy1))
        #x2=cx2;y2=cy2;
        #if(dist1 < dist2):
        #    x3=cx1;y3=cy1;x1=m;y1=n;
        #else:
        #    x3=m;y3=n;x1=cx1;y1=cy1;
        #q1=x2-x1;q2=y2-y1;p1=x2-x3;p2=y2-y3;
        #try:
        #    ang_deg=(math.acos((p1*q1 + p2*q2)/(math.sqrt(p1*p1 + p2*p2) * math.sqrt(q1*q1 + q2*q2))))*180/3.14
        #except ZeroDivisionError:
        #    print "N.A."
        #print ang_deg
        
        
        #try:
        #   t_2d=(cy2-n)/(cx2-m)
        #    t_21=(cy2-cy1)/(cx2-cx1)
        #    angle=np.rad2deg(np.arctan((t_21-t_2d)/(1+t_21*t_2d)))
        #except ZeroDivisionError:
        #    print "N.A."
        #print angle

        #Tracking the yellow Color
        #(contours,hierarchy)=cv2.findContours(yellow,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        #for pic, contour in enumerate(contours):
        #        area = cv2.contourArea(contour)
        #        if(area>300):
        #                x,y,w,h = cv2.boundingRect(contour)     
        #                img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
        cv2.imshow("Test",img)
            
           
        #cv2.imshow("Redcolour",red)
        #cv2.imshow("Color Tracking",img)
        
        key = cv2.waitKey(1) & 0xFF
        rawCapture.truncate(0)
        #cv2.imshow("red",res)  
        if key == ord('q'):
                cap.release()
                cv2.destroyAllWindows()
                break  
          

    
