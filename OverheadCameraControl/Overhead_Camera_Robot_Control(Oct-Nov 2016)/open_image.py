import cv2
import numpy as np
import picamera
import picamera.array
from picamera import PiCamera
from picamera.array import PiRGBArray
import time

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 50
camera.hflip = True
camera.vflip = True


with picamera.PiCamera() as camera:
     rawCap=picamera.array.PiRGBArray(camera,size=(640,480))
     camera.start_preview()
     time.sleep(3)
     camera.capture(rawCap,format='bgr')
     image=rawCap.array
cv2.imshow('test',image)
if cv2.waitKey(1) & 0xFF=='q':
           cv2.destroyAllWindows()
