from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

camera=cv2.VideoCapture(0)

while True:
    ret,frame=cam.read()
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF ==ord('q'):
        break

cam.release()
cv2.destroyAllWindows()
