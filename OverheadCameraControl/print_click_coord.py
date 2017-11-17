import cv2
import numpy as np

x=0
y=0
# mouse callback function
def draw_circle(event,x,y,flags,param):
	if event == cv2.EVENT_LBUTTONDBLCLK:
		cv2.circle(img,(x,y),5,(255,0,0),-1)
		print x,y
# Create a black image, a window and bind the function to window
img = np.zeros((100,512,3), np.uint8)
cv2.namedWindow('image')
cv2.setMouseCallback('image',draw_circle)
while(1):
	cv2.imshow('image',img)
	if cv2.waitKey(20) & 0xFF == 27:
		break
	
cv2.destroyAllWindows()