import numpy as np
import cv2
import cv2.cv as cv
from heapq import *
import time
import math
#import serial

def heuristic(a, b):
	return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2

def astar(array, start, goal):

	#neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)] # 8 - point connectivity
	neighbors = [(0,1),(0,-1),(1,0),(-1,0)] # 4 - point connectivity

	close_set = set()
	came_from = {}
	gscore = {start:0}
	fscore = {start:heuristic(start, goal)}
	oheap = []

	heappush(oheap, (fscore[start], start))
	
	while oheap:

		current = heappop(oheap)[1]

		if current == goal:
			data = []
			while current in came_from:
				data.append(current)
				current = came_from[current]
			return data

		close_set.add(current)
		for i, j in neighbors:
			neighbor = current[0] + i, current[1] + j            
			tentative_g_score = gscore[current] + heuristic(current, neighbor)
			if 0 <= neighbor[0] < array.shape[0]:
				if 0 <= neighbor[1] < array.shape[1]:                
					if array[neighbor[0]][neighbor[1]] == 1:
						continue
				else:
					# array bound y walls
					continue
			else:
				# array bound x walls
				continue
				
			if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
				continue
				
			if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
				came_from[neighbor] = current
				gscore[neighbor] = tentative_g_score
				fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
				heappush(oheap, (fscore[neighbor], neighbor))
				
	return False

'''Here is an example of using my algo with a np array,
   astar(array, start, destination)
   astar function returns a list of points (shortest path)'''

cap=cv2.VideoCapture(0)
#ret=cap.set(5,25)
#cap.open(0)
time.sleep(2)


while(cap.isOpened()):
    
    ret,frame=cap.read()
    if ret==True:
        key=cv2.waitKey(1)
        
    #print cap.get(5)
    #print cap.get(4)
    #if frame.all():
    #try:
    #cv2.imshow('frame',frame)
    if key & 0xFF==ord('s'):
    	cv2.imwrite('C:\Python27\Lib\idlelib\map_test\ test_map_img.jpg',frame)
    #print "test_img{0}".format(i)
    #i=i+1 
    cv2.imshow('Frame4Arena',frame)
    if key & 0xFF==ord('q'):
        break
    #except error:
    #print 'N.A.'

cap.release()
cv2.destroyAllWindows()

arena=cv2.imread('C:\Python27\Lib\idlelib\map_test\ test_map_img.jpg')
arena_1=arena.copy()
cv2.imshow('arena',arena)

h_m,w_m,ch=arena.shape

print h_m,w_m

#h,w=input(" Enter no. of rows(height) and columns(width) (h,w) : ")


# h_d=h_m/h
# w_d=w_m/w

# print "h_d,w_d",h_d,w_d

h_in=0
w_in=0

h=0
w=0

def nothing(x):
	pass

cv2.namedWindow('Grid')
cv2.namedWindow('arena_grid')

cv2.createTrackbar('Height','Grid',1,50,nothing)
cv2.createTrackbar('Width','Grid',1,50,nothing)

# create switch for ON/OFF functionality
switch = '0 : OFF \n1 : ON'
cv2.createTrackbar(switch, 'Grid',0,1,nothing)

while 1:
	
	cv2.imshow('arena_grid',arena)
	arena=arena_1.copy()
	
	h=cv2.getTrackbarPos('Height','Grid')
	print 'h',h
	
	w=cv2.getTrackbarPos('Width','Grid')
	print 'w',w
	
	switch = '0 : OFF \n1 : ON'
	s=cv2.createTrackbar(switch, 'Grid',0,1,nothing)
	
	k = cv2.waitKey(1) & 0xFF
	
	if k == 27:
		break
	if s == 0:
		img[:] = 0
	else:
		h_d=h_m/h
		w_d=w_m/w

		print "h_d,w_d",h_d,w_d

		h_in=0
		w_in=0
		
		while(h_in <= 479):
			cv2.line(arena, (0,h_in), (w_m,h_in), (0, 255, 0), 2)
			h_in=h_in + (h_d - 1)

		while(w_in <= 639):
			cv2.line(arena, (w_in,0), (w_in,h_m), (255, 0, 0), 2)
			w_in=w_in + (w_d - 1)

cv2.destroyAllWindows()

#print "h,w : ",h,w
#print "h_d,w_d : ",h_d,w_d
h_in=0
w_in=0
h_d_list=[]
w_d_list=[]
while(h_in <= 479):
	h_d_list.append(h_in)
	h_in=h_in + (h_d - 1)

while(w_in <= 639):
	w_d_list.append(w_in)
	w_in=w_in + (w_d - 1)

# print 'h_d_list',h_d_list
# print 'w_d_list',w_d_list

# print 'len(h_d_list)',len(h_d_list)
# print 'len(w_d_list)',len(w_d_list)


test=np.ones((h_m,w_m,ch),np.uint8)
for i in h_d_list:
	print i
	cv2.line(test, (0,i), (w_m,i), (0, 255, 0), 2)

for j in w_d_list:
	print j
	cv2.line(test, (j,0), (j,h_m), (255, 0, 0), 2)

nmap=np.zeros((h,w))
print nmap

def select_obstacle(event,x,y,flags,param):
	if event == cv2.EVENT_LBUTTONDBLCLK:
		cv2.circle(test,(x,y),5,(0,0,255),-1)
		print x,y
		for i in range(0,len(h_d_list)-1):
			if y>h_d_list[i] and y<h_d_list[i+1] :
				for j in range(0,len(w_d_list)-1):
					if x>w_d_list[j] and x<w_d_list[j+1] :
						nmap[i][j]=1
						print nmap
# Create a black image, a window and bind the function to window
#print nmap
#img = np.zeros((100,512,3), np.uint8)
cv2.namedWindow('image')
cv2.setMouseCallback('image',select_obstacle)
while(1):
	cv2.imshow('image',test)
	if cv2.waitKey(20) & 0xFF == 27:
		break
	

print "final nmap"
print nmap
#cv2.imshow('test',test)

s=(0,0)
d=(0,0)

#h_n,w_n=nmap.shape

#print nmap
#n=input(" Enter 1(Obstacle) or 0(Path) : ")
#for i in range(0,h_m):
#    for j in range(0,w_m):
#        print i,j
#        n=input(" Enter 1(Obstacle) or 0(Path) : ")
#        nmap[i][j]=n

#print nmap

in_map=np.ones((h,w,3),np.uint8)
out_map=in_map
	
for i in range(0,h):
	for j in range(0,w):
		if nmap[i,j]==1 :
			in_map[i,j]=[0,0,255] 

out_map=in_map
#s = input('Enter source(h,w) : ')
#d = input('Enter destination(h,w) : ')

#cv2.destroyAllWindows()

# cv2.imshow('in_map',in_map)


# ans = astar(nmap, s, d)

# for c in ans:
# 	out_map[c[0],c[1]]=[255,0,0]


# cv2.imshow('out_map',out_map)

print "h,w : ",h,w

print 'h_d_list',h_d_list
print 'w_d_list',w_d_list

print 'len(h_d_list)',len(h_d_list)
print 'len(w_d_list)',len(w_d_list)

# print ans
# print s

# path=ans
# path.append(s)

# print 'ans',ans
# print 'path',path


kernel = np.ones((5,5),np.uint8)

# Take input from webcam
cap = cv2.VideoCapture(0)
time.sleep(2)
#ser = serial.Serial('COM4', 9600, timeout=0)
_,img_0=cap.read()
#print img_0.shape
h,w,_=img_0.shape
centre_x=w/2
centre_y=h/2
print w,h

cx1=0
cy1=0
cx2=0
cy2=0

f=0
t=0

path=[]
flag_path=0

robot_ang_deg=0
path_cell_ang_deg=0

# Reduce the size of video to 320x240 so rpi can process faster
#cap.set(3,320)
#cap.set(4,240)

def nothing(x):
	pass
# Creating a windows for later use
#cv2.namedWindow('HueComp')
#cv2.namedWindow('SatComp')
#cv2.namedWindow('ValComp')
#cv2.namedWindow('closing1')
#cv2.namedWindow('tracking1')
#cv2.namedWindow('frame')


# Creating track bar for min and max for hue, saturation and value
# You can adjust the defaults as you like
#cv2.createTrackbar('hmin', 'HueComp',12,179,nothing)
#cv2.createTrackbar('hmax', 'HueComp',37,179,nothing)

#cv2.createTrackbar('smin', 'SatComp',96,255,nothing)
#cv2.createTrackbar('smax', 'SatComp',255,255,nothing)

#cv2.createTrackbar('vmin', 'ValComp',186,255,nothing)
#cv2.createTrackbar('vmax', 'ValComp',255,255,nothing)

# My experimental values
# hmn1 = 12
# hmx1 = 37
# smn1 = 145
# smx1 = 255
# vmn1 = 186
# vmx1 = 255

# RED : Lower - 130,83,105  Higher - 180,255,255
# BLUE : Lower - 94,110,145  Higher - 115,255,255
# YELLOW : Lower - 22,60,200  Higher - 60,255,255

cv2.namedWindow('Color 1')
cv2.createTrackbar('H1 Lower','Color 1',121,180,nothing)
cv2.createTrackbar('H1 Higher','Color 1',180,180,nothing)
cv2.createTrackbar('S1 Lower','Color 1',191,255,nothing)
cv2.createTrackbar('S1 Higher','Color 1',255,255,nothing)
cv2.createTrackbar('V1 Lower','Color 1',165,255,nothing)
cv2.createTrackbar('V1 Higher','Color 1',255,255,nothing)

cv2.namedWindow('Color 2')
cv2.createTrackbar('H2 Lower','Color 2',94,180,nothing)
cv2.createTrackbar('H2 Higher','Color 2',115,180,nothing)
cv2.createTrackbar('S2 Lower','Color 2',250,255,nothing)
cv2.createTrackbar('S2 Higher','Color 2',158,255,nothing)
cv2.createTrackbar('V2 Lower','Color 2',255,255,nothing)
cv2.createTrackbar('V2 Higher','Color 2',255,255,nothing)

cv2.namedWindow('Movement Sensitivity(Reduces)')
cv2.createTrackbar('Forward','Movement Sensitivity(Reduces)',10,30,nothing)
cv2.createTrackbar('Turn','Movement Sensitivity(Reduces)',30,50,nothing)

while(cap.isOpened()):

	ret,frame=cap.read()
	if ret==True:
		k=cv2.waitKey(1)

	

	#converting to HSV
	hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
	hue,sat,val = cv2.split(hsv)

	hmn1=cv2.getTrackbarPos('H1 Lower','Color 1')
	smn1=cv2.getTrackbarPos('S1 Lower','Color 1')
	vmn1=cv2.getTrackbarPos('V1 Lower','Color 1')
	hmx1=cv2.getTrackbarPos('H1 Higher','Color 1')
	smx1=cv2.getTrackbarPos('S1 Higher','Color 1')
	vmx1=cv2.getTrackbarPos('V1 Higher','Color 1')

	hmn2=cv2.getTrackbarPos('H2 Lower','Color 2')
	smn2=cv2.getTrackbarPos('H2 Higher','Color 2')
	vmn2=cv2.getTrackbarPos('S2 Lower','Color 2')
	hmx2=cv2.getTrackbarPos('S2 Higher','Color 2')
	smx2=cv2.getTrackbarPos('V2 Lower','Color 2')
	vmx2=cv2.getTrackbarPos('V2 Higher','Color 2')

	f=cv2.getTrackbarPos('Forward','Movement Sensitivity(Reduces)')
	t=cv2.getTrackbarPos('Turn','Movement Sensitivity(Reduces)')

	# Apply thresholding
	hthresh1 = cv2.inRange(np.array(hue),np.array(hmn1),np.array(hmx1))
	sthresh1 = cv2.inRange(np.array(sat),np.array(smn1),np.array(smx1))
	vthresh1 = cv2.inRange(np.array(val),np.array(vmn1),np.array(vmx1))

	hthresh2 = cv2.inRange(np.array(hue),np.array(hmn2),np.array(hmx2))
	sthresh2 = cv2.inRange(np.array(sat),np.array(smn2),np.array(smx2))
	vthresh2 = cv2.inRange(np.array(val),np.array(vmn2),np.array(vmx2))

	# AND h s and v
	tracking1 = cv2.bitwise_and(hthresh1,cv2.bitwise_and(sthresh1,vthresh1))

	# Some morpholigical filtering
	erosion1=cv2.erode(tracking1,kernel,iterations = 1)
	dilation1 = cv2.dilate(erosion1,kernel,iterations = 1)
	closing1 = cv2.morphologyEx(dilation1, cv2.MORPH_CLOSE, kernel)
	closing1 = cv2.GaussianBlur(closing1,(5,5),0)

	# AND h s and v
	tracking2 = cv2.bitwise_and(hthresh2,cv2.bitwise_and(sthresh2,vthresh2))

	# Some morpholigical filtering
	erosion2=cv2.erode(tracking2,kernel,iterations = 2)
	dilation2 = cv2.dilate(erosion2,kernel,iterations = 2)
	closing2 = cv2.morphologyEx(dilation2, cv2.MORPH_CLOSE, kernel)
	closing2 = cv2.GaussianBlur(closing2,(5,5),0)

	# Detect circles using HoughCircles
	circles1 = cv2.HoughCircles(closing1,cv.CV_HOUGH_GRADIENT,2,120,param1=120,param2=50,minRadius=10,maxRadius=0)
	circles2 = cv2.HoughCircles(closing2,cv.CV_HOUGH_GRADIENT,2,120,param1=120,param2=50,minRadius=10,maxRadius=0)
	# circles = np.uint16(np.around(circles))

	#Draw Circles
	if circles1 is not None:
			for c1 in circles1[0,:]:
				# If the ball is far, draw it in green
				#if int(round(i[2])) < 30:
				cv2.circle(frame,(int(round(c1[0])),int(round(c1[1]))),int(round(c1[2])),(255,0,255),5)
				cv2.circle(frame,(int(round(c1[0])),int(round(c1[1]))),2,(255,0,255),10)
				cx1=int(round(c1[0]))
				cy1=int(round(c1[1]))

				# else draw it in red
				#elif int(round(i[2])) > 35:
				#    cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),int(round(i[2])),(0,0,255),5)
				#    cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),2,(0,0,255),10)
				#    buzz = 1

	if circles2 is not None:
			for c2 in circles2[0,:]:
				# If the ball is far, draw it in green
				#if int(round(i[2])) < 30:
				cv2.circle(frame,(int(round(c2[0])),int(round(c2[1]))),int(round(c2[2])),(255,255,0),5)
				cv2.circle(frame,(int(round(c2[0])),int(round(c2[1]))),2,(255,255,0),10)
				cx2=int(round(c2[0]))
				cy2=int(round(c2[1]))

				# else draw it in red
				#elif int(round(i[2])) > 35:
				#    cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),int(round(i[2])),(0,0,255),5)
				#    cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),2,(0,0,255),10)
				#    buzz = 1

	#you can use the 'buzz' variable as a trigger to switch some GPIO lines on Rpi :)
	# print buzz                    
	# if buzz:
		# put your GPIO line here

	#cv2.circle(img,(cx1,cy1),10,(255,0,0),-1)
	
	#print "cx1,cy1 : ",cx1,cy1
	#print "cx2,cy2 : ",cx2,cy2
	
	for i in h_d_list:
		#print i
		cv2.line(frame, (0,i), (w_m,i), (0, 255, 0), 2)

	for j in w_d_list:
		#print j
		cv2.line(frame, (j,0), (j,h_m), (255, 0, 0), 2)

	for i1 in range(0,len(h_d_list)-1):
		if cy1>h_d_list[i1] and cy1<h_d_list[i1+1] :
			for j1 in range(0,len(w_d_list)-1):
				if cx1>w_d_list[j1] and cx1<w_d_list[j1+1] :
					#print "circle1 cell : ",i1,j1
					s=(i1,j1)
					r=(i1,j1)
	#print 'i1,j1_s',s[0],s[1]
	cv2.putText(frame,str(s),(500,450),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,255),2,cv2.CV_AA)
	#print 'i1,j1_puttext',s[0],s[1]
	if k & 0xFF==ord('a'):
		
		d = input('Enter destination(h,w) : ')
		cv2.imshow('in_map',in_map)
		path = astar(nmap, s, d)
		#print 'i1,j1_astar',s[0],s[1]
		path.append(s)
		#print 'i1,j1_astar_2',s[0],s[1]
		print path
		for c in path:
			out_map[c[0],c[1]]=[255,0,0]
		cv2.imshow('out_map',out_map)
		path.reverse()
		#print 'i1,j1_astar_2',s[0],s[1]
		print 'Path : ',path
		flag_path=1

	
	if flag_path==1 :
		#print 'i1,j1_flag_path',s[0],s[1]
		if (cy1>h_d_list[d[0]] and cy1<h_d_list[d[0]+1] and cx1>w_d_list[d[1]] and cx1<w_d_list[d[1]+1]) is False :
			#print 'i1,j1_if_d',s[0],s[1]
			
			d_2dx=cx2-cx1;d_2dy=cy2-cy1;d_1dx=0-cx1;d_1dy=0-cy1;
			
			rad1=math.atan2(d_1dy,d_1dx) ; rad2=math.atan2(d_2dy,d_2dx)
			robot_ang_deg=(rad2-rad1)*180/3.14;
			
			cv2.putText(frame,str(robot_ang_deg),(50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.CV_AA)
			cv2.line(frame, (cx1,cy1), (cx2,cy2), (255, 255, 0), 2) ; cv2.line(frame, (cx1,cy1), (0,0), (255, 255, 0), 2)
			
			# angle between  current and next cell
			#print 'i1,j1_after_angle',s[0],s[1]

			for p in range(0,len(path)):
				#print 'path[p]',path[p][0],path[p][1]
				#print 'i1,j1',i1,j1
				if r[0]==path[p][0] and r[1]==path[p][1]:
					#print 'i1,j1',i1,j1

					p1_x=(w_d_list[path[p][1]]) + ((w_d_list[(path[p][1])+1] - w_d_list[path[p][1]])/2)
					p1_y=(h_d_list[path[p][0]]) + ((h_d_list[(path[p][0])+1] - h_d_list[path[p][0]])/2)

					p2_x=(w_d_list[path[p+1][1]]) + ((w_d_list[(path[p+1][1])+1] - w_d_list[path[p+1][1]])/2)
					p2_y=(h_d_list[path[p+1][0]]) + ((h_d_list[(path[p+1][0])+1] - h_d_list[path[p+1][0]])/2)

					d_2dx_p=p2_x-p1_x; d_2dy_p=p2_y-p1_y; d_1dx_p=0-p1_x; d_1dy_p=0-p1_y;
					
					rad1_p=math.atan2(d_1dy_p,d_1dx_p) ; rad2_p=math.atan2(d_2dy_p,d_2dx_p)
					path_cell_ang_deg=(rad2_p-rad1_p)*180/3.14;

					cv2.circle(frame,(p1_x,p1_y),10,(255,0,0),10) ; cv2.circle(frame,(p2_x,p2_y),10,(0,0,255),10)
					cv2.line(frame,(p1_x,p1_y),((p2_x,p2_y)), (0, 255, 0), 2) ; cv2.line(frame,(p1_x,p1_y),(0,0), (255, 255, 255), 2)

					cv2.putText(frame,str(path_cell_ang_deg),(50,100),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2,cv2.CV_AA)
	
			if robot_ang_deg<(path_cell_ang_deg+f) and robot_ang_deg>(path_cell_ang_deg-f) :
			#if robot_ang_deg==(path_cell_ang_deg) :
				print 'Forward'
				cv2.putText(frame,"Forward",(400,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.CV_AA)
				#ser.write(chr(1))
			else :
				
				if robot_ang_deg>(path_cell_ang_deg+t) :
					print 'Left'
					cv2.putText(frame,"Left",(400,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.CV_AA)
				elif robot_ang_deg<(path_cell_ang_deg-t) :
					print 'Right'
					cv2.putText(frame,"Right",(400,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.CV_AA)
				else:
					print "Don't Move"
					cv2.putText(frame,"Don't Move",(400,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.CV_AA)


			# elif robot_ang_deg!=path_cell_ang_deg :
			# 	print "Stop!"
			# 	#ser.write(chr(0))
			# 	if <condition> :
			# 		left turn
			# 	else:
			# 		right turn
		elif (cy1>h_d_list[d[0]] and cy1<h_d_list[d[0]+1] and cx1>w_d_list[d[1]] and cx1<w_d_list[d[1]+1]) is True :
			print "Stop!"
			#ser.write(chr(0))
			print "Robot has reached the destination ! "

	cv2.imshow('frame',frame)
	cv2.imshow('closing1',closing1)
	cv2.imshow('closing2',closing2)

	#k = cv2.waitKey(1) & 0xFF
	if k == 27:
		break

cap.release()
cv2.destroyAllWindows()