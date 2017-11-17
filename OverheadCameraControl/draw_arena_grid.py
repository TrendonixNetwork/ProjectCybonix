import numpy as np
import cv2
from heapq import *

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
for i in range(0,len(h_d_list)):
	print h_d_list[i]
	cv2.line(test, (0,h_d_list[i]), (w_m,h_d_list[i]), (0, 255, 0), 2)

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



#h_n,w_n=nmap.shape

print nmap
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
cv2.imshow('in_map',in_map)

s = input('Enter source(h,w) : ')
d = input('Enter destination(h,w) : ')

ans = astar(nmap, s, d)

for c in ans:
    out_map[c[0],c[1]]=[255,0,0]


cv2.imshow('out_map',out_map)

print "h,w : ",h,w

print 'h_d_list',h_d_list
print 'w_d_list',w_d_list

print 'len(h_d_list)',len(h_d_list)
print 'len(w_d_list)',len(w_d_list)

print ans
print s
cv2.waitKey(0)
cv2.destroyAllWindows()