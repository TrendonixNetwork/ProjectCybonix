import numpy as np
from heapq import *
import cv2


def heuristic(a, b):
    return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2

def astar(array, start, goal):

    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

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

nmap = np.array([
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [1,1,1,1,1,1,1,1,1,1,1,1,0,1],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [1,0,1,1,1,1,1,1,1,1,1,1,1,1],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [1,1,1,1,1,1,1,1,1,1,1,1,0,1],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [1,0,1,1,1,1,1,1,1,1,1,1,1,1],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [1,1,1,1,1,1,1,1,1,1,1,1,0,1],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0]])

#dimensions=input(" Enter no. of rows(height) and columns(width) (h,w) : ")
#nmap=np.zeros(dimensions)
#nmap=np.zeros((11,14))

h_m,w_m=nmap.shape

print nmap
#n=input(" Enter 1(Obstacle) or 0(Path) : ")
#for i in range(0,h_m):
#    for j in range(0,w_m):
#        print i,j
#        n=input(" Enter 1(Obstacle) or 0(Path) : ")
#        nmap[i][j]=n

#print nmap

in_map=np.ones((h_m,w_m,3),np.uint8)
out_map=in_map
    
for i in range(0,h_m):
    for j in range(0,w_m):
        if nmap[i,j]==1 :
            in_map[i,j]=[0,0,255] 
cv2.imshow('in_map',in_map)

s = input('Enter source(h,w) : ')
d = input('Enter destination(h,w) : ')

ans = astar(nmap, s, d)

for c in ans:
    out_map[c[0],c[1]]=[255,0,0]


cv2.imshow('out_map',out_map)
cv2.waitKey(0)
cv2.destroyAllWindows()

print ans
