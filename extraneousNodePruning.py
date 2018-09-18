import math, sys, pygame, random
from math import *
from pygame import *

obs = []
obs.append(((150,100),18))
obs.append(((150,130),18))
obs.append(((150,160),18))
obs.append(((150,190),18))
obs.append(((150,220),18))
obs.append(((150,250),18))
obs.append(((180,250),18))
obs.append(((210,250),18))
obs.append(((240,250),18))
obs.append(((270,250),18))
obs.append(((300,250),18))
obs.append(((330,250),18))
obs.append(((330,220),18))
obs.append(((330,190),18))
obs.append(((330,160),18))
obs.append(((330,130),18))
obs.append(((330,100),18))
obs.append(((300,100),18))
obs.append(((270,100),18))
obs.append(((240,100),18))
obs.append(((240,130),18))
obs.append(((240,160),18))
obs.append(((240,190),18))

def dist(p1,p2):    #distance between two points
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

def lineCollides(p1, p2, obstacles=None):
    global obs
    if(obstacles == None):
        obstacles = obs
    else:
        obstacles = [obstacles]
    list = []
    a = p2[1] - p1[1]
    b = -(p2[0] - p1[0])
    c = -b * p2[1] - a * p2[0]
    for circle in obstacles:
        cen, rad = circle
        val = (a * cen[0] + b * cen[1] + c * 1.0) / (a * a + b * b)
        val = -val
        h = (val * a) + cen[0]
        k = (val * b) + cen[1]
        #pygame.draw.circle(screen, (255,255,0), (round(h),round(k)), 1)
        if dist(cen,(h,k)) < 1.1*rad:
            if((h<= max(p1[0],p2[0]) and h >= min(p1[0],p2[0])) and (k<= max(p1[1],p2[1]) and k >= min(p1[1],p2[1]))):
                return True
    for i in list:
        pygame.draw.circle(screen, (255,255,0), (round(i[0]),round(i[1])), 1)
    return False

def pruning(inputPath):
	path = inputPath[::]
	n = len(path)
	i=1
	removed = 0
	while(i<n-1-removed):
		if(lineCollides(path[i-1],path[i+1])==False):
			print("Removed",i)
			path.pop(i)
			removed+=1
		else:
			i+=1
	return path
			
path = [(200,200),(205,215),(205, 215), (172.3643522123525, 91.94675206036877), (156.89667097170965, 79.26799671775464), 
(136.94673206738491, 80.68218825842729), (111.8586581467608, 102.59362572338374), (96, 422)]
print(path)
path = pruning(path)
print(path)