import numpy

def midPoint(p1,p2):
	x = (p1.x + p2.x)/2
	y = (p1.y + p2.y)
	return Point(x,y)

def newList(list,start,end):
	l = []
	for i in xrange(start,end):
		l.append(list[i])
	return l

def distanceOfLineFromCircle(a,b,c,circle): # ax+by+c = 0
	x = circle.center.x
	y = circle.center.y
	distance = (a*x + b*y + c)/sqrt(a*a + b*b)
	return distance

def collisionFinderCircularObject(p1,p2,data):
	#y = mx + c.  ax + by + c = 0.
	# Therefore a = m, b = -1.

	'''m = (p2.y - p1.y)/(p2.x - p1.x)
	a = m
	b = -1
	c = p2.y - (m*p2.x)'''
	a = p2.y - p1.y
	b = -(p2.x-p1.x)
	c = -b*p2.y - a*p2.x

	distances = []
	for i in data:
		d = distanceOfLineFromCircle(a,b,c,i)
		if (d <= i.radius):
		 	return true
		 	break
	return false

def collisionFinder(start,end,volume):
	if(start == end):
		return false
		break
	point = midPoint(start,end)

	if(volume(point) == 1):  # This means it is colliding.
		return true
		break
	else:
		collisionFinder(start,point,volume)
		collisionFinder(point,end,volume)

def collisionFinderSpline(path,volume):
	length = len(path)
	if (volume(path[length/2]) == 1):
		return true
		break
	else:
		path1 = newList(path,0,length/2)
		path2 = newList(path,(length/2),length)
		collisionFinderSpline(path1,volume)
		collisionFinderSpline(path2,volume)