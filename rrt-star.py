import math, sys, pygame, random
from math import *
from pygame import *

class Node(object):
    def __init__(self, point, parent, cost=0):
        super(Node, self).__init__()
        self.point = point
        self.parent = parent
        self.cost = cost;

XDIM = 720
YDIM = 500
windowSize = [XDIM, YDIM]
delta = 10.0
GAME_LEVEL = 1
GOAL_RADIUS = 10
MIN_DISTANCE_TO_ADD = 1.0
NUMNODES = 5000
FPS = 1000
pygame.init()
fpsClock = pygame.time.Clock()
screen = pygame.display.set_mode(windowSize)
white = 255, 255, 255
black = 25, 25, 25
red = 255, 0, 0
blue = 0, 128, 0
green = 0, 0, 255
cyan = 0,180,105

count = 0
rectObs = []
obs = []

def dist(p1,p2):    #distance between two points
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

def point_circle_collision(p1, p2, radius):
    distance = dist(p1,p2)
    if (distance <= radius):
        return True
    return False

def step_from_to(p1,p2):
    if dist(p1,p2) < delta:
        return p2
    else:
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + delta*cos(theta), p1[1] + delta*sin(theta)

def collides(p):    #check if point collides with the obstacle
    for rect in rectObs:
        if rect.collidepoint(p) == True:
            return True
    return False
	
def pointCollides(p):
    for circle in obs:
        cen, rad = circle
        if(dist(p,cen)<=rad):
            return True
    return False

def lineCollides(p1, p2):
    a = p2[1] - p1[1]
    b = -(p2[0] - p1[0])
    c = -b * p2[1] - a * p2[0]
    for circle in obs:
        cen, rad = circle
        val = -(a * cen[0] + b * cen[1] + c) / (a * a + b * b);
        h = (val * a) + cen[0]
        k = (val * b) + cen[1]
        if dist(cen,(h,k)) < rad:
            return True
        '''else:
            t1 = (h - p1[0]) / (p2[0] - p1[0])
            t2 = (k - p1[1]) / (p2[1] - p1[1])
            if t1 != t2 or t1 < 0 or t1 > 1:
                return False'''
    return False


def get_random_clear():
    while True:
        p = random.random()*XDIM, random.random()*YDIM
        noCollision = collides(p)
        if noCollision == False:
            return p


def init_obstacles(configNum):  #initialized the obstacle
    global rectObs
    rectObs = []
    print("config "+ str(configNum))
    if (configNum == 0):
        rectObs.append(pygame.Rect((XDIM / 2.0 - 50, YDIM / 2.0 - 100),(100,200)))
    if (configNum == 1):
        rectObs.append(pygame.Rect((100,50),(200,150)))
        rectObs.append(pygame.Rect((400,200),(200,100)))
    if (configNum == 2):
        rectObs.append(pygame.Rect((100,50),(200,150)))
    if (configNum == 3):
        rectObs.append(pygame.Rect((100,50),(200,150)))

    for rect in rectObs:
        pygame.draw.rect(screen, black, rect)

def init_circular_obstacles(configNum=0):
    global obs
    obs.append(((150,100),20))
    obs.append(((150,130),20))
    obs.append(((150,160),20))
    obs.append(((150,190),20))
    obs.append(((150,220),20))
    obs.append(((150,255),20))
    '''obs.append(((150,100),20))
    obs.append(((200,150),150))'''
    for circ in obs:
        pygame.draw.circle(screen, black, circ[0], circ[1])

def reset():
    global count
    screen.fill(white)
    #init_obstacles(GAME_LEVEL)
    init_circular_obstacles()
    count = 0

def main():
    global count
    
    initPoseSet = False
    initialPoint = Node(None, None)
    goalPoseSet = False
    goalPoint = Node(None, None)
    currentState = 'init'

    nodes = []
    reset()

    while True:
        if currentState == 'init':
            pygame.display.set_caption('Select Starting Point and then Goal Point')
            fpsClock.tick(10)
        elif currentState == 'goalFound':
            currNode = goalNode
            pygame.display.set_caption('Goal Reached')
            
            while currNode.parent != None:
                pygame.draw.line(screen,red,currNode.point,currNode.parent.point)
                currNode = currNode.parent
            optimizePhase = True
        elif currentState == 'optimize':
            fpsClock.tick(0.5)
            pass
        elif currentState == 'buildTree':
            count = count+1
            pygame.display.set_caption('Performing RRT')
            if count < NUMNODES:
                foundNext = False
                while foundNext == False:
                    rand = get_random_clear()
                    parentNode = nodes[0]
                    for p in nodes:
                        if dist(p.point,rand) <= dist(parentNode.point,rand):
                            newPoint = step_from_to(p.point,rand)
                            if lineCollides(newPoint,p.point) == False and pointCollides(newPoint) == False:
                                parentNode = p
                                foundNext = True

                newnode = step_from_to(parentNode.point,rand)
				
                for p in nodes:
                    if dist(p.point,newnode) + p.cost <= dist(parentNode.point,newnode) + parentNode.cost:
                            if lineCollides(newPoint,p.point) == False and pointCollides(newPoint) == False:
                                parentNode = p
				
                nodes.append(Node(newnode, parentNode,parentNode.cost+dist(newnode,parentNode.point)))
                pygame.draw.line(screen,cyan,parentNode.point,newnode)

                if point_circle_collision(newnode, goalPoint.point, GOAL_RADIUS):
                    currentState = 'goalFound'

                    goalNode = nodes[len(nodes)-1]

                
            else:
                print("Ran out of nodes... :(")
                return;

        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Exiting")
            if e.type == MOUSEBUTTONDOWN:
                print('mouse down')
                if currentState == 'init':
                    if initPoseSet == False:
                        nodes = []
                        if collides(e.pos) == False:
                            print('initiale point set: '+str(e.pos))

                            initialPoint = Node(e.pos, None)
                            nodes.append(initialPoint) # Start in the center
                            initPoseSet = True
                            pygame.draw.circle(screen, red, initialPoint.point, GOAL_RADIUS)
                    elif goalPoseSet == False:
                        print('goal point set: '+str(e.pos))
                        if collides(e.pos) == False:
                            goalPoint = Node(e.pos,None)
                            goalPoseSet = True
                            pygame.draw.circle(screen, green, goalPoint.point, GOAL_RADIUS)
                            currentState = 'buildTree'
                else:
                    currentState = 'init'
                    initPoseSet = False
                    goalPoseSet = False
                    reset()

        pygame.display.update()
        fpsClock.tick(FPS)



if __name__ == '__main__':
    main()
    
