obs =[]

def init_circular_obstacles(configNum=0):
    global obs
    obs.append(((0,0),50))
    '''obs.append(((150,100),18))
    obs.append(((150,130),18))
    obs.append(((150,160),18))
    obs.append(((150,190),18))
    obs.append(((150,220),18))
    obs.append(((150,250),18))
    obs.append(((180,250),20))
    obs.append(((210,250),20))
    obs.append(((240,250),20))
    obs.append(((270,250),20))
    obs.append(((300,250),20))
    obs.append(((330,250),20))
    for circ in obs:
        pygame.draw.circle(screen, black, circ[0], circ[1])'''

def lineCollides(p1, p2):
    global obs
    list = []
    a = p2[1] - p1[1]
    b = -(p2[0] - p1[0])
    c = (-1*b * p2[1]) - (a * p2[0])
    for circle in obs:
        cen, rad = circle
        val = (a * cen[0] + b * cen[1] + c*1.0) / (a * a + b * b);
        #val = (a * cen[0] + b * cen[1] + c*1.0) / (a * a + b * b);
        h = (val * a) + cen[0]
        k = (val * b) + cen[1]
        #pygame.draw.circle(screen, (255,255,0), (round(h),round(k)), 1)
        if dist(cen,(h,k)) < rad:
            return True
        else:
            t1 = (h - p1[0]) #/ (p2[0] - p1[0])
            t2 = (k - p1[1]) #/ (p2[1] - p1[1])
            if t1 * (p2[1] - p1[1]) != t2 * (p2[0] - p1[0]) and (t1 >= 0 and t1 <= (p2[0] - p1[0])):
            #if t1 != t2 and t1 >= 0 and t1 <= 1:
                return True
        #list.append((h,k))
    for i in list:
        pygame.draw.circle(screen, (255,255,0), (round(i[0]),round(i[1])), 1)
    return False

init_circular_obstacles()
while(True):
    x1 = input()
    y1 = input()
    x2 = input()
    y2 = input()
    print(str(lineCollides((x1,y1),(x2,y2))))