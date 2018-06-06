import numpy as np
from numpy import pi as pi
from numpy import sin as sin
from numpy import cos as cos
from matplotlib import pyplot as plt
import matplotlib.patches as patches

inf = 100000000

def node_to_xy(node):
    if node >= 0 and node < N:
        x = int(node%x_size)
        y = int(node/x_size)
        return x, y

def xy_to_node(x,y):
    node = int(y*x_size + x)
    return node

def check(point, direction):
    node = xy_to_node(point[0],point[1])
    if node in blocked:
        return 0
##    if direction == 0:
##        if point[direction] >= x_size:
##            return 0
##        if point[direction] < 0:
##            return 0
##    if direction == 1:
##        if point[direction] >= y_size:
##            return 0
##        if point[direction] < 0:
##            return 0
    return 1
    
def make_rect_obstacle(x_start, x_end, y_start, y_end):
    blocked = np.array([],dtype=int)
    x = x_start
    y = y_start

    while x < x_end:
        node = xy_to_node(x, y)
        blocked = np.append(blocked, node)
        x += 1
    while y < y_end:
        node = xy_to_node(x, y)
        blocked = np.append(blocked, node)
        y += 1
    while x > x_start:
        node = xy_to_node(x, y)
        blocked = np.append(blocked, node)
        x -= 1
    while y > y_start:
        node = xy_to_node(x, y)
        blocked = np.append(blocked, node)
        y -= 1
    
    return blocked
    

def find_neighbors(current):
    x,y = node_to_xy(current)
    nbrs = np.array([],dtype=int)
    
    if x < x_size-1:  #make sure we cant walk off the map
        east = [x+1, y]
        eastn = xy_to_node(east[0],east[1])
        if not eastn in blocked:    #check if the neighbor would be in an obstacle
            nbrs = np.append(nbrs,eastn)
        if y <y_size-1:
            NE = [x+1, y+1]
            NEnode = xy_to_node(NE[0],NE[1])
            if not NEnode in blocked:
                nbrs = np.append(nbrs,NEnode) 
        if y > 0:
            SE = [x+1, y-1]
            SEnode = xy_to_node(SE[0],SE[1])
            if not SEnode in blocked:
                nbrs = np.append(nbrs,SEnode)

    if y < y_size-1:
        north = [x, y+1]
        northn = xy_to_node(north[0],north[1])
        if not northn in blocked:
            nbrs = np.append(nbrs,northn)

    if x >0:
        west = [x-1, y]
        westn = xy_to_node(west[0],west[1])
        if not westn in blocked:
            nbrs = np.append(nbrs,westn)
        if y <y_size-1:
            NW = [x-1, y+1]
            NWnode = xy_to_node(NW[0],NW[1])
            if not NWnode in blocked:
                nbrs = np.append(nbrs,NWnode) 
        if y > 0:
            SW = [x-1, y-1]
            SWnode = xy_to_node(SW[0],SW[1])
            if not SWnode in blocked:
                nbrs = np.append(nbrs,SWnode)  

    if y > 0:
        south = [x, y-1]
        southn = xy_to_node(south[0],south[1])
        if not southn in blocked:     
            nbrs = np.append(nbrs,southn)

    
    
    return nbrs
    

def h_cost(node1, node2):
    x, y = node_to_xy(node1)
    x2, y2 = node_to_xy(node2)
    dist = np.sqrt((x2-x)**2 + (y2-y)**2)  #euclidean distance
    #dist = abs(x_f[0] - x) + abs(x_f[1] - y)   #manhattan distance
    return dist

x = np.array([0,0, pi/2])   #state x positio, y position, theta

x_f = [40,48,0]   #desired final state

x_size=50
y_size=50
x_axis = np.linspace(0,x_size-1,x_size)
y_axis = np.linspace(0,y_size-1,y_size)
[x_grid, y_grid] = np.meshgrid(x_axis,y_axis)

N = x_size*y_size  #number of nodes

startNode = xy_to_node(x[0],x[1])
goal = xy_to_node(x_f[0],x_f[1])

blocked = np.array([],dtype=int)  #an array recording all of the nodes that are obstacles.

b1 =[10,20,0,30]
b2 = [25, 30, 20,49]
blocked = make_rect_obstacle(b1[0], b1[1], b1[2], b1[3])
blocked = np.append(blocked, make_rect_obstacle(b2[0], b2[1], b2[2], b2[3]))

closed = np.array([],dtype=int)
openSet = np.array([startNode],dtype=int)

#cameFrom = np.zeros(N,dtype=int)
cameFrom = np.array([None]*N)

gScore = np.ones(N)+1000
gScore[startNode] = 0
fScore = np.ones(N)+1000
fScore[startNode] = h_cost(startNode, goal)
count = 0
        

def best_node():
    node = openSet[0]
    nodeScore = fScore[node]
    bestScore = nodeScore
    bestNode = node
    for i in range(1,np.size(openSet)):
        node = openSet[i]
        nodeScore = fScore[node]
        if nodeScore < bestScore:
            bestScore = nodeScore
            bestNode = node
    return bestNode

while openSet.size > 0:

    #count += 1
    #if count >10000:
    #    print "Tiemout"
    #    break
    current = best_node()
    if current == goal:
        break
    
    openSet = np.delete(openSet,np.where(openSet == current)) #Remove current from openSet
    closed = np.append(closed, current) #add currend to closed
    nbrs = find_neighbors(int(current)) #get a list of all of current's nbrs
    
    for i in range(np.size(nbrs)):
        if nbrs[i] in closed:
            continue
        
        tentative_gScore = gScore[current] + h_cost(current, nbrs[i]) #distance between neighbors

        if not nbrs[i] in openSet:
            openSet = np.append(openSet, nbrs[i])
            
        elif tentative_gScore >= gScore[nbrs[i]]:
            continue
        
        cameFrom[nbrs[i]] = current
        gScore[nbrs[i]] = tentative_gScore
        fScore[nbrs[i]] = gScore[nbrs[i]] + h_cost(nbrs[i], goal)
    
path = np.array([goal])
i = goal

while i != startNode and count <100:
    path = np.append(path,cameFrom[i])
    i = path[-1]

path = path[::-1]
coords = np.zeros((np.size(path),2))
for i in range(np.size(path)):
    coords[i,0], coords[i,1] = node_to_xy(path[i])

arrow_length = 1
head_x =  arrow_length*cos(x[2])
head_y =  arrow_length*sin(x[2])

arrows = [x[0], x[1], head_x, head_y]    
    
arrows = np.array([[coords[0,0], coords[0,1], coords[1,0]-coords[0,0], coords[1,1]-coords[0,1]]])
for i in range(1, np.size(path)-1):
    arrows = np.append(arrows,np.array([[coords[i,0], coords[i,1], coords[i+1,0]-coords[i,0], coords[i+1,1]-coords[i,1]]]),axis=0)

soa =np.array( arrows ) 
X,Y,U,V = zip(*soa)
plt.figure()
ax = plt.gca()
ax.quiver(X,Y,U,V,angles='xy',scale_units='xy',scale=1)
ax.set_xlim([-1,x_size+1])
ax.set_ylim([-1,y_size+1])
#plt.plot(x_grid, y_grid, marker='.', color='k', linestyle='none')
plt.grid()
ax.add_patch(patches.Rectangle((b1[0], b1[2]),b1[1]-b1[0],b1[3]-b1[2]))
ax.add_patch(patches.Rectangle((b2[0], b2[2]),b2[1]-b2[0],b2[3]-b2[2]))
plt.draw()
plt.show()
