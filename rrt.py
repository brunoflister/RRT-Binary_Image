import numpy as np
import matplotlib.pyplot as plt
import treeClass as trC
import readimage as img

imagem = 'maps/cspace.png'
img.read_image(imagem)

grid = np.load('cspace.npy')

start = np.array([60.0, 100.0])
goal = np.array([700.0, 500.0])
numIterations = 1000
stepSize = 50
goalRegion = plt.Circle((goal[0], goal[1]), stepSize, color='b', fill = False)

fig = plt.figure("RRT Algorithm")
plt.imshow(grid, cmap = 'binary')
plt.plot(start[0], start[1], 'ro')
plt.plot(goal[0], goal[1], 'bo')
ax = fig.gca()
ax.add_patch(goalRegion)
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')

#Algoritmo RRT
rrt = trC.tree(start, goal, numIterations, grid, stepSize)
for i in range(rrt.iterations):
    rrt.resetNearestValues()
    print("iteration: ", i)
    #begin
    point = rrt.sampleAPoint()
    rrt.findNearest(rrt.randomTree, point)
    new = rrt.steerToPoint(rrt.nearestNode, point)
    bool = rrt.isInObstacle(rrt.nearestNode, new)
    if(bool == False):
        rrt.addChild(new[0], new[1])
        plt.pause(0.10)
        plt.plot([rrt.nearestNode.locationX, new[0]], [rrt.nearestNode.locationY, new[1]], 'go', linestyle = "--")
        if(rrt.goalFound(new)):
            rrt.addChild(goal[0], goal[1])
            print("Goal Found!")
            break

rrt.retraceRRTPath(rrt.goal)
rrt.Waypoints.insert(0,start)
print("Number of Waypoints:", rrt.numWaypoints)
print("Path Distance (m): ", rrt.path_distance)
print("Waypoints: ", rrt.Waypoints)

for i in range(len(rrt.Waypoints)-1):
    plt.plot([rrt.Waypoints[i][0], rrt.Waypoints[i+1][0]], [rrt.Waypoints[i][1], rrt.Waypoints[i+1][1]], 'ro', linestyle = '--')
    plt.pause(0.10)

plt.show()