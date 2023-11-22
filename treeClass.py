import numpy as np
import math
import matplotlib.pyplot as plt
import random

class treeNode():
    def __init__(self, locationX, locationY):
        self.locationX = locationX
        self.locationY = locationY
        self.children = []
        self.parent = None

class tree():
    def __init__(self, start, goal, numIterations, grid, stepSize):
        self.randomTree = treeNode(start[0], start[1])      #root position
        self.goal = treeNode(goal[0], goal[1])              #goal position
        self.nearestNode = None                             #nearest node
        self.iterations = numIterations           #number of iterations
        self.grid = grid                                    #the map
        self.rho = stepSize                                 #lenght of each branch
        self.path_distance = 0                              #total path distance
        self.nearestDist = 10000                            #distance to nearest node
        self.numWaypoints = 0                               #number of waypoints
        self.Waypoints = []                                 #the waypoints

    #add the point to the nearest node and add goal when reached
    def addChild(self, locationX, locationY):
        if(locationX == self.goal.locationX):
            self.nearestNode.children.append(self.goal)
            self.goal.parent = self.nearestNode
        else:
            tempNode = treeNode(locationX,locationY)
            self.nearestNode.children.append(tempNode)
            tempNode.parent = self.nearestNode
    
    #sample a random point in the grid
    def sampleAPoint(self):
        x = random.randint(1, self.grid.shape[1])
        y = random.randint(1, self.grid.shape[0])
        point = np.array([x,y])
        return point

    #steer a distance stepsize from star to end location, return a candidate to be add into the tree
    def steerToPoint(self, locationStart, locationEnd):
        offset = self.rho*self.unitVector(locationStart, locationEnd)
        point = np.array([locationStart.locationX + offset[0], locationStart.locationY + offset[1]])
        if point[0] >= self.grid.shape[1]:
            point[0] = self.grid.shape[1]-1
        if point[0] <= 0:
            point[0] = 0+1
        if point[1] >= self.grid.shape[0]:
            point[1] = self.grid.shape[0]-1
        if point[1] <= 0:
            point[1] = 0+1
        return point

    #check if there is an obstacles between the start node and the candidate to be add into the tree
    def isInObstacle(self, locationStart, locationEnd):
        u_hat = self.unitVector(locationStart, locationEnd)
        testPoint = np.array([0.0, 0.0])
        for i in range(self.rho+1):
            testPoint[0] = min(locationStart.locationX + i*u_hat[0], self.grid.shape[1]-1)
            testPoint[1] = min(locationStart.locationY + i*u_hat[1], self.grid.shape[0]-1)
            if self.grid[math.floor(testPoint[1]), math.floor(testPoint[0])] != 0:
                return True
        return False

    def unitVector(self, locationStart, locationEnd):
        v = np.array([locationEnd[0] - locationStart.locationX, locationEnd[1] - locationStart.locationY])
        u_hat = v/np.linalg.norm(v)
        return u_hat

    #find the nearest node to a given unconnected point
    def findNearest(self, root, point):
        if not root:
            return
        dist = self.distance(root, point)
        if dist <= self.nearestDist:
            self.nearestNode = root
            self.nearestDist = dist
        for child in root.children:
            self.findNearest(child, point)
        pass
    
    #find euclidean distance between a node and an XY point
    def distance(self, node1, point):
        dist = np.sqrt((node1.locationX - point[0])**2 + (node1.locationY - point[1])**2)
        return dist

    #check if the goal has been reched within step size
    def goalFound(self, point):
        if self.distance(self.goal, point) <= self.rho:
            return True
        pass
    
    #reset nearestNode and nearest Distance
    def resetNearestValues(self):
        self.nearestNode = None
        self.nearestDist = 10000

    def retraceRRTPath(self, goal):
        if goal.locationX == self.randomTree.locationX:
            return
        self.numWaypoints += 1
        currentPoint = np.array([goal.locationX, goal.locationY])
        self.Waypoints.insert(0, currentPoint)
        self.path_distance += self.rho
        self.retraceRRTPath(goal.parent)
#end of class methods ------------------------------------------------------