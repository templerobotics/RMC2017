#!/usr/bin/env python

import math
import numpy as np
from Queue import PriorityQueue
from collections import deque

#Custom exception
class MapException(Exception):
    pass

#A map laid out as a 2 dimensional cartesian plane
class CoordinateMap:

    #Initializes the map
    def __init__(self, xmax, ymax, xmin = 0, ymin = 0):
        if xmin > xmax:
            raise ValueError('xmin cannot be larger than xmax')
        elif ymin > ymax:
            raise ValueError('ymin cannot be larger than ymax')

        self.xrange = (xmin, xmax)
        self.yrange = (ymin, ymax)
        self.xdim = xmax - xmin + 1
        self.ydim = ymax - ymin + 1

        self.map = np.zeros((self.xdim, self.ydim), dtype=np.int8)

    #Standard distance formula
    def _distance(self, point1, point2):
        return math.sqrt((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2)

    #Returns True if the specified point lies in the defined map range
    def _inmap(self, point):
        return ((point[0] <= self.xrange[1]) and (point[0] >= self.xrange[0])) and ((point[1] <= self.yrange[1]) and (point[1] >= self.yrange[0]))

    #Shifts a point from a contextual coordinate to the appropriate index
    def _shiftPoint(self, point):
        return (point[0] - self.xrange[0], point[1] - self.yrange[1])

    #Boolean indicating if the point is open
    def _isOpen(self, point):
        try:
            if self._inmap(point):
                point = self._shiftPoint(point)
                if self.map[point[0], point[1]] == 0:
                    return True
                else:
                    return False
            return False
        except IndexError:
            return False

    #Blocks out a square region of the map. Radius = 1/2 side length
    def addSquareObstacle(self, center, radius = 0, rejectOutOfMap = True):
        #Verifies the specified point exists
        #The rejectOutOfMap flag allows points whose radius affects the map to be used
        if not self._inmap(center) and rejectOutOfMap:
            raise ValueError('Point {} is outside of map'.format(str(point)))

        #Shifts the point from a contextual coordinate to the appropriate index
        point = self._shiftPoint(center)

        #Blocks out any points that exist in the designated square
        for x in range(point[0] - radius, point[0] + radius + 1):
            for y in range(point[1] - radius, point[1] + radius + 1):
                try:
                    self.map[x,y] = 1
                except IndexError: #If the point isn't in the map, ignore it
                    continue

    #Blocks out a circular region of the map
    def addCircleObstacle(self, center, radius = 0, rejectOutOfMap = True):
        #Verifies the specified point exists
        #The rejectOutOfMap flag allows points whose radius affects the map to be used
        if not self._inmap(center) and rejectOutOfMap:
            raise ValueError('Point {} is outside of map'.format(str(point)))

        #Shifts the point from a contextual coordinate to the appropriate index
        point = self._shiftPoint(center)

        #Blocks out any points that exist in the designated cirlce
        for x in range(point[0] - radius, point[0] + radius + 1):
            for y in range(point[1] - radius, point[1] + radius + 1):
                try:
                    if self._distance(point, self.map[x,y]) <= radius: #Determines if point is in circle
                        self.map[x,y] = 1
                except IndexError: #If the point isn't in the map, ignore it
                    continue

    def pathfind(self, start, target):

        searchQueue = PriorityQueue()
        searched = []
        pathPoints = deque()

        if not (self._inmap(start) or self._inmap(target)):
            raise ValueError('Start and target points must be in the map')

        found, _ = self._pathfind(start, target, searchQueue, searched, pathPoints)

        if not found:
            raise MapException('No path found')

        pathPoints.reverse()

        path = []
        for point in pathPoints:
            path.append(point)

        return path

    def _pathfind(self, current, target, searchQueue, searched, pathPoints):

        #If we have arrived at the target
        if current == target:
            pathPoints.append(current)
            return True, current

        #Add the current point to the searched list
        searched.append(current)

        #Add the neighbors to the searching queue with their distance as priority
        for neighbor in self._getNeighbors(current):
            if neighbor not in searched:
                distance = self._distance(neighbor, target)
                searchQueue.put((distance, neighbor), False)

        #There are no connected points left to check, return False
        if searchQueue.empty():
            return False, None

        #Pathfind from the next closest point
        found, nextPoint = self._pathfind(searchQueue.get(False)[1], target, searchQueue, searched, pathPoints)

        if found:
            if self._isAdjacent(current, nextPoint):
                pathPoints.append(current)
                return True, current
            return True, nextPoint
        else:
            return False, None

    #Returns list of points adjacent to initial point
    def _getNeighbors(self, point):
        #The adjacent points
        up = (point[0], point[1] + 1)
        down = (point[0], point[1] - 1)
        right = (point[0] + 1, point[1])
        left = (point[0] - 1, point[1])

        #Empty list
        neighbors = []

        #Add point to list if it is in the map
        if self._inmap(up) and self._isOpen(up):
            neighbors.append(up)
        if self._inmap(down)and self._isOpen(down):
            neighbors.append(down)
        if self._inmap(right) and self._isOpen(right):
            neighbors.append(right)
        if self._inmap(left) and self._isOpen(left):
            neighbors.append(left)

        return neighbors

    #Returns True if the two points are face adjacent. Note: returns false on diagonals
    def _isAdjacent(self, point1, point2):
        deltaX = abs(point1[0] - point2[0])
        deltaY = abs(point1[1] - point2[1])
        return (deltaX == 1 and deltaY == 0) or (deltaX == 0 and deltaY == 1)
