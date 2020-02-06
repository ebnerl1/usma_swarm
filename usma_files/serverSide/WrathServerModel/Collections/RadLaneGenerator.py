#!/usr/bin/python

import ap_lib.gps_utils as gps
import math

import Queue
from Collections import ContourLine

def getDist(p1, p2):
    return gps.gps_distance(p1[0], p1[1], p2[0], p2[1])

# Generates based on 3 pieces of info:
#   -- Error: error between our model and real data
#   -- Sparcity: areas that haven't been looked at yet
#   -- Location: how far away the drone is
class RadLaneGenerator(object):

    # TODO: Make this work for multiple contour lines!
    def __init__(self, contourLine, numSteps, laneLength, sparcityWeight, sparcityDiffusion, 
                 errorWeight, errorDiffusion, errorThreshold, proximityWeight):
        self.contourLine = contourLine
        self.numSteps = numSteps
        self.laneLength = laneLength

        # Sparcity values
        self.sparcityWeight = sparcityWeight
        self.sparcityDiffusion = sparcityDiffusion
        self.sparcitySources = list()
        self.sparcityValues = [0 for i in range(numSteps)]

        # Error values
        self.errorWeight = errorWeight
        self.errorDiffusion = errorDiffusion
        self.errorThreshold = errorThreshold
        self.errorValues = [0 for i in range(numSteps)]

        # Drone proximity values
        self.proximityWeight = proximityWeight
        self.proximityValues = [0 for i in range(numSteps)]

        for point in self.contourLine.graph.vertices[1:]:
            self.sparcitySources.append(point.coord)
        
        self.addSparcitySource(self.contourLine.graph.vertices[0].coord)


    def generateLane(self, droneLocation):
        self.calculateProximityValues(droneLocation)
        tValue = self.chooseTValue()

        location = self.contourLine.evaluateT(tValue)
        normal = self.contourLine.getClosestEdge(location).getNormal()
        
        edge = self.contourLine.getClosestEdge(location)

        ratio = (self.laneLength / 2) / getDist((location[0] + normal[0], location[1] + normal[1]), location)

        diff = (normal[0] * ratio, normal[1] * ratio)

        p1 = (location[0] + diff[0], location[1] + diff[1])
        p2 = (location[0] - diff[0], location[1] - diff[1])
        return (p1, location, p2)


    def chooseTValue(self):
        # Combine 3 pieces of info into one mega info
        finalWeights = [0 for i in range(self.numSteps)]
        for i in range(self.numSteps):
            finalWeights[i] = self.sparcityValues[i] * self.sparcityWeight \
                              + (self.errorValues[i] + 0.5) * self.errorWeight \
                              +  self.proximityValues[i] * self.proximityWeight

        #print [round(f, 2) for f in self.sparcityValues]
        #print [round(f + .5, 2) for f in self.errorValues]
        #print [round(f, 2) for f in self.proximityValues]
        #print [round(f, 2) for f in finalWeights], "\n"
        
        # Go through and pick the biggest
        largestVal, largestIndex = (-10000, -1)
        for i in range(self.numSteps):
            if (finalWeights[i] > largestVal):
                largestVal = finalWeights[i]
                largestIndex = i
        print "Largest Index: ", largestIndex, "\n"
        return largestIndex / float(self.numSteps)


    def calculateProximityValues(self, droneLocation):
        # Get a location for each t value
        points = self.contourLine.getEvenlyDistributedLocations(self.numSteps)

        # Calculate distance from drone to each location
        distances = [getDist(droneLocation, point) for point in points]

        # Assign proximity values based on a function
        # Function: y = 2 / (1 + e ^ (x / 30))
        self.proximityValues = [2 / (1 + math.exp(dist / 30)) for dist in distances]
    

    # Sparcity source has distribution [0, 1]
    def addSparcitySource(self, location):
        # add the soure and recalculate values
        self.sparcitySources.append(location)
        tValues = [self.contourLine.getTFromLocation(source) for source in self.sparcitySources]
        indices = [int(t * (self.numSteps - 1)) for t in tValues]
        
        propagatedSources = Queue.Queue()
        self.sparcityValues = [-1 for i in range(self.numSteps)]
        for i in indices:
            propagatedSources.put((i - 1, -1))
            propagatedSources.put((i + 1, 1))
            self.sparcityValues[i] = 1

        while propagatedSources.qsize() > 0:
            source = propagatedSources.get()
            if source[0] > -1 and source[0] < self.numSteps:
                if self.sparcityValues[source[0]] == -1:
                    newValue = self.sparcityValues[source[0] - source[1]] * self.sparcityDiffusion
                    self.sparcityValues[source[0]] = newValue
                    propagatedSources.put((source[0] + source[1], source[1]))
        self.sparcityValues = [1 - x for x in self.sparcityValues]
    

    # Error source has distribution [-.5, .5]
    def addErrorSource(self, location, errorValue):
        # find location t value and index
        tValue = self.contourLine.getTFromLocation(location)
        i = int(tValue * self.numSteps)

        # convert errorValue (m) to [-.5, .5] distribution
        # TODO: Change this function?
        # Using function: y = -1 / (1 + e ^ (x - errorThreshold) / 5) + 0.5
        distributedErrorVal = -1 / (1 + math.exp(float(errorValue - self.errorThreshold)/5)) + 0.5
        
        # start at the index, and propagate additive source out by diffusion
        self.errorValues[i] += distributedErrorVal

        propagatedSources = Queue.Queue()
        propagatedSources.put((i - 1, -1, distributedErrorVal * self.errorDiffusion))
        propagatedSources.put((i + 1, +1, distributedErrorVal * self.errorDiffusion))

        while propagatedSources.qsize() > 0:
            source = propagatedSources.get()
            if source[0] > -1 and source[0] < self.numSteps:
                self.errorValues[source[0]] += source[2]
                propagatedSources.put((source[0] + source[1], source[1], source[2] * self.errorDiffusion))
