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

    def __init__(self, contourLines, numSteps, laneLength, sparcityWeight, sparcityDiffusion, 
                 errorWeight, errorDiffusion, errorThreshold, proximityWeight):
        self.numSteps = numSteps
        self.laneLength = laneLength
        
        self.contourDataSelections = [ContourData(contour, numSteps) for contour in contourLines]

        # Sparcity values
        self.sparcityWeight = sparcityWeight
        self.sparcityDiffusion = sparcityDiffusion

        # Error values
        self.errorWeight = errorWeight
        self.errorDiffusion = errorDiffusion
        self.errorThreshold = errorThreshold

        # Drone proximity values
        self.proximityWeight = proximityWeight
        
        for contourData in self.contourDataSelections:
            self.addSparcitySource(contourData, contourData.contourLine.graph.vertices[0].coord)


    def generateLane(self, droneLocation):
        tValues = list()
        for contourData in self.contourDataSelections:
            self.calculateProximityValues(contourData, droneLocation)
            (weight, value) = self.chooseTValue(contourData)
            tValues.append((contourData, weight, value))        

        tValue = -1
        contourData = None
        highestWeight = -10000000
        index = -1
        for i in range(len(tValues)):
            (contour, weight, val) = tValues[i]
            if weight > highestWeight:
                index = i
                highestWeight = weight
                tValue = val
                contourData = contour


        location = contourData.contourLine.evaluateT(tValue)
        normal = contourData.contourLine.getClosestEdge(location).getNormal()
        
        edge = contourData.contourLine.getClosestEdge(location)

        ratio = (self.laneLength / 2) / getDist((location[0] + normal[0], location[1] + normal[1]), location)

        diff = (normal[0] * ratio, normal[1] * ratio)

        p1 = (location[0] + diff[0], location[1] + diff[1])
        p2 = (location[0] - diff[0], location[1] - diff[1])
        return (index, p1, location, p2)


    def chooseTValue(self, contourData):
        # Combine 3 pieces of info into one mega info
        finalWeights = [0 for i in range(self.numSteps)]
        for i in range(self.numSteps):
            finalWeights[i] = contourData.sparcityValues[i] * self.sparcityWeight \
                              + (contourData.errorValues[i] + 0.5) * self.errorWeight \
                              +  contourData.proximityValues[i] * self.proximityWeight

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
        return (largestVal, largestIndex / float(self.numSteps))


    def calculateProximityValues(self, contourData, droneLocation):
        # Get a location for each t value
        points = contourData.contourLine.getEvenlyDistributedLocations(self.numSteps)

        # Calculate distance from drone to each location
        distances = [getDist(droneLocation, point) for point in points]

        # Assign proximity values based on a function
        # Function: y = 2 / (1 + e ^ (x / 30))
        contourData.proximityValues = [2 / (1 + math.exp(dist / 30)) for dist in distances]
    

    # Sparcity source has distribution [0, 1]
    def addSparcitySource(self, contourIndex, location):
        # add the soure and recalculate values
        contourData = self.contourDataSelections[contourIndex]
        contourData.sparcitySources.append(location)
        tValues = [contourData.contourLine.getTFromLocation(source) for source in contourData.sparcitySources]
        indices = [int(t * (self.numSteps - 1)) for t in tValues]
        
        propagatedSources = Queue.Queue()
        contourData.sparcityValues = [-1 for i in range(self.numSteps)]
        for i in indices:
            propagatedSources.put((i - 1, -1))
            propagatedSources.put((i + 1, 1))
            contourData.sparcityValues[i] = 1

        while propagatedSources.qsize() > 0:
            source = propagatedSources.get()
            if source[0] > -1 and source[0] < self.numSteps:
                if contourData.sparcityValues[source[0]] == -1:
                    newValue = contourData.sparcityValues[source[0] - source[1]] * self.sparcityDiffusion
                    contourData.sparcityValues[source[0]] = newValue
                    propagatedSources.put((source[0] + source[1], source[1]))
        contourData.sparcityValues = [1 - x for x in contourData.sparcityValues]
    

    # Error source has distribution [-.5, .5]
    def addErrorSource(self, index, location, errorValue):
        contourData = self.contourDataSelections[index]

        # find location t value and index
        tValue = contourData.contourLine.getTFromLocation(location)
        i = int(tValue * self.numSteps)

        # convert errorValue (m) to [-.5, .5] distribution
        # TODO: Change this function?
        # Using function: y = -1 / (1 + e ^ (x - errorThreshold) / 5) + 0.5
        distributedErrorVal = -1 / (1 + math.exp(float(errorValue - self.errorThreshold)/5)) + 0.5
        
        # start at the index, and propagate additive source out by diffusion
        contourData.errorValues[i] += distributedErrorVal

        propagatedSources = Queue.Queue()
        propagatedSources.put((i - 1, -1, distributedErrorVal * self.errorDiffusion))
        propagatedSources.put((i + 1, +1, distributedErrorVal * self.errorDiffusion))

        while propagatedSources.qsize() > 0:
            source = propagatedSources.get()
            if source[0] > -1 and source[0] < self.numSteps:
                contourData.errorValues[source[0]] += source[2]
                propagatedSources.put((source[0] + source[1], source[1], source[2] * self.errorDiffusion))


class ContourData(object):

    def __init__(self, contourLine, numSteps):
        self.contourLine = contourLine

        # Sparcity values
        self.sparcityValues = [0 for i in range(numSteps)]
        self.sparcitySources = list()

        # Error value
        self.errorValues = [0 for i in range(numSteps)]

        #Proximity value
        self.proximityValues = [0 for i in range(numSteps)]

        for point in self.contourLine.graph.vertices[1:]:
            self.sparcitySources.append(point.coord)
