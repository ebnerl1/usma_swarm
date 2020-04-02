#!/usr/bin/python

import heapq
import ap_lib.gps_utils as gps

from Collections import Graph
from Collections import Vertex
from Collections import Edge
from Collections import Traversal
from Collections import LineSegment

def getDist(p1, p2):
    return gps.gps_distance(p1[0], p1[1], p2[0], p2[1])

def fill(bounds, points):
    contour = ContourLine(bounds)
    segments = contour.generateLineSegments(points)
    contour.generateGraph(segments, points)
    return contour

# Object that represents a radiation contour line.
#
# It's created after we have at least 2 known points on the
# same contour line. It connects those points, then approximates
# where the points would be on the bounding rectangle
#
# 2 main functions:
#   -- Track updates as we gain more rad info
#   -- Evaluate t values similar to a parametric vector, where
#      you can give it a t value between 0 and 1, and it will
#      tell you where that point is in world space
class ContourLine(object):

    def __init__(self, bounds):
        self.value = -1.0 # radiation value. Is this even needed?
        self.bounds = bounds # list of 4 points:

        self.startVertex = None
        self.graph = None
        
        self.totalDist = 0.0 # total distance over all line segments (m)


    def generateAllSortedSegments(self, points, sortFunc = lambda x, y: getDist(x, y)):
        numPoints = len(points)
        possibleSegments = [(i, ii) for i in range(numPoints) 
                                    for ii in range(i + 1, numPoints)]
        orderedSegments = [(sortFunc(points[x], points[y]), (x,y)) for (x,y) in possibleSegments]
        orderedSegments.sort()
        return orderedSegments


    def generateLineSegments(self, points):        
        orderedSegments = self.generateAllSortedSegments(points)
        usedIndices = set()
        segments = list()
        while (len(segments) < len(points) - 1):
            node = heapq.heappop(orderedSegments)
            if (node[1] in usedIndices or (node[1][1], node[1][0]) in usedIndices):
                continue
            
            segments.append(node)
            self.totalDist += node[0]
            usedIndices.add(node[1])
        return segments


    def generateGraph(self, segments, points):
        # construct graph with current vertices and edges
        self.graph = Graph.fill(points, [segment[1] for segment in segments])

        # find overall slope
        orderedSegments = self.generateAllSortedSegments(points, lambda x, y: -getDist(x, y))
        maxDistPoints = [points[i] for i in orderedSegments[0][1]]

        # project slope to hit bounds lines
        boundsLines = [LineSegment.LineSegment(self.bounds[i], self.bounds[(i+1) % len(self.bounds)]) for i in range(len(self.bounds)) ]
        boundsIntersections = [segment.findIntersection(maxDistPoints) for segment in boundsLines]
        boundsIntersections = [point for (line, point) in zip(boundsLines, boundsIntersections) if line.pointBetweenLine(point)]

        for intersection in boundsIntersections:
            self.graph.connectClosestVertex(Vertex.Vertex(intersection))

        startIndex = self.graph.nVertices - 2
        self.startVertex = self.graph.getVertex(startIndex)

    def findClosestPointOnEdge(self, point, edge):
        segment = LineSegment.LineSegment(edge.start.coord, edge.end.coord)
        normal = segment.getNormal()

        normalSegment = LineSegment.LineSegment(point, (point[0] + normal[0], point[1] + normal[1]))
        intersection = normalSegment.findIntersection(segment)
        if (not segment.pointBetweenLine(intersection)):
            if getDist(edge.start.coord, point) < getDist(edge.end.coord, point):
                return edge.start.coord
            else:
                return edge.end.coord
        return intersection

    def getIntersectingEdges(self, point, dir):
        segments = [(LineSegment.LineSegment(edge.start.coord, edge.end.coord), edge) for edge in self.graph.getEdges()]
        return [e for (s, e) in segments if s.isIntersectingWithLine(point, dir)]


    def getClosestEdge(self, point):
        # find all intersection points
        intersections = [(self.findClosestPointOnEdge(point, edge), edge) for edge in self.graph.getEdges()]

        # find distances between intersections and point
        distances = [(getDist(intersection, point), edge) for (intersection, edge) in intersections]

        # sort distances
        heapq.heapify(distances)

        # return edge
        return distances[0][1]


    def calculateError(self, point, dir):
        print "Calculate Error"
        intersectingEdges = self.getIntersectingEdges(point, dir)
        print len(intersectingEdges)
        print intersectingEdges[0]
        distances = [(getDist(self.findClosestPointOnEdge(point, e), point), e) for e in intersectingEdges]
        print len(distances)
        print distances[0]
        heapq.heapify(distances)
        edge = distances[0][1]

        # find nearest point
        segment = LineSegment.LineSegment(edge.start.coord, edge.end.coord)
        normal = segment.getNormal()

        normalSegment = LineSegment.LineSegment(point, (point[0] + normal[0], point[1] + normal[1]))

        intersection = normalSegment.findIntersection(segment)

        return getDist(intersection, point)

    def updateContour(self, newPoint, dir):
        print "Update Contour"
        intersectingEdges = self.getIntersectingEdges(newPoint, dir)
        print len(intersectingEdges)
        distances = [(getDist(self.findClosestPointOnEdge(newPoint, e), newPoint), e) for e in intersectingEdges]
        heapq.heapify(distances)
        edge = distances[0][1]

        # Add vertex
        newVertex = Vertex.Vertex(newPoint)
        self.graph.addVertex(newVertex)

        # Remove edge
        self.graph.removeEdge(edge)

        # Add 2 new edges
        self.graph.addEdge(Edge.Edge(edge.start, newVertex))
        self.graph.addEdge(Edge.Edge(edge.end, newVertex))


    # t in the parametric vector form
    def evaluateT(self, tValue):
        # get total length
        totalLength = 0
        for edge in self.graph.getEdges():
            totalLength += edge.distance

        tValue *= totalLength

        # find edge that contains t
        path = Traversal.traverseEulerianPath(self.graph.copy(), self.startVertex)

        pathEdges = list()
        lastVertex = path[0]
        for vertex in path[1:]:
            pathEdges.append((lastVertex, vertex))
            lastVertex = vertex

        tEdge = None
        startVertex = None
        endVertex = None
        for (start, end) in pathEdges:
            edge = self.graph.getEdge(start.id, end.id)

            if tValue - edge.distance < 0:
                tEdge = edge
                startVertex = start
                endVertex = end
                break
            tValue -= edge.distance

        ratio = tValue / tEdge.distance

        slope = (end.coord[0] - start.coord[0], end.coord[1] - start.coord[1])
        diff = (slope[0] * ratio, slope[1] * ratio)

        finalPoint = (start.coord[0] + diff[0], start.coord[1] + diff[1])

        return finalPoint


    def getTFromLocation(self, location):
        # figure out which edge it's closest to
        edge = self.getClosestEdge(location)

        # figure out how long the edges are before it
        path = Traversal.traverseEulerianPath(self.graph.copy(), self.startVertex)

        distToEdge = 0
        lastVertex = path[0]
        for v in path[1:]:
            curEdge = self.graph.getEdge(lastVertex.id, v.id)
            if curEdge == edge:
                break
            distToEdge += curEdge.distance
            lastVertex = v

        # figure out how far up the edge it is
        distFromEdge = getDist(lastVertex.coord, location)

        # figure out total length
        totalLength = 0
        for edge in self.graph.getEdges():
            totalLength += edge.distance

        # calculate t value
        return (distToEdge + distFromEdge) / totalLength
    

    def getEvenlyDistributedLocations(self, numSteps):
        # Determine total distance
        totalLength = 0
        for edge in self.graph.getEdges():
            totalLength += edge.distance

        # Determine distance between t values
        distBetween = totalLength / (numSteps - 1)

        # Traverse the path
        path = Traversal.traverseEulerianPath(self.graph.copy(), self.startVertex)
        
        pathEdges = list()
        lastVertex = path[0]
        for vertex in path[1:]:
            pathEdges.append((lastVertex, vertex, self.graph.getEdge(lastVertex.id, vertex.id)))
            lastVertex = vertex

        locations = [pathEdges[0][0].coord]
        lastPoint = pathEdges[0][0].coord
        distToNext = distBetween
        edgeDist = 0

        # Walk the path and create locations
        for (start, end, edge) in pathEdges:
            while edgeDist < edge.distance:
                # we can fit another point
                if (edge.distance - edgeDist > distToNext):
                    remainingEdgeDist = getDist(lastPoint, end.coord)
                    ratio = distToNext / remainingEdgeDist

                    # construct point between last point and end thats distToNext meters away
                    slope = (end.coord[0] - lastPoint[0], end.coord[1] - lastPoint[1])
                    newPoint = (lastPoint[0] + slope[0] * ratio, lastPoint[1] + slope[1] * ratio)
                    locations.append(newPoint)

                    # reset lastPoint and distToNext
                    lastPoint = newPoint
                    edgeDist += distToNext
                    distToNext = distBetween
                # we need to go to the next edge
                else:
                    distToNext -= edge.distance - edgeDist
                    lastPoint = end.coord
                    edgeDist = 0
                    break
        if (len(locations) < numSteps):
            locations.append(lastPoint)
        return locations
            
