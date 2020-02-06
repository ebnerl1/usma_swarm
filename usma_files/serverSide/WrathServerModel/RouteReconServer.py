#!/usr/bin/python

import ap_lib.gps_utils as gps

import enum

from WrathServerModel import Server
from WrathServerModel.Collections import Graph

vertices = [
    (41.39077, -73.95298),
    (41.39127, -73.95320),
    (41.39189, -73.95297),
    (41.39169, -73.95299),
    (41.39180, -73.95262),
    (41.39169, -73.95229),
    (41.39153, -73.95261),
    (41.39133, -73.95223),
    (41.39103, -73.95252)
]

edges = [
    (0, 1),
    (0, 8),
    (1, 2),
    (1, 3),
    (3, 4),
    (4, 5),
    (5, 6),
    (5, 7),
    (6, 7),
    (7, 8)
]

# Client Messages:
# 0: Heartbeat
# 1: Start Behavior: numDrones, id, location
# 2: Road Analyzed: (start, end)
# 3: Object Found (location, picture, size?)
#
# Server Message:
# 0: Heartbeat
# 1: Init Graph
class MessageType(enum.IntEnum):
    Heartbeat = 0
    StartBehavior = 1
    RoadAnalyzed = 2
    ObjectFound = 3

class RouteReconServer(Server.Server):

    def __init__(self):
        super(RouteReconServer, self).__init__()
        self.numDrones = -1
        self.curDrones = 0
        self.dronePositions = list()
        self.roadNetwork = Graph.fill(vertices, edges, True)
        self.analyzedRoads = set() # Should this be a subgraph

    
    def handleMessageData(self, data):
        messageType = data[0]

        if messageType == MessageType.Heartbeat:
            pass

        elif messageType == MessageType.StartBehavior:
            if self.numDrones == -1:
                self.numDrones = data[1]
                self.dronePositions = [-1 for i in range(self.numDrones)]
            self.curDrones += 1
            self.dronePositions[data[2]] = data[3]
            if self.curDrones == self.numDrones:
                self.broadcast([1, vertices, edges, self.dronePositions])

        elif  messageType == MessageType.RoadAnalyzed:
            start, end = data[1]
            startIndex = -1
            endIndex = -1
            i = 0
            for v in vertices:
                startDist = gps.gps_distance(start[0], start[1], v[0], v[1])
                if startDist < 5:
                    startIndex = i
                endDist = gps.gps_distance(end[0], end[1], v[0], v[1])
                if endDist < 5:
                    endIndex = i
                i += 1
            print "MODEL: Analyzed Road:", startIndex, endIndex
            self.analyzedRoads.add((startIndex, endIndex))


        elif messageType == MessageType.ObjectFound:
            # TODO: Handle object found message
            pass
        
        return [0]

