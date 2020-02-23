#!/usr/bin/python

import ap_lib.gps_utils as gps

import enum
import struct

from WrathServerModel import Server
from WrathServerModel.Collections import Graph
from WrathServerModel import wrath_to_kml as kml
from WrathServerModel import RouteReconMessages as msgs

vertices = [
    (41.39094, -73.95294),
    (41.39127, -73.95320),
    (41.39189, -73.95297),
    (41.39169, -73.95299),
    (41.39180, -73.95262),
    (41.39169, -73.95229),
    (41.39153, -73.95261),
    (41.39139, -73.95240),
    (41.39107, -73.95263)
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

class RouteReconServer(Server.Server):

    def __init__(self):
        super(RouteReconServer, self).__init__()
        self.numDrones = -1
        self.curDrones = 0
        self.dronePositions = list()
        self.roadNetwork = Graph.fill(vertices, edges, True)
        self.analyzedRoads = set() # Should this be a subgraph
        kml.generate()
        for v in vertices:
            point = (v[1], v[0])
            kml.addPoint(point)
        kml.save("route_recon")

    
    def handleMessageData(self, data):
        data = "".join(data)
        id = struct.unpack("!l", data[:4])[0]
        data = data[4:]

        # start behavior
        if (id == 1):
            print ("MODEL: Drone Starting Behavior")
            parser = msgs.StartBehaviorMessage()
            parser.unpack(data)

            if self.numDrones == -1:
                self.numDrones = parser.numDrones
                self.dronePositions = [-1 for i in range(self.numDrones)]
            self.curDrones += 1
            self.dronePositions[parser.id] = parser.location
            if self.curDrones == self.numDrones:
                parser = msgs.InitGraphMessage()
                parser.vertices = vertices
                parser.edges = edges
                parser.dronePositions = self.dronePositions
                data = struct.pack("!l", 3) + parser.pack()
                self.broadcast(data)

        # road analyzed
        elif (id == 2):
            parser = msgs.RoadAnalyzedMessage()
            parser.unpack(data)

            start, end = (parser.start, parser.end)
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
            
            kml.generate()
            for v in vertices:
                point = (v[1], v[0])
                kml.addPoint(point)
            for road in self.analyzedRoads:
                start = vertices[road[0]]
                start = (start[1], start[0])
                end = vertices[road[1]]
                end = (end[1], end[0])
                kml.addLine(start, end)
            kml.save("route_recon")

        # TODO: Handle object found message

