#!/usr/bin/python

import ap_lib.gps_utils as gps

import enum

import logging
import datetime
import sys

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

        self.registerMessageCallback(msgs.StartBehaviorMessage.id, 
                                     self.onReceiveStartBehavior)
        self.registerMessageCallback(msgs.RoadAnalyzedMessage.id, 
                                     self.onReceiveRoadAnalyzed)
        self.registerMessageCallback(msgs.DetectedObjectMessage.id, 
                                     self.onDetectObject)
        self.registerMessageCallback(msgs.LogMessage.id, 
                                     self.onReceiveLog)

        kml.generate()
        for v in vertices:
            point = (v[1], v[0])
            kml.addPoint(point)
        kml.save("route_recon")

        name = datetime.datetime.now().strftime("%y:%m:%d:%H:%M")
        logging.basicConfig(filename="logs/routeRecon - " + name, level=logging.INFO, 
            format="%(levelname)s:%(message)s")
        logging.getLogger().addHandler(logging.StreamHandler(sys.stdout))
        logging.info("Route Recon Starting")


    def onReceiveStartBehavior(self, message):
        logging.info("MODEL: Drone Starting Behavior")
        if self.numDrones == -1:
            self.numDrones = message.numDrones
            self.dronePositions = [-1 for i in range(self.numDrones)]
        self.curDrones += 1
        self.dronePositions[message.id] = message.location
        if self.curDrones == self.numDrones:
            message = msgs.InitGraphMessage()
            message.vertices = vertices
            message.edges = edges
            message.dronePositions = self.dronePositions
            self.broadcast(message)
    

    def onReceiveRoadAnalyzed(self, message):
        start, end = (message.start, message.end)
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
        logging.info("MODEL: Analyzed Road: " + str(startIndex) + " " + str(endIndex))
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

    def onDetectObject(self, msg):
        logging.info("DETECTED OBJ: " + str(msg.name) + " " + str(msg.probability))

    
    def onReceiveLog(self, msg):
        logging.info(msg.msg)

