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

# vertices = [
#     (41.39094, -73.95294),
#     (41.39127, -73.95320),
#     (41.39189, -73.95297),
#     (41.39169, -73.95299),
#     (41.39180, -73.95262),
#     (41.39169, -73.95229),
#     (41.39153, -73.95261),
#     (41.39139, -73.95240),
#     (41.39107, -73.95263)
# ]

vertices = [
    (40.78974, -73.95738), # P0 - 0
    (40.79173, -73.95559), # P104 - 1
    (40.79264, -73.95500), # P110 - 2
    (40.79435, -73.95482), # P121 - 3
    (40.79493, -73.95426), # P125 - 4
    (40.79526, -73.95353), # P126 - 5
    (40.79553, -73.95334), # P127 - 6
    (40.79595, -73.95365), # P128 - 7
    (40.79591, -73.95439), # P129 - 8
    (40.79552, -73.95499), # P130 - 9
    (40.79565, -73.95558), # P131 - 10
    (40.79623, -73.95566), # P132 - 11
    (40.79769, -73.95437), # P134 - 12
    (40.79842, -73.95443), # P54 - 13
    (40.79917, -73.95568), # P55 - 14
    (40.79958, -73.95787), # P57 - 15
    (40.79876, -73.95856), # P59 - 16
    (40.79837, -73.95820), # P60 - 17
    (40.79780, -73.95720), # P61 - 18
    (40.79612, -73.95697), # P64 - 19
    (40.79522, -73.95881), # P66 - 20
    (40.79441, -73.95934), # P67 - 21
    (40.79406, -73.95995), # P68 - 22
    (40.79369, -73.96108), # P69 - 23
    (40.79178, -73.96246), # P73 - 24
    (40.79161, -73.96147), # P75 - 25
    (40.79123, -73.96046), # P81 - 26
    (40.79051, -73.95916), # P83 - 27
    (40.79022, -73.95755), # P85 - 28
    (40.79447, -73.95576), # P122 - 29
    (40.79364, -73.95810) # P124 - 30
]

# edges = [
#     (0, 1),
#     (0, 8),
#     (1, 2),
#     (1, 3),
#     (3, 4),
#     (4, 5),
#     (5, 6),
#     (5, 7),
#     (6, 7),
#     (7, 8)
# ]

edges = [
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 4),
    (4, 5),
    (5, 6),
    (6, 7),
    (7, 8),
    (8, 9),
    (9, 10),
    (10, 11),
    (11, 12),
    (12, 13),
    (13, 14),
    (14, 15),
    (15, 16),
    (16, 17),
    (17, 18),
    (18, 19),
    (19, 20),
    (20, 21),
    (21, 22),
    (22, 23),
    (23, 24),
    (24, 25),
    (25, 26),
    (26, 27),
    (27, 28),
    (28, 0),
    (3, 29),
    (29, 30),
    (21, 30)
]

class RouteReconServer(Server.Server):

    def __init__(self):
        super(RouteReconServer, self).__init__()
        self.numDrones = -1
        self.curDrones = 0
        self.dronePositions = list()
        self.roadNetwork = Graph.fill(vertices, edges, True)
        self.unanalyzedRoads = self.roadNetwork.copy()
        self.analyzedRoads = set() # make this a subgraph probably
        self.obstructedRoads = set() # make this a subgraph probably
        self.foundVehicles = list()


        self.registerMessageCallback(msgs.StartBehaviorMessage.id, 
                                     self.onReceiveStartBehavior)
        self.registerMessageCallback(msgs.RoadAnalyzedMessage.id, 
                                     self.onReceiveRoadAnalyzed)
        self.registerMessageCallback(msgs.DetectedObjectMessage.id,
                                     self.onDetectObject)
        self.registerMessageCallback(msgs.LogMessage.id, 
                                     self.onReceiveLog)

        kml.generate()
        kml.addGraph(self.unanalyzedRoads, 2)
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
        
        if (startIndex, endIndex) not in self.obstructedRoads and (endIndex, startIndex not in self.obstructedRoads):
            self.analyzedRoads.add((startIndex, endIndex))

        if (self.unanalyzedRoads.hasEdge(self.unanalyzedRoads.getVertex(startIndex), self.unanalyzedRoads.getVertex(endIndex))):
            self.unanalyzedRoads.removeEdge(self.unanalyzedRoads.getEdge(startIndex, endIndex))
    
        toRemove = set()
        for road in self.analyzedRoads:
            if (road[0], road[1]) in self.obstructedRoads or (road[1], road[0]) in self.obstructedRoads:
                toRemove.add(road)
        for road in toRemove:
            self.analyzedRoads.remove(road)

        kml.generate()
        kml.addGraph(Graph.fill(vertices, list(self.analyzedRoads), True), 1, "Analyzed Roads")
        kml.addGraph(Graph.fill(vertices, list(self.obstructedRoads), True), 0, "Obstructed Roads")
        kml.addGraph(self.unanalyzedRoads, 2, "Unanalyzed Roads")
        for i in range(len(self.foundVehicles)):
            l = self.foundVehicles[i]
            kml.addPoint((l[1], l[0]), "Vehicle: " + str(i + 1))
        kml.save("route_recon")


    def onDetectObject(self, msg):
        print "Found Object!"
        l = msg.location
        self.foundVehicles.append(l)
        for edge in self.roadNetwork.getEdges():
            diff1 = gps.gps_distance(edge.start.coord[0], edge.start.coord[1], l[0], l[1])
            diff2 = gps.gps_distance(edge.end.coord[0], edge.end.coord[1], l[0], l[1])
            if diff1 + diff2 - edge.distance < 5:
                self.obstructedRoads.add((edge.start.id, edge.end.id))
                break


    
    def onReceiveLog(self, msg):
        logging.info(msg.msg)

