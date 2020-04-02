#!/usr/bin/python

import enum
import logging
import datetime
import sys
import time

from WrathServerModel.Collections import ContourLine 
from WrathServerModel import Server
from WrathServerModel import wrath_to_kml as kml
from WrathServerModel import RadDetectionMessages as msgs
from WrathServerModel import RouteReconMessages as rrmsgs
import struct

altitudeTime = time.time()

class RadDetectionServer(Server.Server):

    def __init__(self, simulationData = None):
        super(RadDetectionServer, self).__init__()

        # This is only for testing
        self.lanes = list()

        self.state = 1
        self.bounds = [
            (41.39105, -73.95342),
            (41.39197, -73.95297),
            (41.39080, -73.95253),
            (41.39172, -73.95208)
        ]
        rearrangedBounds = [self.bounds[2], self.bounds[0], self.bounds[1], self.bounds[3]]
        self.contourLine = ContourLine.fill(rearrangedBounds, simulationData)
        kml.generate()
        kml.addGraph(self.contourLine.graph)
        kml.save("wrath_rad")

        name = datetime.datetime.now().strftime("%y:%m:%d:%H:%M")
        logging.basicConfig(filename="logs/radDetection - " + name, level=logging.INFO, 
            format="%(levelname)s:%(message)s")
        logging.getLogger().addHandler(logging.StreamHandler(sys.stdout))
        logging.info("Rad Detection Starting")

        logging.info(str([v.coord for v in self.contourLine.graph.vertices]))
        logging.info(str(self.contourLine.graph))
        self.numDronesInSwarm = 0
        self.dronesFinished = 0
        if simulationData != None:
            self.IS_SIMULATION = True
            self.simulationData = simulationData
        else:
            self.IS_SIMULATION = False   
        
        self.registerMessageCallback(msgs.StartInitPassMessage.id,
                                     self.onReceiveStartInitPass)
        self.registerMessageCallback(msgs.FinishInitPassMessage.id,
                                     self.onReceiveFinishInitPass)
        self.registerMessageCallback(msgs.RadLocationMessage.id,
                                     self.onReceiveRadLocation)
        self.registerMessageCallback(msgs.LaneUpdateMessage.id,
                                     self.onReceiveLaneUpdate)
        self.registerMessageCallback(msgs.RadiationMessage.id,
                                     self.onReceiveRadiation)
        self.registerMessageCallback(rrmsgs.LogMessage.id,
                                     self.onReceiveLog)


    def onReceiveStartInitPass(self, message):
        if self.state == 0:
            logging.info("MODEL: State Change: Initial Pass!")
            self.state = 1
        self.numDronesInSwarm += 1


    def onReceiveFinishInitPass(self, message):
        self.dronesFinished += 1
        logging.info("MODEL: Drone Finished Init Pass")
        if (self.dronesFinished == self.numDronesInSwarm):
            logging.info("MODEL: State Change: Lane Generation!")
            logging.info("MODEL: Sending points: " + str(self.simulationData))
            self.state = 2
            if self.IS_SIMULATION:
                messageParser = msgs.StartLaneGenerationMessage()
                messageParser.contourPoints = self.simulationData
                self.broadcast(messageParser)


    def onReceiveRadLocation(self, message):
        self.contourLine.updateContour(message.location, message.direction)
        kml.generate()
        kml.addGraph(self.contourLine.graph)
        kml.save("wrath_rad")
        logging.info(str(self.contourLine.graph))
        logging.info("MODEL: Update Contour Line: " + str(message.location))
        logging.info("MODEL: Error Calculated: " + str(message.error))
    

    def onReceiveLaneUpdate(self, message):
        logging.info("MODEL: New Lane: " + str(message.start) + " " + str(message.center) + " " + str(message.end))
        
        # Test Code
        self.lanes.append((self.contourLine.graph.copy(), message.start, message.end))
        
        kml.generate()
        for i in range(len(self.lanes)):
            contour = self.lanes[i][0]
            start = self.lanes[i][1]
            end = self.lanes[i][2]
            kml.addPoint((start[1], start[0]), "Lane: " + str(i) + " Start")
            kml.addPoint((end[1], end[0]), "Lane: " + str(i) + " End")
            kml.addGraph(contour)
        kml.save("lane_testing")


    def onReceiveRadiation(self, message):
        pass
        #logging.info("Received Rad!: " + str(message.time) + " " + str(message.count))


    def onReceiveLog(self, message):
        global altitudeTime
        if ("altitude" in message.msg.lower()):
            # print "happening!!!!"
            # print time.time()
            if (time.time() - altitudeTime < 1):
                # print "Next point!!!"
                return
            altitudeTime = time.time()        
        logging.info(message.msg)
