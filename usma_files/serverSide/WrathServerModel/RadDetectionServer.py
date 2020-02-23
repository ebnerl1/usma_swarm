#!/usr/bin/python

import enum
from WrathServerModel.Collections import ContourLine 
from WrathServerModel import Server
from WrathServerModel import wrath_to_kml as kml
from WrathServerModel import RadDetectionMessages as msgs
import struct

class RadDetectionServer(Server.Server):

    def __init__(self, simulationData = None):
        super(RadDetectionServer, self).__init__()
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
        print [v.coord for v in self.contourLine.graph.vertices]
        print self.contourLine.graph
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


    def onReceiveStartInitPass(self, message):
        if self.state == 0:
            print "MODEL: State Change: Initial Pass!"
            self.state = 1
        self.numDronesInSwarm += 1
    

    def onReceiveFinishInitPass(self, message):
        self.dronesFinished += 1
        print "MODEL: Drone Finished Init Pass"
        if (self.dronesFinished == self.numDronesInSwarm):
            print "MODEL: State Change: Lane Generation!"
            print "MODEL: Sending points: ", self.simulationData
            self.state = 2
            if self.IS_SIMULATION:
                messageParser = msgs.StartLaneGenerationMessage()
                messageParser.contourPoints = self.simulationData
                self.broadcast(messageParser)
    

    def onReceiveRadLocation(self, message):
        self.contourLine.updateContour(message.location)
        kml.generate()
        kml.addGraph(self.contourLine.graph)
        kml.save("wrath_rad")
        print self.contourLine.graph
        print "MODEL: Update Contour Line: ", message.location
        print "MODEL: Error Calculated: ", message.error
    

    def onReceiveLaneUpdate(self, message):
        print "MODEL: New Lane: ", message.start, message.center, message.end       

