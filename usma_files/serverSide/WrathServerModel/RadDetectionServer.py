#!/usr/bin/python

import enum
from WrathServerModel.Collections import ContourLine 
from WrathServerModel import Server
from WrathServerModel import wrath_to_kml as kml
from WrathServerModel import RadDetectionMessages
import struct

# Client Messages:
# 0: Heartbeat: No Data
# 1: Starting Initial Pass: No Data
# 2: Sending Radiation Data: TODO: data
# 3: Finished Initial Pass: No Data
# 4: Update Contour Line: (lat, lon), time
# 5: Lane Update: Start, Center, End
#
# Server Messages:
# 0: Heartbeat: No Data
# 1: Begin Lane Generation: list((lat, lon))
#
# TODO: get this out of this class?
class MessageType(enum.IntEnum):
    Heartbeat = 0
    StartInitPass = 1
    RadiationData = 2
    FinishInitPass = 3
    UpdateContour = 4
    LaneUpdate = 5

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


    def handleMessageData(self, data):
        data = "".join(data)
        id = struct.unpack("!l", data[:4])[0]
        data = data[4:]
            
        # Start init pass
        if id == 1:
            parser = RadDetectionMessages.StartInitPassMessage()
            parser.unpack(data)

            if self.state == 0:
                print "MODEL: State Change: Initial Pass!"
                self.state = 1
            self.numDronesInSwarm += 1

        # rad data
        elif id == 2:
            # TODO: Please implement this!
            pass

        # finished init pass
        elif id == 3:
            parser = RadDetectionMessages.FinishInitPassMessage()
            parser.unpack(data)

            self.dronesFinished += 1
            print "MODEL: Drone Finished Init Pass"
            if (self.dronesFinished == self.numDronesInSwarm):
                print "MODEL: State Change: Lane Generation!"
                print "MODEL: Sending points: ", self.simulationData
                self.state = 2
                if self.IS_SIMULATION:
                    messageParser = RadDetectionMessages.StartLaneGenerationMessage()
                    messageParser.contourPoints = self.simulationData
                    data = struct.pack("!l", 6) + messageParser.pack()            
                    self.broadcast(data)

                else:
                    # Broadcast real data
                    pass
        
        # update contour line
        elif id == 4:
            parser = RadDetectionMessages.UpdateContourLineMessage()
            parser.unpack(data)

            self.contourLine.updateContour(parser.location)
            kml.generate()
            kml.addGraph(self.contourLine.graph)
            kml.save("wrath_rad")
            print self.contourLine.graph
            print "MODEL: Update Contour Line: ", parser.location
            print "MODEL: Error Calculated: ", parser.error

        # lane update
        elif id == 5:
            parser = RadDetectionMessages.LaneUpdateMessage()
            parser.unpack(data)

            print "MODEL: New Lane: ", parser.start, parser.center, parser.end

        

