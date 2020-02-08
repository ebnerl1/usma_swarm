#!/usr/bin/python

import enum
from WrathServerModel.Collections import ContourLine 
from WrathServerModel import Server
from WrathServerModel import wrath_to_kml as kml

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
        try:
            messageType = data[0]
        except:
            print "Exception!!! ", data
            return [0]
            

        if messageType == MessageType.Heartbeat:
            pass            

        elif messageType == MessageType.StartInitPass:
            if self.state == 0:
                print "MODEL: State Change: Initial Pass!"
                self.state = 1
            self.numDronesInSwarm += 1

        elif messageType == MessageType.RadiationData:
            # TODO: Please implement this!
            pass

        elif messageType == MessageType.FinishInitPass:
            self.dronesFinished += 1
            print "MODEL: Drone Finished Init Pass"
            if (self.dronesFinished == self.numDronesInSwarm):
                print "MODEL: State Change: Lane Generation!"
                print "MODEL: Sending points: ", self.simulationData
                self.state = 2
                if self.IS_SIMULATION:
                    self.connectionLock.acquire()
                    time = 0
                    for connection in self.connections:
                        connection.send(str([1, self.simulationData, time]))
                        time += 5
                    self.connectionLock.release()
#                    self.broadcast(str([1, self.simulationData]))
                else:
                    # Broadcast real data
                    pass
        elif messageType == MessageType.UpdateContour:
            self.contourLine.updateContour(data[1])
            kml.generate()
            kml.addGraph(self.contourLine.graph)
            kml.save("wrath_rad")
            print self.contourLine.graph
            print "MODEL: Update Contour Line: ", data[1]
            print "MODEL: Error Calculated: ", data[2]

        elif messageType == MessageType.LaneUpdate:
            print "MODEL: New Lane: ", data[1], data[2], data[3]
        
        return [0]

        

