#!/usr/bin/python

import enum

from WrathServerModel import Server

# Client Messages:
# 0: Heartbeat
# 1: Start Analysis: droneId
# 2: Road Analyzed: (start, end)
# 3: Object Found (location, picture, size?)
class MessageType(enum.IntEnum):
    Heartbeat = 0
    StartAnalysis = 1
    RoadAnalyzed = 2
    ObjectFound = 3

class RouteReconServer(Server.Server):

    def __init__(self):
        super(RadDetectionServer, self).__init__()

        # TODO: init graph data structure

    
    def handleMessageData(self, data):
        message = data[0]

        if  message == MessageType.RoadAnalyzed:
            # TODO: Handle road analyzed
            pass

        elif message == MessageType.ObjectFound:
            # TODO: Handle object found message
            pass
        
        return [0]

