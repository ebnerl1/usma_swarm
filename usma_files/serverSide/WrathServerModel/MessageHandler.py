import struct

import RouteReconMessages as rrmsgs
import RadDetectionMessages as rdmsgs

msgList = [
    rrmsgs.LocationMessage,
    rrmsgs.StartBehaviorMessage,
    rrmsgs.RoadAnalyzedMessage,
    rrmsgs.InitGraphMessage,
    rrmsgs.DetectedObjectMessage,
    rrmsgs.LogMessage,

    rdmsgs.SparsityMessage,
    rdmsgs.RadLocationMessage,
    rdmsgs.LockLaneGenMessage,
    rdmsgs.UnlockLaneGenMessage,
    rdmsgs.StartInitPassMessage,
    rdmsgs.FinishInitPassMessage,
    rdmsgs.LaneUpdateMessage,
    rdmsgs.StartLaneGenerationMessage,
    rdmsgs.RadiationMessage
]

idToMessage = { msg.id: msg for msg in msgList }

class MessageHandler(object):

    def __init__(self):
        self.messageCallbacks = dict()


    def registerCallback(self, id, callback):
        self.messageCallbacks[id] = callback
    

    def processMessage(self, data):
        messageId = struct.unpack("!l", data[:struct.calcsize("!l")])[0]

        if messageId in self.messageCallbacks:
            message = idToMessage[messageId]()
            message.unpack(data)        
            self.messageCallbacks[messageId](message)
