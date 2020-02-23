from Server import Server

import struct
import time
import thread

class TestServer(Server):

    def __init__(self):
        super(TestServer, self).__init__()


    def handleMessageData(self, data):
        i1, f, i2 = struct.unpack("!lfl", "".join(data))
        print "Message: ", i1, f, i2


if __name__ == '__main__':
    server = TestServer()
    thread.start_new_thread(server.start, ('127.0.0.1', 10000))
    timeBtwnMsg = 1
    msgTime = time.time()
    while True:
        if (len(server.connections) > 0):
            if time.time() > timeBtwnMsg + msgTime:
                msgTime = time.time()

                print ("Sending message")
                msg = struct.pack("!flf", 1.23, 4, 4.55)
                server.broadcast(msg)
                
