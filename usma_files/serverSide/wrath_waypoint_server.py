#!/usr/bin/python

from WrathServerModel.RadDetectionServer import RadDetectionServer as Server

simulationData = [(41.39126646, -73.95287588),
				  (41.39155737, -73.95258772)]

Server = Server(simulationData)

print "-----SIMULATION-----"
Server.start("127.0.0.1", 10000)
