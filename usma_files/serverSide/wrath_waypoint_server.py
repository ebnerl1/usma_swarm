#!/usr/bin/python

from WrathServerModel.RadDetectionServer import RadDetectionServer as Server

IS_SIMULATION = False

simulationData = [(41.39126646, -73.95287588),
				  (41.39155737, -73.95258772)]

Server = Server(simulationData)

print "-----SIMULATION-----"
ipAddress = "127.0.0.1" if (IS_SIMULATION) else "192.168.11.202"
Server.start(ipAddress, 10000)
