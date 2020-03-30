#!/usr/bin/python

from WrathServerModel import RouteReconServer

IS_SIMULATION = True

Server = RouteReconServer.RouteReconServer()

print "-----SIMULATION-----"
ipAddress = "127.0.0.1" if (IS_SIMULATION) else "192.168.11.202"
Server.start(ipAddress, 10000)
