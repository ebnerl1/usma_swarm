#!/usr/bin/python

from WrathServerModel import RouteReconServer

Server = RouteReconServer.RouteReconServer()

print "-----SIMULATION-----"
Server.start("127.0.0.1", 10000)