#!/usr/bin/pyhton
#Malik "Herbie Hancock" 4February2020
import simplekml
import math
import time
#create kml
# add point, line segment
# save 

from Collections import Graph

kml = None

#building kml
def generate():
    global kml
    kml = simplekml.Kml()
    kml.networklinkcontrol.minrefreshperiod = 1

def addPoint(point):
	pnt = kml.newpoint()
 	pnt.coords = [point]

def addLine(start,end):
	ls = kml.newlinestring()
	ls.coords = [start,end]

def addGraph(graph):
	for vertex in graph.vertices:
		addPoint((vertex.coord[1], vertex.coord[0]))
	for edge in graph.getEdges():
		start = (edge.start.coord[1], edge.start.coord[0])
		end = (edge.end.coord[1], edge.end.coord[0])
		addLine(start, end)

# need to save it to a destination for viewing
def save(name):
	kml.save("/home/user1/usma_swarm/usma_files/serverSide/archive/" + name + ".kml")
