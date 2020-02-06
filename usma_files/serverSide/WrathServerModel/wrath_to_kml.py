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
		addPoint(vertex.coord)
	for edge in graph.getEdges():
		addLine(edge.start.coord, edge.end.coord)

# need to save it to a destination for viewing
def save():
	kml.save("/home/user1/usma_swarm/usma_files/serverSide/archive/wrath_heatmap.kml")
