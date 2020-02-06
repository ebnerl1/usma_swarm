#!/usr/bin/pyhton
#Malik "Herbie Hancock" 4February2020
import simplekml
import math
import time
#create kml
# add point, line segment
# save 

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

# need to save it to a destination for viewing
def save():
	kml.save("/home/user1/usma_swarm/usma_files/serverSide/archive/wrath_heatmap.kml")
