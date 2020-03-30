#!/usr/bin/pyhton
#Malik "Herbie" Hancock 4February2020
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

def addLine(start,end, color = 3):
	ls = kml.newlinestring()
	ls.coords = [start,end]
	if color == 0:
		ls.style.linestyle.color = simplekml.Color.red #if vehicle is detected along path, road is not traversable
	elif color == 1:
		ls.style.linestyle.color = simplekml.Color.green #if no vehicle detected, road is traversable
	else:
		ls.style.linestyle.color = simplekml.Color.white # default for Route Detection, if the road has not yet been traversed. 
		# also used for the creation of Contour lines for radiation swarming algorithm.

def addGraph(graph, color = 3):
	for vertex in graph.vertices:
		addPoint((vertex.coord[1], vertex.coord[0]))
	for edge in graph.getEdges():
		start = (edge.start.coord[1], edge.start.coord[0])
		end = (edge.end.coord[1], edge.end.coord[0])
		addLine(start, end, color)

def addImage(point,alt,image):
	photo = kml.newphotooverlay()
	photo.camera = simplekml.Camera(longitude = point[1], latitude = point[0], altitude = alt,
	 altitudemode=simplekml.AltitudeMode.clamptoground)
	photo.point.coords = [point]
	photo.style.iconstyle.icon.href = 'http://maps.google.com/mapfiles/kml/shapes/camera.png' #creates camera icon we can click on
	# need to create turn image into a url location to grab
	#photo.icon.href= image
	photo.viewvolume = simplekml.ViewVolume(-25,25,-15,15,1)
 

# need to save it to a destination for viewing
def save(name):
	kml.save("/home/user1/usma_swarm/usma_files/serverSide/archive/" + name + ".kml")
