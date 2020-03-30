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

def addHeat(point,rad_info): #point is a tuple of (lat,long)
	pol = kml.newpolygon()
	width = 0.5 # change these parameters for styling
	height = 2 # change these paramaters for styling
	lon = point[1]
	lat = point [0]
	#Code below is a snippet pulled from AY19 team for creating polygon
	top_left_x = (lon - width/2)
	top_left_y = (lat + height/2)
	bottom_left_x = (lon - width/2)
	bottom_left_y = (lat - height/2)
	bottom_right_x = (lon + width/2)
	bottom_right_y = (lat- height/2)
	top_right_x = (lon + width/2)
	top_right_y = (lat + height/2)
	pol.outerboundaryis = [(top_left_x,top_left_y), (top_right_x,top_right_y),
	 (bottom_right_x,bottom_right_y), (bottom_left_x,bottom_left_y),(top_left_x,top_left_y)]

	if (rad_info < 30):
		pol.style.polystyle.color = simplekml.Color.changealphaint(100, simplekml.Color.blue)

	elif (100 >= rad_info > 30):
		pol.style.polystyle.color = simplekml.Color.changealphaint(100, simplekml.Color.green)

	elif (400 >= rad_info > 100):
		pol.style.polystyle.color = simplekml.Color.changealphaint(100, simplekml.Color.yellow)

	else:
		pol.style.polystyle.color = simplekml.Color.changealphaint(100, simplekml.Color.red)
       

 

# need to save it to a destination for viewing
def save(name):
	kml.save("/home/user1/usma_swarm/usma_files/serverSide/archive/" + name + ".kml")
