#!/usr/bin/pyhton
#Malik "Herbie" Hancock 4February2020
import simplekml
import math
import time
import gmaps
#import gmplot 
#create kml
# add point, line segment
# save 

from Collections import Graph

kml = None
fig = gmaps.figure() #gmap is the work around for blending the heatmap
# fig is a different file than the kml

#ignore line below
#gmap = gmplot.GoogleMapPlotter.from_geocode("Central Park, New York, NY") #this is the location for simulation

#building kml
def generate():
    global kml
    kml = simplekml.Kml()
    kml.networklinkcontrol.minrefreshperiod = 1

def addFolder(name = ""):
	return kml.newfolder(name=name)

def addPoint(point, name = ""):
	pnt = kml.newpoint(name=name)
 	pnt.coords = [point]
 	pnt.style.iconstyle.icon.href = 'http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png'

def addLine(start,end, kml, color = 3, name = ""):
	ls = kml.newlinestring(name=name)
	ls.coords = [start,end]
	if color == 0:
		ls.style.linestyle.color = simplekml.Color.red #if vehicle is detected along path, road is not traversable
	elif color == 1:
		ls.style.linestyle.color = simplekml.Color.green #if no vehicle detected, road is traversable
	else:
		ls.style.linestyle.color = simplekml.Color.white # default for Route Detection, if the road has not yet been traversed. 
		# also used for the creation of Contour lines for radiation swarming algorithm.

def addGraph(graph, color = 3, name = ""):
	folder = addFolder(name)
	for vertex in graph.vertices:
		point = folder.newpoint()
		point.coords = [(vertex.coord[1], vertex.coord[0])]
		point.style.iconstyle.icon.href = 'http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png'
	for edge in graph.getEdges():
		start = (edge.start.coord[1], edge.start.coord[0])
		end = (edge.end.coord[1], edge.end.coord[0])
		addLine(start, end, folder, color)

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

#def blendHeat(point):


# This is the gmap implementation
gmaps.configure(api_key='AIzaSyDCIInp2RvfMuzcktjNNeIWhjoygB1zzoc') # Fill in with your API key

def blendHeat(point,count): #point is a tuple (lat,lon)
	#fig = gmaps.figure() 
	fig.add_layer(gmaps.heatmap_layer(point, weights=count))
	#fig

# need to save it to a destination for viewing
def save(name):
	kml.save("/home/user1/usma_swarm/usma_files/serverSide/archive/" + name + ".kml")

def show(): #shows the current heatmap different than kml saved should be able to add layers though.
	fig
