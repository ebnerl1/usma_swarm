#!/usr/bin/pyhton
#Malik "Herbie" Hancock 4February2020
import simplekml
import math
import time
# import gmaps
#import gmplot 
#create kml
# add point, line segment
# save 

from Collections import Graph
import HeatmapGenerator

class WrathKML():

	def __init__(self):
		self.outPath = "WrathServerModel/Output/"
		# self.outPath = ""

		self.bounds = [
			40.78887,
			-73.96360, 
			40.80034, 
			-73.94969
		]

		self.clear()
		self.clearHeatmap()


	def clear(self):
		self.kml = simplekml.Kml()
		self.kml.networklinkcontrol.minrefreshperiod = 1


	def clearHeatmap(self):
		self.heatmapGen = HeatmapGenerator.HeatmapGenerator((1024, 1024), self.bounds, 0.18)


	def addFolder(self, name = ""):
		return self.kml.newfolder(name=name)

	
	def addPoint(self, point, kml = None, name = ""):
		if (kml == None):
			kml = self.kml
		pnt = kml.newpoint(name=name)
 		pnt.coords = [point]
		pnt.style.iconstyle.icon.href = 'http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png'


	def addLine(self, start, end, kml = None, color = 3, name = ""):
		if (kml == None):
			kml = self.kml
		ls = kml.newlinestring(name=name)
		ls.coords = [start,end]
		ls.style.linestyle.width = 5
		if color == 0:
			ls.style.linestyle.color = simplekml.Color.red #if vehicle is detected along path, road is not traversable
		elif color == 1:
			ls.style.linestyle.color = simplekml.Color.green #if no vehicle detected, road is traversable
		else:
			ls.style.linestyle.color = simplekml.Color.white # default for Route Detection, if the road has not yet been traversed. 
			# also used for the creation of Contour lines for radiation swarming algorithm.


	def addGraph(self, graph, color = 3, name = "", drawPoints = True):
		folder = self.addFolder(name)
		if (drawPoints):
			for vertex in graph.vertices:
				self.addPoint((vertex.coord[1], vertex.coord[0]), folder)
		for edge in graph.getEdges():
			start = (edge.start.coord[1], edge.start.coord[0])
			end = (edge.end.coord[1], edge.end.coord[0])
			self.addLine(start, end, folder, color)


	def addHeatMapPoint(self, location, radCount):
		self.heatmapGen.processRad(location, radCount)


	def addHeatmap(self, name):
		self.heatmapGen.saveImage(self.outPath + name + ".png")

		ground = self.kml.newgroundoverlay(name=name)
		ground.icon.href = name + ".png"
		ground.latlonbox.north = self.bounds[0]
		ground.latlonbox.south = self.bounds[2]
		ground.latlonbox.east = self.bounds[3]
		ground.latlonbox.west = self.bounds[1]
		ground.latlonbox.rotation = 0.0


	def save(self, name):
		self.addHeatmap(name)
		self.kml.save(self.outPath + name + ".kml")
