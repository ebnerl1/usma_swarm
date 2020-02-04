#!/usr/bin/env python

import ap_lib.gps_utils as gps

from Collections import LineSegment

def calculateDistance(start, end):
  return gps.gps_distance(start[0], start[1], end[0], end[1])

class Edge(object):
  # start: Vertex object
  # end: Vertex object
  # calculateDistance: lambda gps coord, gps coord => distance
  def __init__(self, start, end, isRoad = False):
    self.start = start
    self.end = end
    self.distance = calculateDistance(start.coord, end.coord)
    self.isRoad = isRoad
  
  def __repr__(self):
    return "Edge(" + str(self.start.id) + ", " + str(self.end.id) + ")"

  def __lt__(self, other):
    return self.distance < other.distance
  
  def __le__(self, other):
    return self.distance <= other.distance

  def getNormal(self):
    return LineSegment.LineSegment(self.start.coord, self.end.coord).getNormal()
