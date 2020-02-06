#!/usr/bin/env python

import ap_lib.gps_utils as gps

class Vertex(object):

  def __init__(self, gpsCoord):
    self.id = -1
    self.coord = gpsCoord


def findClosestVertices(lstA, lstB):
  closest = (10000000, (lstA[0], lstB[0]))
  for i in range(len(lstA)):
    for j in range(len(lstB)):
      start = lstA[i]
      end = lstB[j]
      dist = gps.gps_distance(start.coord[0], start.coord[1], end.coord[0], end.coord[1])
      if (dist < closest[0]):
        closest = (dist, (start, end))
  return closest[1]
