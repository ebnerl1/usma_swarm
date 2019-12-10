from geopy import distance

class Vertex:

  def __init__(self, gpsCoord):
    self.id = -1
    self.coord = gpsCoord


def findClosestVertices(lstA, lstB):
  closest = (10000000, (lstA[0], lstB[0]))
  for i in range(len(lstA)):
    for j in range(len(lstB)):
      start = lstA[i]
      end = lstB[j]
      dist = distance.distance(start.coord, end.coord).km
      if (dist < closest[0]):
        closest = (dist, (start, end))
  return closest[1]
