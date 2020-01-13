from geopy import distance

class Edge:

  # start: Vertex object
  # end: Vertex object
  def __init__(self, start, end, isRoad = False):
    self.start = start
    self.end = end
    self.distance = distance.distance(start.coord, end.coord).km
    self.isRoad = isRoad
  
  def __lt__(self, other):
    return self.distance < other.distance
  
  def __le__(self, other):
    return self.distance <= other.distance