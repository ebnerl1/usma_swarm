#!/usr/bin/env python

from Collections import Vertex
from Collections import Edge
import Queue

# fills graph and returns it
# vertices is a list of coords
# edges is a list of tuples of vertex indices
def fill(vertices, edges, areRoads = False):
  graph = Graph()
  for vertex in vertices:
    graph.addVertex(Vertex.Vertex(vertex))

  for (start, end) in edges:
    graph.addEdge(Edge.Edge(graph.vertices[start], graph.vertices[end], areRoads))
  return graph

# uses a Vertex object. If you use your own vertices, please override
# vertex class
class Graph(object):

  # adjacencies: list[(start, end, weight)]
  def __init__(self):
    self.nVertices = 0
    self.nEdges = 0
    # vertex list: maps ID to vertex object
    self.vertices = list()
    # map vertex id to set of adjacent vertex ids
    self.adjacencies = dict()
    # map start and end tuple to edge object
    self.edges = dict()
    # map start and end tuple to number of duplicates
    # when duplicates added, increment,
    # when edges removed, duplicates decremented first
    self.duplicates = dict()
    # start vertex if there is one
    self.start = None

  def __repr__(self):
    return "Graph(" + repr(self.adjacencies) + ")"

  # Shallow copy
  def copy(self):
    newGraph = Graph()
    for vertex in self.vertices:
      newGraph.addVertex(vertex)
    for edge in self.edges.values():
      newGraph.addEdge(edge)
    return newGraph

  # assumes that we are not reclaiming vertices
  def addVertex(self, vertex):
    vertex.id = self.nVertices
    self.vertices.append(vertex)
    self.adjacencies[self.nVertices] = set()
    self.nVertices += 1

  def addEdge(self, edge):
    self.nEdges += 1
    (v1, v2) = (edge.start.id, edge.end.id)
    (lo, hi) = (v1, v2) if v2 > v1 else (v2, v1)

    if (max(v1, v2) >= self.nVertices):
      raise IndexError("Invalid Vertex Index")

    if (v2 in self.adjacencies[v1]):
      self.duplicates[(lo, hi)] += 1
      return

    self.adjacencies[v1].add(v2)
    if (v1 != v2):
      self.adjacencies[v2].add(v1)    
    self.edges[(lo, hi)] = edge
    self.duplicates[(lo, hi)] = 0

  def removeEdge(self, edge):
    self.nEdges -= 1
    (v1, v2) = (edge.start.id, edge.end.id)
    (lo, hi) = (v1, v2) if v2 > v1 else (v2, v1)

    if (self.duplicates[(lo, hi)] > 0):
      self.duplicates[(lo, hi)] -= 1
    else:
      self.adjacencies[v1].remove(v2)
      if (v1 != v2):
        self.adjacencies[v2].remove(v1)
      self.edges.pop((lo, hi))
      self.duplicates.pop((lo, hi))

  def getVertex(self, id):
    if (id < self.nVertices):
      return self.vertices[id]
    else:
      raise IndexError("Invalid Vertex Index")

  def getVertexByCoord(self, coord):
    for v in self.vertices:
      if (coord[0] - v.coord[0] < .0000001 and coord[1] - v.coord[1] < .0000001):
        return v
    return None

  def getVertexDegree(self, vertex):
    return len(self.adjacencies[vertex.id])

  # between two ids
  def getEdge(self, v1, v2):
    (lo, hi) = (v1, v2) if v2 > v1 else (v2, v1)
    return self.edges[(lo, hi)]

  def getEdges(self):
    return list(self.edges.values())

  # between two vertices
  def hasEdge(self, v1, v2):
    (lo, hi) = (v1.id, v2.id) if v2.id > v1.id else (v2.id, v1.id)
    return (lo, hi) in self.edges

  # returns set[(vertex, edge)]
  def getNeighbors(self, vertex):
    if (vertex.id not in self.adjacencies):
      raise IndexError("Invalid Vertex Index: " + str(vertex.id))

    neighbors = list(self.adjacencies[vertex.id])
    return [(self.vertices[i], self.getEdge(vertex.id, i)) for i in neighbors]

  # connects each vertex in vertices to every other vertex
  # in the graph except for those in vertices
  def connectVertices(self, vertices):
    graphVertices = self.vertices[:]
    for vertex in vertices:
      self.addVertex(vertex)
    for start in vertices:
      for end in graphVertices:
        newEdge = Edge.Edge(start, end)
        self.addEdge(newEdge)
  
  # TODO: Add this to the graph that's in the scrimmage repo
  # adds vertex, then connects the closest vertex
  def connectClosestVertex(self, vertex):
    self.addVertex(vertex)
    closestDist, closestEdge = (100000, None)
    for v in self.vertices[:-1]:
      edge = Edge.Edge(vertex, v)
      if (edge.distance < closestDist):
        closestDist = edge.distance
        closestEdge = edge
    self.addEdge(closestEdge)

  # connects disconnected subgraphs in graph
  # with shortest edge possible
  def connectSubgraphs(self):
    subgraphs = self.findSubgraphs()

    for i in range(len(subgraphs)):
      for j in range(i + 1, len(subgraphs)):
        (vertA, vertB) = Vertex.findClosestVertices(subgraphs[i], subgraphs[j])
        newEdge = Edge.Edge(vertA, vertB, calculateDistance)
        self.addEdge(newEdge)

  # returns a list of list of vertices in each subgraph
  def findSubgraphs(self):
    total = set(range(self.nVertices))
    seen = set()
    subgraphs = list()

    diff = total - seen
    while (len(diff) > 0):
      subgraph = self.collectSubgraph(self.getVertex(list(diff)[0]))
      subgraphs.append(subgraph)
      for vertex in subgraph:
        seen.add(vertex.id)
      diff = total - seen
    return subgraphs

  # returns a list of vertices in subgraph containing vertex
  def collectSubgraph(self, vertex):
    seen = set()
    frontier = Queue.Queue()
    frontier.put(vertex)

    while (not frontier.empty()):
      node = frontier.get()

      if (node in seen):
        continue

      seen.add(node)
      
      for neighbor in self.getNeighbors(node):
        if (neighbor[0] not in seen):
          frontier.put(neighbor[0])
    return list(seen)


