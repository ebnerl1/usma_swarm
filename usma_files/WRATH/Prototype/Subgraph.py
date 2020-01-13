import Graph
import Vertex

# TODO: Make this a persistent data structure and remove
# copyWith method
class Subgraph:
  def __init__(self, graph):
    self.graph = graph
    # set of indices
    self.usedVertices = set()
    # set of edge vertex indices
    self.usedEdges = set()
  
  # doesn't really mean anything
  def __lt__(self, other):
    id(self) < id(other)
  
  def __le__(self, other):
    id(self) <= id(other)

  def __str__(self):
    return "Subgraph: " + repr(self.usedEdges)

  def __repr__(self):
    return "Subgraph: " + repr(self.usedEdges)

  def toGraph(self):
    vertexLst = [self.graph.getVertex(i) for i in list(self.usedVertices)]
    vertexMap = dict()
    for i in range(len(vertexLst)):
      vertexMap[vertexLst[i].id] = i
    print(vertexMap, "\n")
    newVertices = [v.coord for v in vertexLst]
    newEdges = [(vertexMap[edge[0]], vertexMap[edge[1]]) for edge in list(self.usedEdges)]
    return Graph.fill(newVertices, newEdges)

  def copy(self):
    subgraph = Subgraph(self.graph)
    subgraph.usedVertices = self.usedVertices.copy()
    subgraph.usedEdges = self.usedEdges.copy()
    return subgraph

  def copyWith(self, edge):
    subgraph = self.copy()
    subgraph.addEdge(edge)
    return subgraph

  def setCost(self, cost):
    self.cost = cost
    return self

  def getEdges(self):
    return [self.graph.getEdge(v1, v2) for (v1, v2) in self.getEdgeIndices()]

  def getEdgeIndices(self):
    return list(self.usedEdges)

  def addVertex(self, vertex):
    self.usedVertices.add(vertex.id)

  def addEdge(self, edge):
    if (edge[0] not in self.usedVertices):
      self.usedVertices.add(edge[0])
    if (edge[1] not in self.usedVertices):
      self.usedVertices.add(edge[1])
    self.usedEdges.add(edge)

  def getNeighbors(self):
    neighbors = list()
    for vertex in list(self.usedVertices):
      for (end, edge) in self.graph.getNeighbors(self.graph.getVertex(vertex)):
        (lo, hi) = (vertex, end.id) if (vertex < end.id) else (end.id, vertex)
        if (not (lo, hi) in self.usedEdges):
          neighbors.append(self.copyWith((lo, hi)))
    return neighbors

  def getSetRepr(self):
    return tuple(self.usedEdges.copy())