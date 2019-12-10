import Graph
import Vertex
import Edge
from geopy import distance

print("\n")

def testNeighbor(graph, vertex, adjacentVertices):
  neighbors = graph.getNeighbors(graph.getVertex(vertex))
  vertices = {vertex.id for (vertex, edge) in neighbors}

  for adj in adjacentVertices:
    if (adj not in vertices):
      return False
  return True

def testConnectedEdge(graph, vertex, nConnections):
  return len(graph.getNeighbors(vertex)) == nConnections

def dist(graph, v1, v2):
  return graph.edges[(v1, v2)].distance

vertices = [
  (33.22619166666667, -81.58043611111111),
  (33.22956388888889, -81.58343888888888),
  (33.22983611111111, -81.58292777777777),
  (33.22979722222222, -81.58360277777777),
  (33.23055833333334, -81.58235833333333),
  (33.22979166666667, -81.5816611111111),
  (33.22959722222223, -81.58150277777777),
  (33.23105, -81.58157222222222),
  (33.23026388888889, -81.58091388888889),
  (33.22989444444445, -81.58059166666666),
  (33.22961666666667, -81.58099166666666),
  (33.22894166666667, -81.58210555555556),
  (33.22802777777778, -81.58156111111111),
  (33.22921388888889, -81.58062777777778),
  (33.22920833333333, -81.58038888888889),
  (33.22881388888889, -81.58003611111111),
  (33.22896666666667, -81.57977222222222),
  (33.22853611111111, -81.57980555555555),
  (33.22804166666667, -81.57991944444444),
  (33.22839444444445, -81.57926666666667),
  (33.22802777777778, -81.57751111111111)
]

edges = [
  (0, 1),
  (1, 2),
  (1, 3),
  (3, 4),
  (4, 5),
  (5, 6),
  (5, 8),
  (4, 7),
  (7, 8),
  (8, 9),
  (9, 10),
  (10, 11),
  (11, 12),
  (10, 13),
  (13, 14),
  (14, 15),
  (9, 16),
  (15, 16),
  (15, 17),
  (17, 18),
  (18, 19),
  (16, 19),
  (19, 20),
  (0, 20)
]

adjacencies = [list() for i in range(21)]
for (start, end) in edges:
  adjacencies[start].append(end)
  adjacencies[end].append(start)

dronePositions = [
  (33.227192, -81.5804365),
  (33.226192, -81.580437)
]
droneVertices = [Vertex.Vertex(pos) for pos in dronePositions]

graph = Graph.fill(vertices, edges)

# Test all adjacencies
print("-----Testing Adjacencies-----")
for i in range(len(adjacencies)):
  print(testNeighbor(graph, i, adjacencies[i]))


# Test edge distances
print("\n-----Testing Distances-----")
print(abs(dist(graph, 0, 1) * 1000 - 465) < 8)
for edge in edges:
  print(edge[0], edge[1], dist(graph, edge[0], edge[1]) * 1000)

print("\n-----Testing Connected Vertex-----")

# Test connecting vertices to graph
connectedVertexGraph = graph.copy()

connectedVertexGraph.connectVertices(droneVertices)

for vertex in droneVertices:
  print(testConnectedEdge(connectedVertexGraph, vertex, connectedVertexGraph.nVertices - 2))

# Test finding subgraphs
print("\n-----Testing Subgraphs-----")

subVerts = [(0, 0) for i in range(10)]

subPairs = [
  (0, 1),
  (1, 2),
  (3, 4),
  (6, 7),
  (6, 8),
  (6, 9),
  (7, 8),
  (7, 9),
  (8, 9)
]

sizes = {1, 2, 3, 4}

disconnectedSubgraphs = Graph.fill(subVerts, subPairs)

subgraphs = disconnectedSubgraphs.findSubgraphs()
print(len(subgraphs) == 4)
for subgraph in subgraphs:
  print(len(subgraph) in sizes)
  sizes.remove(len(subgraph))

# Test closest vertices
print("\n-----Testing Closest Vertices-----")
# A and B: 2 and 5 are closest
# A and C: 2 and 4 are closest
# B and C: 9 and 8 are closest
vertsAID = [0, 1, 2, 3]
vertsBID = [5, 7, 9, 10]
vertsCID = [4, 8, 19, 14]

vertsA = [graph.getVertex(a) for a in vertsAID]
vertsB = [graph.getVertex(b) for b in vertsBID]
vertsC = [graph.getVertex(c) for c in vertsCID]

# This was to test the distances to find the correct ones
# for a in vertsB:
#   for b in vertsC:
#     print(a.id, b.id, distance.distance(a.coord, b.coord).km * 1000)

(vertA, vertB) = Vertex.findClosestVertices(vertsA, vertsB)
(a, b) = (vertA.id, vertB.id)
print((a == 2 and b == 5) or (a == 5 and b == 2))

(vertA, vertB) = Vertex.findClosestVertices(vertsA, vertsC)
(a, b) = (vertA.id, vertB.id)
print((a == 2 and b == 4) or (a == 4 and b == 2))

(vertA, vertB) = Vertex.findClosestVertices(vertsB, vertsC)
(a, b) = (vertA.id, vertB.id)
print((a == 9 and b == 8) or (a == 8 and b == 9))

# Test subgraph connecting
print("\n-----Testing Connecting Subgraphs-----")

disconnectedSubgraphVerts = [vertices[i] for i in [0, 1, 2, 3, 5, 7, 9, 10, 4, 8, 19, 14]]
disconnectedSubgraphEdges = [
  (0, 1),
  (0, 2),
  (0, 3),
  (4, 5),
  (4, 6),
  (4, 7),
  (8, 9),
  (8, 10),
  (8, 11)
]

disSubConnect = Graph.fill(disconnectedSubgraphVerts, disconnectedSubgraphEdges)
subgraphs = disSubConnect.findSubgraphs()
print(len(subgraphs) == 3)
disSubConnect.connectSubgraphs()
print(disSubConnect.hasEdge(disSubConnect.getVertex(2), disSubConnect.getVertex(4)))
print(disSubConnect.hasEdge(disSubConnect.getVertex(2), disSubConnect.getVertex(8)))
print(disSubConnect.hasEdge(disSubConnect.getVertex(6), disSubConnect.getVertex(9)))
