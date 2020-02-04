#!/usr/bin/env python

from Collections import Vertex
from Collections import Graph
from Collections import RoutePlan
from Collections import Traversal

# These are Georgia Vertices and edges!
# vertices = [
#   (33.22619166666667, -81.58043611111111),
#   (33.22956388888889, -81.58343888888888),
#   (33.22983611111111, -81.58292777777777),
#   (33.22979722222222, -81.58360277777777),
#   (33.23055833333334, -81.58235833333333),
#   (33.22979166666667, -81.5816611111111),
#   (33.22959722222223, -81.58150277777777),
#   (33.23105, -81.58157222222222),
#   (33.23026388888889, -81.58091388888889),
#   (33.22989444444445, -81.58059166666666),
#   (33.22961666666667, -81.58099166666666),
#   (33.22894166666667, -81.58210555555556),
#   (33.22802777777778, -81.58156111111111),
#   (33.22921388888889, -81.58062777777778),
#   (33.22920833333333, -81.58038888888889),
#   (33.22881388888889, -81.58003611111111),
#   (33.22896666666667, -81.57977222222222),
#   (33.22853611111111, -81.57980555555555),
#   (33.22804166666667, -81.57991944444444),
#   (33.22839444444445, -81.57926666666667),
#   (33.22802777777778, -81.57751111111111)
# ]

# edges = [
#   (0, 1),
#   (1, 2),
#   (1, 3),
#   (3, 4),
#   (4, 5),
#   (5, 6),
#   (5, 8),
#   (4, 7),
#   (7, 8),
#   (8, 9),
#   (9, 10),
#   (10, 11),
#   (11, 12),
#   (10, 13),
#   (13, 14),
#   (14, 15),
#   (9, 16),
#   (15, 16),
#   (15, 17),
#   (17, 18),
#   (18, 19),
#   (16, 19),
#   (19, 20),
#   (0, 20)
# ]

# dronePositions = [
#   (33.22672778, -81.57888889),
#   (33.22688611, -81.57861111)
# ]

# These are RiverCourt vertices and edges!
vertices = [
  (41.39077, -73.95298),
  (41.39127, -73.95320),
  (41.39185, -73.95313),
  (41.39169, -73.95299),
  (41.39180, -73.95262),
  (41.39169, -73.95229),
  (41.39153, -73.95261),
  (41.39133, -73.95223),
  (41.39103, -73.95252)
]

edges = [
  (0, 1),
  (0, 8),
  (1, 2),
  (1, 3),
  (3, 4),
  (4, 5),
  (5, 6),
  (5, 7),
  (6, 7),
  (7, 8)
]

dronePositions = [
  (41.39073, -73.95333),
  (41.39070, -73.95322)
]

droneVertices = [Vertex.Vertex(pos) for pos in dronePositions]

roads = Graph.fill(vertices, edges, True)

graph = roads.copy()

graph.connectVertices(droneVertices)

print("Connected vertices")

assignment = RoutePlan.assignRoads(graph, roads, droneVertices)

print(assignment)

assignmentGraphs = [subgraph.toGraph() for subgraph in assignment]

print(assignmentGraphs)

paths = [RoutePlan.constructPath(graph) for graph in assignmentGraphs]

print([list(map(lambda v: v.id, lst)) for lst in paths ])
