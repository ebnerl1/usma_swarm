import Graph
import Problem
import Traversal
import heapq

def planRoute(dronePositions, roads):
  graph = roads.copy()
  graph.connectSubgraphs()
  graph.connectVertices(dronePositions)

  droneAssignment = assignRoads(graph, roads, len(dronePositions))
  dronePaths = [constructPath(edges, start) for (edges, start) in zip(droneAssignment, dronePositions)]


# returns a list of graphs, one for each drone
def assignRoads(graph, roads, droneStartPositions):
  problem = Problem.Problem(graph, roads, droneStartPositions)
  return Traversal.uniformCostSearch(problem)


# create euclerian path with graph, return list of vertices
def constructPath(graph, start):
  newGraph = constructEuclerianPath(graph)
  return traversePath(newGraph, start)
