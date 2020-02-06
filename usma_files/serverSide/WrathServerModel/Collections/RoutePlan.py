#!/usr/bin/env python

from Collections import Problem
from Collections import Traversal
import heapq

def planRoute(dronePositions, roads):
  graph = roads.copy()
  graph.connectSubgraphs()
  graph.connectVertices(dronePositions)

  droneAssignment = assignRoads(graph, roads, len(dronePositions))
  dronePaths = [constructPath(edges, start) for (edges, start) in zip(droneAssignment, dronePositions)]


# returns a list of graphs, one for each drone
def assignRoads(graph, roads, droneStartPositions, distWeight, roadWeight):
  problem = Problem.Problem(graph, roads, droneStartPositions, distWeight, roadWeight)
  return Traversal.uniformCostSearch(problem)


# create euclerian path with graph, return list of vertices
def constructPath(graph, startVertex):
  newGraph = Traversal.constructEulerianPath(graph, startVertex)
  return Traversal.traverseEulerianPath(newGraph, startVertex)
