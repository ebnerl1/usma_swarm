#!/usr/bin/env python

import heapq
from Collections import Problem
from Collections import Graph
from Collections import Edge

def uniformCostSearch(problem):
  node = problem.initialState
  weight = problem.getCost(node)

  explored = set()
  frontier = list()
  heapq.heappush(frontier, (weight, node))

  while (len(frontier) > 0):
    node = heapq.heappop(frontier)

    nodeSetRepr = problem.getSetRepr(node[1])
    if (nodeSetRepr in explored):
      continue
    
    if (problem.isGoal(node[1])):
      return node[1]

    explored.add(nodeSetRepr)
    for neighbor in problem.getNeighbors(node[1]):
      if (problem.getSetRepr(neighbor) not in explored):
        childNode = (problem.getCost(neighbor), neighbor)
        heapq.heappush(frontier, childNode)

# construct a path with 2 odd vertices, the start and end
def constructEulerianPath(graph, start):
  oddNodes = list()
  for vertex in graph.vertices:
    if (graph.getVertexDegree(vertex) % 2 == 1 and vertex != start):
      oddNodes.append(vertex)
  
  distances = list()
  for i in range(len(oddNodes)):
    for j in range(i + 1, len(oddNodes)):
      edge = Edge.Edge(oddNodes[i], oddNodes[j])
      heapq.heappush(distances, edge)
  
  print("Num Odd: " + str(len(oddNodes)))

  seen = set()
  while (len(seen) < len(oddNodes) - 1):
    edge = heapq.heappop(distances)
    if (edge.start in seen or edge.end in seen):
      continue
    print("Adding edge: " + str((edge.start.id, edge.end.id)))
    seen.add(edge.start)
    seen.add(edge.end)
    graph.addEdge(edge)
  return graph


# Fleury's Algorithm
def traverseEulerianPath(eulerianGraph, start):
  path = [start]
  curVertex = start

  iterations = 0
  while (eulerianGraph.nEdges > 0 and iterations < 15):
    iterations += 1
    for (newVertex, edge) in eulerianGraph.getNeighbors(curVertex):
      eulerianGraph.removeEdge(edge)
      numSubgraphs = len(list(filter(lambda lst: len(lst) > 1, eulerianGraph.findSubgraphs())))
      if (numSubgraphs < 2 and (eulerianGraph.getVertexDegree(newVertex) > 0 or numSubgraphs == 0)):
        path.append(newVertex)
        curVertex = newVertex
        break
      else:
        eulerianGraph.addEdge(edge)
  return path
