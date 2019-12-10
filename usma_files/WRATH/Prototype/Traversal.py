import heapq
import Problem
import Graph
import Edge

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

def constructEuclerianPath(graph):
  oddNodes = list()
  for vertex in graph.vertices:
    if (graph.getVertexDegree(vertex) % 2 == 1):
      oddNodes.append(vertex)
  
  distances = list()
  for i in range(len(oddNodes)):
    for j in range(i + 1, len(oddNodes)):
      edge = Edge.Edge(oddNodes[i], oddNodes[j])
      heapq.heappush(distances, edge)
  
  print("Num Odd: ", len(oddNodes))

  seen = set()
  while (len(seen) < len(oddNodes)):
    edge = heapq.heappop(distances)
    if (edge.start in seen or edge.end in seen):
      continue
    print("Adding edge: ", edge.start.id, edge.end.id)
    seen.add(edge.start)
    seen.add(edge.end)
    graph.addEdge(edge)
  return graph


def traverseEulerianPath(eulerianGraph, start):
  return list()