import Graph
import Subgraph
import Edge

# Node is a Tuple of Subgraphs
class Problem:
  def __init__(self, graph, roads, startVertices):
    self.graph = graph
    self.roads = roads
    self.distanceWeight = 1
    self.roadWeight = 20000
    self.numRoads = roads.nEdges
    
    node = list()
    for start in startVertices:
      droneGraph = Subgraph.Subgraph(self.graph)
      droneGraph.addVertex(start)
      node.append(droneGraph)

    self.initialState = tuple(node)
  
  def isGoal(self, node):
    nodeEdges = set()
    for droneGraph in node:
      for edge in droneGraph.getEdgeIndices():
        nodeEdges.add(edge)

    nRoads = 0
    for edge in list(nodeEdges):
      if (self.graph.getEdge(edge[0], edge[1]).isRoad):
        nRoads += 1

    if (nRoads > self.numRoads):
      raise IndexError("Something weird happened")

    return self.numRoads == nRoads

  # node is a list of graphs
  # graphs are drone edge assignments
  def getNeighbors(self, node):
    neighbors = list()
    nodelst = list(node)

    for i in range(len(node)):
      droneNeighbors = node[i].getNeighbors()
      for j in range(len(droneNeighbors)): 
        neighbor = nodelst.copy()
        neighbor[i] = droneNeighbors[j]
        neighbors.append(tuple(neighbor))
    return neighbors

  def getCost(self, node):
    numRoads = 0

    nodeEdges = set()
    for droneGraph in node:
      for edge in droneGraph.getEdgeIndices():
        nodeEdges.add(edge)

    # make this easier to calculate
    for edge in self.roads.getEdges():
      (lo, hi) = (edge.start.id, edge.end.id) if (edge.start.id < edge.end.id) else (edge.end.id, edge.start.id)
      if ((lo, hi) in nodeEdges):
        numRoads += 1
    
    distances = [0 for i in node]
    for i in range(len(node)):
      for edge in node[i].getEdges():
        distances[i] += edge.distance
    
    return (1 / (numRoads + 1)) * self.roadWeight + (max(distances)) * 1000 * self.distanceWeight

  def getSetRepr(self, node):
    return tuple([subgraph.getSetRepr() for subgraph in node])

