from Collections import Vertex
from Collections import Graph
from Collections import RoutePlan
from Collections import Traversal
from Collections import Problem

import wrath_to_kml as kml

import ap_lib.gps_utils as gps

def calculateDistance(p0, p1):
    return gps.gps_distance(p0.coord[0], p0.coord[1], p1.coord[0], p1.coord[1])

droneLocations = [
    (40.79053, -73.95701)
]

edges = [
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 4),
    (4, 5),
    (5, 6),
    (6, 7),
    (7, 8),
    (8, 9),
    (9, 10),
    (10, 11),
    (11, 12),
    (12, 13),
    (13, 14),
    (14, 15),
    (15, 16),
    (16, 17),
    (17, 18),
    (18, 19),
    (19, 20),
    (20, 21),
    (21, 22),
    (22, 23),
    (23, 24),
    (24, 25),
    (25, 26),
    (26, 27),
    (27, 28),
    (28, 0),
    (3, 29),
    (29, 30),
    (21, 30)
]

vertices = [
    (40.78974, -73.95738), # P0 - 0
    (40.79173, -73.95559), # P104 - 1
    (40.79264, -73.95500), # P110 - 2
    (40.79435, -73.95482), # P121 - 3
    (40.79493, -73.95426), # P125 - 4
    (40.79526, -73.95353), # P126 - 5
    (40.79553, -73.95334), # P127 - 6
    (40.79595, -73.95365), # P128 - 7
    (40.79591, -73.95439), # P129 - 8
    (40.79552, -73.95499), # P130 - 9
    (40.79565, -73.95558), # P131 - 10
    (40.79623, -73.95566), # P132 - 11
    (40.79769, -73.95437), # P134 - 12
    (40.79842, -73.95443), # P54 - 13
    (40.79917, -73.95568), # P55 - 14
    (40.79958, -73.95787), # P57 - 15
    (40.79876, -73.95856), # P59 - 16
    (40.79837, -73.95820), # P60 - 17
    (40.79780, -73.95720), # P61 - 18
    (40.79612, -73.95697), # P64 - 19
    (40.79522, -73.95881), # P66 - 20
    (40.79441, -73.95934), # P67 - 21
    (40.79406, -73.95995), # P68 - 22
    (40.79369, -73.96108), # P69 - 23
    (40.79178, -73.96246), # P73 - 24
    (40.79161, -73.96147), # P75 - 25
    (40.79123, -73.96046), # P81 - 26
    (40.79051, -73.95916), # P83 - 27
    (40.79022, -73.95755), # P85 - 28
    (40.79447, -73.95576), # P122 - 29
    (40.79364, -73.95810) # P124 - 30
]


def planRoute():
    droneVertices = [Vertex.Vertex(pos) for pos in droneLocations]

    roads = Graph.fill(vertices, edges, True)

    graph = roads.copy()

    graph.connectVertices(droneVertices)

    assignment = RoutePlan.assignRoads(graph, roads, droneVertices, 1, 180000)

    assignmentGraphs = [subgraph.toGraph(start.id) for (subgraph, start) in zip(assignment, droneVertices)]

    for i in range(len(assignmentGraphs)):
        aGraph = assignmentGraphs[i]
        kml.generate()
        kml.addGraph(aGraph)
        kml.save("testGraph" + str(i))

    paths = [RoutePlan.constructPath(graph, graph.start) for graph in assignmentGraphs]

    for path in paths:
        print "num:", len(path)

    maxPathLen = 0
    for i in range(len(paths)):
        pathLen = 0
        path = paths[i]
        for ii in range(1, len(path)):
            # print "dist:", Problem.calculateDistance(path[ii-1], path[ii])
            # if Problem.calculateDistance(path[ii-1], path[ii]) < 1:
                # print ii, path[ii].id, path[ii-1].id
            pathLen += Problem.calculateDistance(path[ii-1], path[ii])
        if (pathLen > maxPathLen):
            maxPathLen = pathLen
        print i, ":", pathLen
    print "Max: ", maxPathLen
    
if __name__ == '__main__':
    planRoute()