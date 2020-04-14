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
    (40.79053, -73.95701),
    (40.79053, -73.95701),
    (40.79053, -73.95701),
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

# vertices = [
#     (40.78992, -73.95730), # P0
#     (40.78992, -73.95730), # P1
#     # (40.78982, -73.95710), # P2
#     # (40.78982, -73.95710), # P3
#     # (40.78982, -73.95710), # P4
#     # (40.79032, -73.95633), # P5
#     # (40.79050, -73.95556), # P6
#     # (40.79050, -73.95556), # P7
#     # (40.79077, -73.95539), # P8
#     # (40.79093, -73.95538), # P9
#     # (40.79084, -73.95453), # P10
#     # (40.79084, -73.95453), # P11
#     # (40.79097, -73.95405), # P12
#     # (40.79097, -73.95405), # P13
#     # (40.79097, -73.95405), # P14
#     # (40.79119, -73.95391), # P15
#     # (40.79146, -73.95407), # P16
#     # (40.79146, -73.95407), # P17
#     # (40.79182, -73.95377), # P18
#     # (40.79182, -73.95377), # P19
#     # (40.79204, -73.95363), # P20
#     # (40.79221, -73.95381), # P21
#     # (40.79221, -73.95381), # P22
#     # (40.79221, -73.95381), # P23
#     # (40.79269, -73.95321), # P24
#     # (40.79275, -73.95319), # P25
#     # (40.79275, -73.95319), # P26
#     # (40.79275, -73.95319), # P27
#     # (40.79275, -73.95319), # P28
#     # (40.79275, -73.95319), # P29
#     # (40.79310, -73.95256), # P30
#     # (40.79325, -73.95249), # P31
#     # (40.79345, -73.95211), # P32
#     # (40.79343, -73.95212), # P33
#     # (40.79360, -73.95200), # P34
#     # (40.79360, -73.95200), # P35
#     # (40.79416, -73.95146), # P36
#     # (40.79434, -73.95191), # P37
#     # (40.79434, -73.95191), # P38
#     # (40.79461, -73.95130), # P39
#     # (40.79488, -73.95075), # P40
#     # (40.79488, -73.95075), # P41
#     # (40.79531, -73.95047), # P42
#     # (40.79597, -73.95002), # P43
#     # (40.79644, -73.94971), # P44
#     # (40.79644, -73.94971), # P45
#     # (40.79668, -73.95019), # P46
#     # (40.79705, -73.95100), # P47
#     # (40.79742, -73.95226), # P48
#     # (40.79767, -73.95247), # P49
#     # (40.79765, -73.95302), # P50
#     # (40.79765, -73.95302), # P51
#     # (40.79777, -73.95363), # P52
#     # (40.79801, -73.95359), # P53
#     (40.79848, -73.95445), # P54
#     (40.79900, -73.95519), # P55
#     (40.79886, -73.95612), # P56
#     (40.79886, -73.95612), # P57
#     (40.79847, -73.95720), # P58
#     (40.79847, -73.95720), # P59
#     (40.79842, -73.95789), # P60
#     (40.79827, -73.95735), # P61
#     (40.79755, -73.95708), # P62
#     (40.79652, -73.95701), # P63
#     (40.79652, -73.95701), # P64
#     (40.79544, -73.95750), # P65
#     (40.79439, -73.95890), # P66
#     (40.79439, -73.95890), # P67
#     (40.79414, -73.95921), # P68
#     (40.79372, -73.96064), # P69
#     (40.79310, -73.96115), # P70
#     (40.79310, -73.96025), # P71
#     (40.79228, -73.96183), # P72
#     (40.79094, -73.96041), # P73
#     (40.79160, -73.96142), # P74
#     (40.79160, -73.96142), # P75
#     (40.79139, -73.96082), # P76
#     (40.79139, -73.96082), # P77
#     # (40.79180, -73.96122), # P78
#     # (40.79180, -73.96122), # P79
#     # (40.79218, -73.96149), # P80
#     (40.79095, -73.96033), # P81
#     (40.79077, -73.95980), # P82
#     (40.79077, -73.95980), # P83
#     (40.79074, -73.95886), # P84
#     (40.79034, -73.95716), # P85
#     (40.78992, -73.95730), # P86
#     # (40.79025, -73.95672), # P87
#     # (40.79095, -73.95631), # P88
#     # (40.79143, -73.95597), # P89
#     # (40.79209, -73.95532), # P90
#     # (40.79276, -73.95524), # P91
#     # (40.79322, -73.95662), # P92
#     # (40.79301, -73.95664), # P93
#     # (40.79298, -73.95713), # P94
#     # (40.79272, -73.95765), # P95
#     # (40.79272, -73.95798), # P96
#     # (40.79272, -73.95798), # P97
#     # (40.79126, -73.95789), # P98
#     # (40.79327, -73.95928), # P99
#     # (40.79363, -73.96008), # P100
#     # (40.79296, -73.96037), # P101
#     # (40.79296, -73.96037), # P102
#     (40.79121, -73.95600), # P103
#     (40.79148, -73.95590), # P104
#     # (40.79158, -73.95499), # P105
#     (40.79158, -73.95499), # P106
#     (40.79174, -73.95430), # P107
#     (40.79183, -73.95433), # P108
#     (40.79204, -73.95497), # P109
#     (40.79241, -73.95467), # P110
#     # (40.79245, -73.95456), # P111
#     (40.79300, -73.95465), # P112
#     # (40.79300, -73.95465), # P113
#     # (40.79300, -73.95465), # P114
#     # (40.79328, -73.95462), # P115
#     # (40.79342, -73.95445), # P116
#     # (40.79342, -73.95445), # P117
#     # (40.79342, -73.95445), # P118
#     (40.79342, -73.95445), # P119
#     # (40.79384, -73.95432), # P120
#     (40.79426, -73.95527), # P121
#     (40.79444, -73.95502), # P122
#     (40.79369, -73.95765), # P123
#     (40.79359, -73.95920), # P124
#     (40.79429, -73.95396), # P125
#     (40.79478, -73.95319), # P126
#     (40.79496, -73.95318), # P127
#     (40.79514, -73.95373), # P128
#     (40.79514, -73.95373), # P129
#     (40.79545, -73.95408), # P130
#     (40.79569, -73.95448), # P131
#     (40.79569, -73.95448), # P132
#     (40.79655, -73.95372), # P133
#     (40.79709, -73.95360), # P134
#     (40.79709, -73.95360), # P135
#     # (40.79687, -73.95344), # P136
#     # (40.79687, -73.95344), # P137
#     # (40.79687, -73.95344), # P138
#     # (40.79641, -73.95333), # P139
#     # (40.79641, -73.95333), # P140
#     # (40.79641, -73.95333), # P141
#     # (40.79641, -73.95333), # P142
#     # (40.79608, -73.95288), # P143
#     # (40.79608, -73.95288), # P144
#     # (40.79608, -73.95288), # P145
#     # (40.79569, -73.95270), # P146
#     # (40.79579, -73.95231), # P147
#     # (40.79579, -73.95231), # P148
#     # (40.79579, -73.95231), # P149
#     # (40.79579, -73.95231), # P150
#     # (40.79579, -73.95231), # P151
#     # (40.79579, -73.95231), # P152
#     # (40.79501, -73.95219), # P153
#     # (40.79501, -73.95219) # P154
#     # (40.79790, -73.95853), # P155
#     # (40.79790, -73.95853), # P156
#     # (40.79790, -73.95853), # P157
#     # (40.79790, -73.95853), # P158
#     # (40.79733, -73.95851), # P159
#     # (40.79733, -73.95851), # P160
#     # (40.79675, -73.95863), # P161
#     # (40.79675, -73.95863), # P162
#     # (40.79666, -73.95906), # P163
#     # (40.79677, -73.95738), # P164
#     # (40.79690, -73.95743), # P165
#     # (40.79640, -73.95830), # P166
#     # (40.79676, -73.95893), # P167
#     # (40.79631, -73.95946), # P168
#     # (40.79583, -73.95832), # P169
#     # (40.79583, -73.95832), # P170
#     # (40.79566, -73.95820), # P171
#     # (40.79566, -73.95820), # P172
#     # (40.79566, -73.95820), # P173
#     # (40.79566, -73.95820) # P174
# ]


def planRoute():
    droneVertices = [Vertex.Vertex(pos) for pos in droneLocations]

    roads = Graph.fill(vertices, edges, True)

    graph = roads.copy()

    graph.connectVertices(droneVertices)

    assignment = RoutePlan.assignRoads(graph, roads, droneVertices, 1, 51000)

    assignmentGraphs = [subgraph.toGraph(start.id) for (subgraph, start) in zip(assignment, droneVertices)]

    for i in range(len(assignmentGraphs)):
        aGraph = assignmentGraphs[i]
        kml.generate()
        kml.addGraph(aGraph)
        kml.save("testGraph" + str(i))

    paths = [RoutePlan.constructPath(graph, graph.start) for graph in assignmentGraphs]

    for path in paths:
        print "num:", len(path)

    # for i in range(len(paths)):
    #     print "Path: ", i
    #     for v in paths[i]:
    #         print v.id, v.coord

    for i in range(len(paths)):
        pathLen = 0
        path = paths[i]
        for ii in range(1, len(path)):
            print "dist:", Problem.calculateDistance(path[ii-1], path[ii])
            if Problem.calculateDistance(path[ii-1], path[ii]) < 1:
                print ii, path[ii].id, path[ii-1].id
            pathLen += Problem.calculateDistance(path[ii-1], path[ii])
        print i, ":", pathLen


if __name__ == '__main__':
    planRoute()