#!/usr/bin/python

import ap_lib.gps_utils as gps

def getDist(p1, p2):
    return gps.gps_distance(p1[0], p1[1], p2[0], p2[1])

class LineSegment(object):

    def __init__(self, p1, p2):
        self.items = (p1, p2)
    
    def __getitem__(self, key):
        return self.items[key]

    def getSlope(self):
        return (self[1][0] - self[0][0], self[1][1] - self[0][1])

    def getNormal(self):
        slope = self.getSlope()
        return (slope[1], -slope[0])

    def findIntersection(self, otherSegment):
        xdiff = (self[0][0] - self[1][0], otherSegment[0][0] - otherSegment[1][0])
        ydiff = (self[0][1] - self[1][1], otherSegment[0][1] - otherSegment[1][1])

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)
        if div == 0:
            return None

        d = (det(*self), det(*otherSegment))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        return x, y

    def pointBetweenLine(self, point):
        return getDist(self[0], point) + getDist(self[1], point) < getDist(self[0], self[1]) + .0001
