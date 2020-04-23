from PIL import Image 
import random
import ap_lib.gps_utils as gps
import math
import wrath_to_kml as kml

def inBounds(location):
    bottomLeft = (40.788171, -73.957488)
    topLeft = (40.800595, -73.959536)
    bottomRight = (40.788483, -73.953212)
    topRight = (40.801085, -73.956553)

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    v1 = (topLeft[0] - bottomLeft[0], topLeft[1] - bottomLeft[1])
    v2 = (topRight[0] - topLeft[0], topRight[1] - topLeft[1])
    v3 = (bottomRight[0] - topRight[0], bottomRight[1] - topRight[1])
    v4 = (bottomLeft[0] - bottomRight[0], bottomLeft[1] - bottomRight[1])

    a1 = (location[0] - bottomLeft[0], location[1] - bottomLeft[1])
    a2 = (location[0] - topLeft[0], location[1] - topLeft[1])
    a3 = (location[0] - topRight[0], location[1] - topRight[1])
    a4 = (location[0] - bottomRight[0], location[1] - bottomRight[1])
    
    return  det(a1, v1) < 0 and det(a2, v2) < 0 and det(a3, v3) < 0 and det(a4, v4) < 0


class RadModel():

    def __init__(self):
        self.A = (40.80004, -73.95810)
        self.B = (40.78904, -73.95531)
        self.Aintensity = 50
        self.Bintensity = 100
        Dir = (self.A[0] - self.B[0], self.A[1] - self.B[1])
        DirLen = math.sqrt(Dir[0] * Dir[0] + Dir[1] * Dir[1])
        self.NormDir = (Dir[0] / DirLen, Dir[1] / DirLen)
        self.abDistMeters = gps.gps_distance(self.A[0], self.A[1], self.B[0], self.B[1])
        self.maxDistOffset = 0.0004

    def getIntersection(self, location):
        v = (location[0] - self.B[0], location[1] - self.B[1])
        dist = v[0] * self.NormDir[0] + v[1] * self.NormDir[1]
        offset = (self.NormDir[0] * dist, self.NormDir[1] * dist)
        
        return (self.B[0] + offset[0], self.B[1] + offset[1])


    def getIntensity(self, intersection):
        pDistMeters = gps.gps_distance(intersection[0], intersection[1], self.A[0], self.A[1])
        return self.Aintensity + (self.Bintensity - self.Aintensity) * (pDistMeters / self.abDistMeters)


    def pollRadiation(self, location):
        randDist = random.uniform(0.0, self.maxDistOffset)
        randAngle = random.uniform(0.0, 360.0)

        xOffset = math.cos(math.radians(randAngle)) * randDist
        yOffset = math.sin(math.radians(randAngle)) * randDist

        location = (location[0] + yOffset, location[1] + xOffset)

        intersection = self.getIntersection(location)
        intensity = self.getIntensity(intersection)

        intersectionDistance = gps.gps_distance(intersection[0], intersection[1], location[0], location[1])

        return  intensity / (4 * math.pi * (intersectionDistance + 40))

radModel = RadModel()

overallMinX = 40.78887
overallMinY = -73.96360
overallMaxX = 40.80034
overallMaxY = -73.94969

coords = [(random.uniform(overallMinX, overallMaxX), random.uniform(overallMinY, overallMaxY)) for x in range(50000)]
coords = [coord for coord in coords if inBounds(coord)]
radPoints = [radModel.pollRadiation(x) for x in coords]

print max(radPoints)

origin = (40.79256, -73.96360)
topLeft = (40.80004, -73.95810)
bottomRight = (40.78904, -73.95531)
topRight = (40.796891, -73.949201)

KML = kml.WrathKML()

for coord, radPoint in zip(coords, radPoints):
    KML.addHeatMapPoint(coord, radPoint)

KML.addPoint((40.79256, -73.96360))

KML.save("test1")
