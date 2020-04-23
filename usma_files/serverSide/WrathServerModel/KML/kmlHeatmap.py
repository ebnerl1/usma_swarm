import heatmap
import random
import ap_lib.gps_utils as gps
import math

A = (40.80004, -73.95810)
B = (40.78904, -73.95531)
Aintensity = 30
Bintensity = 100
Dir = (A[0] - B[0], A[1] - B[1])
DirLen = math.sqrt(Dir[0] * Dir[0] + Dir[1] * Dir[1])
NormDir = (Dir[0] / DirLen, Dir[1] / DirLen)
abDistMeters = gps.gps_distance(A[0], A[1], B[0], B[1])


def getIntersection(location):
    v = (location[0] - B[0], location[1] - B[1])
    dist = v[0] * NormDir[0] + v[1] * NormDir[1]
    offset = (NormDir[0] * dist, NormDir[1] * dist)
    
    return (B[0] + offset[0], B[1] + offset[1])


def getIntensity(intersection):
    pDistMeters = gps.gps_distance(intersection[0], intersection[1], A[0], A[1])
    return Aintensity + (Bintensity - Aintensity) * (pDistMeters / abDistMeters) 


def pollRadiation(location):
    intersection = getIntersection(location)
    intensity = getIntensity(intersection)    

    intersectionDistance = gps.gps_distance(intersection[0], intersection[1], location[0], location[1])

    rad = intensity / (4 * math.pi * (intersectionDistance * intersectionDistance + 40 * 40))
    return rad

hm = heatmap.Heatmap()

coords = [(random.uniform(40.78887, 40.80034), random.uniform(-73.96360, -73.94969)) for x in range(10000)]
radPoints = [pollRadiation(x) for x in coords]

print max(radPoints)
print min(radPoints)

pts = list()
for coord, radPoint in zip(coords, radPoints):
    num = 1 + int(radPoint * 2000)

    for i in range(num):
        pts.append((coord[1], coord[0]))

hm.heatmap(pts, 5)
hm.saveKML("data.kml")
