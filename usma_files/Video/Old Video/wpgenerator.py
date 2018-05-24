import math

def wpGenerator(starting, ending, numOfPoints):
  iterableWP = [starting]
  incLon = (ending[0] - starting[0])/(numOfPoints-1)
  incLat = (ending[0] - starting[1])/(numOfPoints-1)
  numRow = int(math.sqrt(numOfPoints))
  storedPt = (starting[0]+incLon, starting[1]+incLat)
  for col in range(1,numRow):
    for row in range(1,numRow):
      iterableWP.append(storedPt)
      storedPt = (storedPt[0], storedPt[1]+incLat)
    storedPt = (storedPt[0]+incLon ,storedPt[1])
  return iterableWP

first = (1,1)
second = (5,5)
number = 25

print(wpGenerator(first,second,number))

