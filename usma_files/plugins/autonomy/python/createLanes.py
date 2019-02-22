#from box import Box

#b = Box([41.391207, -73.953357],100,50)
#import new_rivercourt_enumerations as usma_enums
def createLanes():
	nw = [41.391380, -73.953237]
	ne = [41.391358, -73.952564]
	sw = [41.390779, -73.953283]
	lanes = [] #this will be a list of tuples
	lane_height = nw[0]-sw[0]
	lane_width = (ne[1]-nw[1])/10 #not sure if grids increase left to right #or vice versa
	start = nw
	point = [0,0]
	counter = 0
	while start[1] < ne[1]:
		point = start
		if counter%2 ==1:
			newLane = [[point[0]-lane_height, point[1]],[point[0],point[1]]]
			lanes.append(newLane)
		else:
			newLane = [[point[0],point[1]],[point[0]-lane_height,point[1]]]
			lanes.append(newLane)
		start[1] += lane_width
		counter += 1
	return lanes


