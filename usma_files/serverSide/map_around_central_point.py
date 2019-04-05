import math

WP_List = []

new_lat = 99
new_long = 99

def grab_hotspot(latitude,longitude):
    new_lat = latitude
    new_long = longitude
    print "the hotspot center was updated"
    return new_lat, new_long

def createGrid(lat,lon):

    #How far apart desired waypoints are in meters
    mesh = 5

    #Number of desired waypoints per side
    n = 10

    #Latitude of center point
    central_latitude = lat #this will be a variable from initial_pass

    #Degrees latitde per meter (do not alter)
    lat_dis = 0.000009005401

    #Longitude of center point
    central_longitude = lon #this will be a variable from initial_pass

    #Degrees longitude per meter (do not alter)
    long_dis = 1/(math.cos(central_latitude*180/math.pi)*69.172*1609.34)


    if n % 2 == 1:
        for i in range(-n/2+1,n/2+1):
            for j in range(-n/2+1,n/2+1):
                waypoint = []
                latitude = central_latitude+j*mesh*lat_dis
                longitude = central_longitude+i*mesh*long_dis
                waypoint.append(latitude)
                waypoint.append(longitude)
                WP_List.append(waypoint)
    else:
        for i in range(-n/2,n/2):
            for j in range(-n/2,n/2):
                waypoint = []
                latitude = central_latitude+(j+0.5)*mesh*lat_dis
                longitude = central_longitude+(i+0.5)*mesh*long_dis
                waypoint.append(latitude)
                waypoint.append(longitude)
                WP_List.append(waypoint)

    #f = open("hotspot_grid.py","w")
    index = 0
    #f.write("WP_LOC_Range11 = dict()\n")
    #for point in WP_List:
        #f.write("WP_LOC_Range11[" + str(index) + "] = " + "(" + str(point[0]) + ", " + str(point[1]) + ")" +"\n")
        #index += 1
    #f.close() 
    with open('~/scrimmage/usma/plugins/autonomy/python/hotspot_grid.py', 'w') as output:
        output.write("WP_LOC_Range11[" + str(index) + "] = " + "(" + str(point[0]) + ", " + str(point[1]) + ")" +"\n")
        for point in WP_List:
            output.write("WP_LOC_Range11[" + str(index) + "] = " + "(" + str(point[0]) + ", " + str(point[1]) + ")" +"\n")
            index += 1
    print "hotspot grid created"

