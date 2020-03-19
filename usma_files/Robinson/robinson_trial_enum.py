#!/usr/bin/env python


#Kriging Imports
from __future__ import print_function
__author__ = 'cpaulson'
import pyKriging
from pyKriging.krige import kriging
from pyKriging.samplingplan import samplingplan

#Kriging imports complete
import math

#Extra import for column stuff
import numpy as np
#Extra import for column stuff complete
WP_LOC = []

#How far apart desired waypoints are in meters
mesh = 5

#For creating desired number of waypoints
wp = 5

min_latitude =   41
#Degrees latitde per meter (do not alter)
lat_dis = 0.000009005401
#Using 19 for the purpose of this trial, actual max value can be used for real test
max_latitude = min_latitude+wp*mesh*lat_dis
if abs((max_latitude-min_latitude)/(lat_dis*mesh)) % 1 < 0.5:
    lat_points = abs(int((max_latitude-min_latitude)/(lat_dis*mesh)))
else:
    lat_points = 1+abs(int((max_latitude-min_latitude)/(lat_dis*mesh)))
print("Latitude points: " + repr(lat_points))
min_longitude = -74
#Degrees longitude per meter (do not alter)
long_dis = 1/(math.cos(min_latitude*180/math.pi)*69.172*1609.34)
max_longitude = -74+wp*mesh*long_dis
if abs((max_longitude-min_longitude)/(long_dis*mesh)) % 1 < 0.5:
    long_points = abs(int((max_longitude-min_longitude)/(long_dis*mesh)))
else:
    long_points = 1+abs(int((max_longitude-min_longitude)/(long_dis*mesh)))
print("Longitude points: " + repr(long_points))

#Default value set for waypoins
default_value = 10

for i in range(0,lat_points):
    for j in range(0,long_points):
        n = i*long_points+j
        waypoint = []
        waypoint.append(min_latitude+lat_dis*i*mesh)
        waypoint.append(min_longitude+long_dis*j*mesh)
        waypoint.append(default_value)
        waypoint.append(n)
        WP_LOC.append(waypoint)
        print("Waypoint: " + repr(WP_LOC[n]))
print("Total number of waypoints generated: " + repr(len(WP_LOC)))
print(WP_LOC)
hotspot_value = 40
hotspot_lat = 2
hotspot_long = 3
hotspot_wp = hotspot_lat*long_points+hotspot_long
print("hotspot_wp=", hotspot_wp)
n = 1
decrease = 2
WP_LOC[hotspot_wp][2] = hotspot_value
while hotspot_value/decrease**n > default_value:
    for i in range(-n,n+1):
        for j in range(-n,n+1):
            wp_update = int(hotspot_wp+long_points*i+j)
            print("wp_update=", wp_update)
            if WP_LOC[wp_update][2] < int(hotspot_value/decrease**n):
                WP_LOC[wp_update][2] = int(hotspot_value/decrease**n)
    n = n+1
for i in range(0,len(WP_LOC)-1):
    print(WP_LOC[i][2])

x=[item[0] for item in WP_LOC]
x_norm = (x - min_latitude)/(max_latitude - min_latitude)
array=np.array([x])
x2=(array-min_latitude)/(max_latitude-min_latitude)
print("x2=",x2)
    
Y=[item[1] for item in WP_LOC]


print('x=',x)
print('Y=',Y)
X=np.column_stack((x,Y))
print('x=',X)


#Kriging code now

#Next, we define the problem we would like to solve
testfun = pyKriging.testfunctions().branin
#testfun2 = pyKriging.krige().branin

# We generate our observed values based on our sampling plan and the test function

#y = testfun(X)
y=[item[2] for item in WP_LOC]
print('y=', y)
print('Setting up the Kriging Model')

# Now that we have our initial data, we can create an instance of a kriging model
k = kriging(X, y, testfunction=testfun, name='simple', testPoints=250)
k.train(optimizer='ga')
k.snapshot()
print('k=', k)
for i in range(5):
    newpoints = k.infill(2)
    print('i', i)
    print('newpoints', newpoints)
    print('k.infill', k.infill)
    for point in newpoints:
        print(('Adding point {}'.format(point)))
        k.addPoint(point, testfun(point)[0])
        print('point=', point)
        print('testfun(point)[0]=', testfun(point)[0])
        print('testfun(point)=', testfun(point))
    k.train()
    k.snapshot()

 #And plot the model

print('Now plotting final results...')
k.plot()
