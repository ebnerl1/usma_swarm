from __future__ import print_function
__author__ = 'cpaulson'
import pyKriging
from pyKriging.krige import kriging
from pyKriging.samplingplan import samplingplan
import numpy as np
from random import random
from random import seed
seed(1)
import math
# The Kriging model starts by defining a sampling plan, we use an optimal Latin Hypercube here
#sp = samplingplan(2)

#print('sp=', sp)

#X = sp.optimallhc(5)







#LatLong stuff

min_latitude =   33.224849
#Degrees latitde per meter (do not alter)
lat_dis = 0.000009005401
min_latitude =   33.224849
max_latitude = 33.226819
min_longitude = -81.579296
max_longitude = -81.577394
#lx=range (min_latitude, max_latitude[, lat_dis])
#ly=range (min_longitude, max_longitude[, lat_dis])
#nx=(((x-min_latitude)*(1-0))/(max_latitude-min_latitude))+0
#ny=(((x-min_longitude)*(1-0))/(max_longitude-min_longitude))+0
#for i in range(x):
#    nx=(((x-min_latitude)*(1-0))/(max_latitude-min_latitude))+0
#    ny=(((x-min_longitude)*(1-0))/(max_longitude-min_longitude))+0

    






#Lat long end
#print('X=', X)
z=.1
zs2=20*lat_dis
s=30.0
hotspots=1
xs=.2
ys=.2
HotLoc=[]
for i in range(hotspots):
    Loc=[]
    xs2=(((xs-0)*(max_latitude-min_latitude))/(1-0))+min_latitude
    ys2=(((ys-0)*(max_longitude-min_longitude))/(1-0))+min_longitude
    Loc.append(xs2)
    Loc.append(ys2)
    HotLoc.append(Loc)
print("HotLoc=",HotLoc)


Fieldf=[]
print("field=",Fieldf)
for i in range(5):
    x=random()
    y=random()
    x2=(((x-0)*(max_latitude-min_latitude))/(1-0))+min_latitude
    y2=(((y-0)*(max_longitude-min_longitude))/(1-0))+min_longitude
    r=math.sqrt((xs2-x2)**2+(ys2-y2)**2+zs2**2)
    r=r/lat_dis
    print("r=",r)
    Y=s/(r**2)
    Field=[]
    Field.append(x)
    Field.append(y)
    Field.append(Y)
    Fieldf.append(Field)
    print("FieldF=",Field)
    print("x=",x)
    print("y=",y)
    print("r=",r)
    print("Y=",Y)
    #print("bob",bob)
    i=i+1
print("FieldF=",Fieldf)

x=[item[0] for item in Fieldf]
#x_norm = (x - min_latitude)/(max_latitude - min_latitude)
array=np.array([x])
#x2=(array-min_latitude)/(max_latitude-min_latitude)
#print("x2=",x2)
    
y=[item[1] for item in Fieldf]


print('x=',x)
print('Y=',y)
X=np.column_stack((x,y))
print('x=',X)
# Next, we define the problem we would like to solve
testfun = pyKriging.testfunctions().branin

# We generate our observed values based on our sampling plan and the test function
#y = testfun(X)
y=[item[2] for item in Fieldf]


print('y=', y)
print('Setting up the Kriging Model')

#Now that we have our initial data, we can create an instance of a kriging model
k = kriging(X, y, testfunction=None, name='', testPoints=10)
k.train(optimizer='ga')
k.snapshot()
print('k=', k)
for i in range(30):
    newpoints = k.infill(1)
    print('i', i)
    print('newpoints', newpoints)
    print('k.infill', k.infill)
    for point in newpoints:
        print(('Adding point {}'.format(point)))
        x1=point[0]
        y1=point[1]
        x2=(((x1-0)*(max_latitude-min_latitude))/(1-0))+min_latitude
        y2=(((y1-0)*(max_longitude-min_longitude))/(1-0))+min_longitude
        Ynew2=0
        for j in range(hotspots):
            xlat=[item[0] for item in HotLoc]
            ylong=[item[1] for item in HotLoc]
            r=math.sqrt((xlat-x2)**2+(ylong-y2)**2+zs2**2)
            r=r/lat_dis
            print("r=",r)
            Ynew1=s/(r**2)
            Ynew=Ynew1+Ynew2
            Ynew2=Ynew
        k.addPoint(point, Ynew)
        print('point=', point)
        print('testfun(point)[0]=', Ynew)
        #print('testfun(point)=', testfun(point))
    k.train()
    k.snapshot()

# #And plot the model

print('Now plotting final results...')
k.plot()
