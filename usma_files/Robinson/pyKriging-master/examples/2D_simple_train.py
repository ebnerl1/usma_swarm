from __future__ import print_function
__author__ = 'cpaulson'
import pyKriging
from pyKriging.krige import kriging
from pyKriging.samplingplan import samplingplan

# The Kriging model starts by defining a sampling plan, we use an optimal Latin Hypercube here
sp = samplingplan(2)

print('sp=', sp)

X = sp.optimallhc(15)

print('X=', X)

# Next, we define the problem we would like to solve
testfun = pyKriging.testfunctions().branin

# We generate our observed values based on our sampling plan and the test function
y = testfun(X)
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

# #And plot the model

print('Now plotting final results...')
k.plot()
