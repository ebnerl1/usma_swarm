#!/usr/bin/env python

# This script writes points to a kml file to view in GoogleEarth
# To work, must install simplekml python module. In terminal run 2 commands:
# 1: `curl https://raw.github.com/pypa/pip/master/contrib/get-pip.py | sudo python`
# 2: `pip install simplekml`

# Andrew Kopeikin
# 3 March 2018

import csv
import simplekml

kml_name = 'range11enums.kml'

WP_LOC = dict()
WP_LOC[0] = ( 41.359354, -74.032467 )
WP_LOC[1] = ( 41.358756, -74.032392 )
WP_LOC[2] = ( 41.358159, -74.032318 )
WP_LOC[3] = ( 41.357562, -74.032243 )
WP_LOC[4] = ( 41.356965, -74.032169 )
WP_LOC[5] = ( 41.359303, -74.032019 )
WP_LOC[6] = ( 41.358687, -74.031973 )
WP_LOC[7] = ( 41.358072, -74.031927 )
WP_LOC[8] = ( 41.357457, -74.031882 )
WP_LOC[9] = ( 41.356842, -74.031836 )
WP_LOC[10] = ( 41.359339, -74.031713 )
WP_LOC[11] = ( 41.358708, -74.031665 )
WP_LOC[12] = ( 41.358078, -74.031618 ) 
WP_LOC[13] = ( 41.357447, -74.031571 ) 
WP_LOC[14] = ( 41.356817, -74.031524 )
WP_LOC[15] = ( 41.359350, -74.031328 )
WP_LOC[16] = ( 41.358727, -74.031279 )
WP_LOC[17] = ( 41.358105, -74.031230 )
WP_LOC[18] = ( 41.357482, -74.031181 )
WP_LOC[19] = ( 41.356860, -74.031132 )
WP_LOC[20] = ( 41.359365, -74.031067 )
WP_LOC[21] = ( 41.358732, -74.031027 )
WP_LOC[22] = ( 41.358099, -74.030987 )
WP_LOC[23] = ( 41.357466, -74.030947 )
WP_LOC[24] = ( 41.356833, -74.030907 )

kml=simplekml.Kml()

for i in range(0, len(WP_LOC)):
    kml.newpoint(name=str(i), coords=[(WP_LOC[i][1],WP_LOC[i][0])])

kml.save(kml_name)

