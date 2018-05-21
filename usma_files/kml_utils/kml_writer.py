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
WP_LOC[0] = ( 41.391865, -73.952959 )
WP_LOC[1] = ( 41.391829, -73.952747 )
WP_LOC[2] = ( 41.391774, -73.952565 )
WP_LOC[3] = ( 41.391732, -73.952344 )
WP_LOC[4] = ( 41.391670, -73.952175 )
WP_LOC[5] = ( 41.391696, -73.952907 )
WP_LOC[6] = ( 41.391647, -73.952751 )
WP_LOC[7] = ( 41.391572, -73.952574 )
WP_LOC[8] = ( 41.391540, -73.952404 )
WP_LOC[9] = ( 41.391478, -73.952253 )
WP_LOC[10] = ( 41.391494, -73.953107 )
WP_LOC[11] = ( 41.391432, -73.952946 )
WP_LOC[12] = ( 41.391397, -73.952764 )
WP_LOC[13] = ( 41.391328, -73.952548 )
WP_LOC[14] = ( 41.391267, -73.952365 )
WP_LOC[15] = ( 41.391241, -73.952860 )
WP_LOC[16] = ( 41.391202, -73.953022 )
WP_LOC[17] = ( 41.391173, -73.952946 )
WP_LOC[18] = ( 41.391121, -73.952764 )
WP_LOC[19] = ( 41.391068, -73.952548 )
WP_LOC[20] = ( 41.391011, -73.952365 )
WP_LOC[21] = ( 41.390957, -73.953041 )
WP_LOC[22] = ( 41.390904, -73.952751 )
WP_LOC[23] = ( 41.390842, -73.952574 )
WP_LOC[24] = ( 41.390964, -73.952560 )

kml=simplekml.Kml()

for i in range(0, len(WP_LOC)):
    kml.newpoint(name=str(i), coords=[(WP_LOC[i][1],WP_LOC[i][0])])

kml.save(kml_name)

