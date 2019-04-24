#!/usr/bin/python
# Written by Hayden Trainor, Ben Baumgartner. 10April2019

import simplekml
import csv
import math 
import time

#Grab initial lat and set up
path = "convertThis.csv"


i = 0
start_lat = '0'
with open(path) as csv_file:

    csv_reader = csv.reader(csv_file, delimiter = ',')
    for row in csv_reader:
        lat = row[3]
        lon = row[4]
        rad_info = row[7]
        row_info = str(i) + ',' + str(lat) + ',' + str(lon) + ',' + str(rad_info)
        row_info = row_info.split(',')

        if row_info[0] == '0':
            start_lat = row_info[2]
        i = i +1

    csv_file.close()

dist_width = 2
dist_height = 2
start_lat = float(start_lat)
start_lat_radians = math.cos((start_lat*3.14)/180)
degree_lon = start_lat_radians * 111321.5432
width = (1/degree_lon) * dist_width
height = (1/111321.5432) * dist_height

#KML set up

kml = simplekml.Kml()
i = 0

with open(path) as csv_file:

    csv_reader = csv.reader(csv_file, delimiter = ',')

    for row in csv_reader:
        lat = float(row[3])
        lon = float(row[4])
        rad_info = float(row[7])
        row_info = str(i) + ',' + str(lat) + ',' + str(lon) + ',' + str(rad_info)
        i = i + 1
        row_info = row_info.split(',')
    
        top_left_x = (lon - width/2)
        top_left_y = (lat + height/2)
        bottom_left_x = (lon - width/2)
        bottom_left_y = (lat - height/2)
        bottom_right_x = (lon + width/2)
        bottom_right_y = (lat- height/2)
        top_right_x = (lon + width/2)
        top_right_y = (lat + height/2)

        pol = kml.newpolygon(name = row_info[0], outerboundaryis = ([(top_left_x,top_left_y), (top_right_x,top_right_y), (bottom_right_x,bottom_right_y), (bottom_left_x,bottom_left_y),(top_left_x,top_left_y)]))

        if (rad_info < 50):
            pol.style.polystyle.color = simplekml.Color.changealphaint(100, simplekml.Color.green)

        elif (200 >= rad_info > 598):
            pol.style.polystyle.color = simplekml.Color.changealphaint(100, simplekml.Color.yellow)

        else:
            pol.style.polystyle.color = simplekml.Color.changealphaint(100, simplekml.Color.red)
   
    csv_file.close()

kml.save("/home/user1/usma_swarm/usma_files/serverSide/archive/heatmap.kml")
print("KML file updated in DropBox. Ready for viewing in ATAK.")

