#!/usr/bin/env python

#^^Adding the shebang above to make the script executable
#Remember to do chmod +x <filename>

import csv

import sys
import numpy as np
import shutil



def main():
    
    #Test for proper user input
    try:
        test = sys.argv[6]
    except IndexError as err:
        print ("Input Error: Improper Usage \nProper Usage: ./wpDuplicate.py wpFile.wp x0 y0 x1 y1 numDrones")
        return -1

    #Test/Open the given wp file
    try:
        file = open(sys.argv[1], "r")
    except IOError:
        print ("Cannot open ", sys.argv[1])

    #Store/Rename Framework Long/Lat Point
    x0 = float(sys.argv[2])
    y0 = float(sys.argv[3])
    x1 = float(sys.argv[4])
    y1 = float(sys.argv[5])
    number_of_drones = int(sys.argv[6])

    #create the points 
    xs=np.linspace(x0,x1,number_of_drones)
    ys=np.linspace(y0,y1,number_of_drones)

    #print them for error checking
    for i in range(len(xs)):
        print (xs[i],ys[i])

    #Test manipulation of file
    start = 0

    for line in file:
    	if start == 0:
    		start = 1
    	else:
    		fields = line.split("\t")
    		id = fields[0]
    		lat = fields[8]
    		lon = fields[9]
    		if id == '5':
    			print ("id is " + str(id) + " lat is " + str(lat) + " long is " + str(lon))

    file.close()

    #Creating files
    for i in range(number_of_drones):
        filename = "quadWP_" + str(i) + ".wp"
        shutil.copy(str(sys.argv[1]), filename)
        r = csv.reader(open(filename))
        lines = list(r)
        newWaypoint4 = lines[5][0].split("\t")
        newWaypoint5 = lines[6][0].split("\t")
        id4 = newWaypoint4[0]
        lat4 = newWaypoint4[8]
        lon4 = newWaypoint4[9]
        newWaypoint4[8] = str(xs[i])
        newWaypoint4[9] = str(ys[i])
        newWaypoint5[8] = newWaypoint4[8]
        newWaypoint5[9] = newWaypoint4[9]
        lines[5][0] = "\t".join(newWaypoint4)
        lines[6][0] = "\t".join(newWaypoint5)
        writer = csv.writer(open(filename, 'w'))
        writer.writerows(lines)

    tester = csv.reader(open("launch.csv"))
    test_lines = list(tester)
    for line in test_lines:
        print (line)


    #Testing opening up of document as Xlsx file

    #r = csv.reader(open("quadWP_0.wp"))
    #lines = list(r)
    #newLine = lines[6][0].split("\t")
    #print ("This is the newLine: ", newLine)
    #id = newLine[0]
    #lat = newLine[8]
    #lon = newLine[9]
    #newLine[8] = "1234"
    #newLine[9] = "4321"
    #print ("This is the newer newLine: ", newLine)
    #lines[6][0] = "\t".join(newLine)
    #writer = csv.writer(open("quadWP_0.wp", 'w'))
    #writer.writerows(lines)

    #Conner I don't know why I am typing on this high-speed computer
    #I'm going to laugh when they get these photos on the sit and I actually have no idea what I am typing
    #They are behid me now I hope you enjoy the novel I am leaving you on this computer
         




if __name__ == "__main__":
    main()