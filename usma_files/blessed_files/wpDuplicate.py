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
        sys.argv[1]
    except IndexError:
        print ("Input Error: Improper Usage \nProper Usage: ./wpDuplicate.py launch.csv")
        return -1

    #Test/Open the given launch file
    try:
        launchfile = open(sys.argv[1], "r")
    except IOError:
        print ("Cannot open ", sys.argv[1])

    #Convert CSV to list for ease of use
    launchfile = list(launchfile)[0].split(",")
    print(launchfile)

    #Test/Open the given WP file
    try:
        open(launchfile[1], "r")
    except IOError:
        print ("Cannot open ", launchfile[1])

    #Store/Rename Framework Long/Lat Point
    x0 = float(launchfile[3])
    y0 = float(launchfile[5])
    x1 = float(launchfile[7])
    y1 = float(launchfile[9])
    number_of_drones = int(launchfile[11])

    #Create the points 
    xs=np.linspace(x0,x1,number_of_drones)
    ys=np.linspace(y0,y1,number_of_drones)

    #print them for error checking
    #for i in range(len(xs)):
    #    print (xs[i],ys[i])

    #Creating files
    for i in range(number_of_drones):
        filename = "quadWP_" + str(i) + ".wp"
        shutil.copy(str(launchfile[1]), filename)
        print("Creating " + filename + "...")
        r = csv.reader(open(filename))
        lines = list(r)
        newWaypoint3 = lines[4][0].split("\t")
        newWaypoint4 = lines[5][0].split("\t")
        newWaypoint5 = lines[6][0].split("\t")
        #id4 = newWaypoint4[0]
        #lat4 = newWaypoint4[8]
        #lon4 = newWaypoint4[9]
        newWaypoint4[8] = str(xs[i])
        newWaypoint4[9] = str(ys[i])
        newWaypoint5[8] = newWaypoint4[8]
        newWaypoint5[9] = newWaypoint4[9]
        newWaypoint3[8] = newWaypoint4[8]
        newWaypoint3[9] = newWaypoint4[9]
        lines[5][0] = "\t".join(newWaypoint4)
        lines[6][0] = "\t".join(newWaypoint5)
        lines[4][0] = "\t".join(newWaypoint3)
        writer = csv.writer(open(filename, 'w'))
        writer.writerows(lines)
        print("Creation of " + filename + "complete!")




    #Conner I don't know why I am typing on this high-speed computer
    #I'm going to laugh when they get these photos on the sit and I actually have no idea what I am typing
    #They are behid me now I hope you enjoy the novel I am leaving you on this computer
         




if __name__ == "__main__":
    main()