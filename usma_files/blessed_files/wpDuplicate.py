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
    
    ax0 = float(launchfile[13])
    ay0 = float(launchfile[15])
    ax1 = float(launchfile[17])
    ay1 = float(launchfile[19])

    bx0 = float(launchfile[21])
    by0 = float(launchfile[23])
    bx1 = float(launchfile[25])
    by1 = float(launchfile[27])

    #Create the points 
    xs=np.linspace(x0,x1,number_of_drones)
    ys=np.linspace(y0,y1,number_of_drones)

    axs=np.linspace(ax0,ax1,number_of_drones)
    ays=np.linspace(ay0,ay1,number_of_drones)

    bxs=np.linspace(bx0,bx1,number_of_drones)
    bys=np.linspace(by0,by1,number_of_drones)

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
        
        #LAUNCH coordinates
        newWaypoint3 = lines[4][0].split("\t")
        newWaypoint4 = lines[5][0].split("\t")
        newWaypoint5 = lines[6][0].split("\t")

        #LAND A coordinates
        newWaypoint21 = lines[22][0].split("\t")
        newWaypoint23 = lines[24][0].split("\t")
        newWaypoint25 = lines[26][0].split("\t")
        newWaypoint26 = lines[27][0].split("\t")

        #LAND B coordinates
        newWaypoint29 = lines[30][0].split("\t")
        newWaypoint31 = lines[32][0].split("\t")
        newWaypoint33 = lines[34][0].split("\t")
        newWaypoint34 = lines[35][0].split("\t")
        
        #Uncomment to see ID/LAT/LONG for specific point
        #id4 = newWaypoint4[0]
        #lat4 = newWaypoint4[8]
        #lon4 = newWaypoint4[9]

        #Dynamically allocated coordinates
        coordX = str(xs[i])
        coordY = str(ys[i])

        acX = str(axs[i])
        acY = str(ays[i])

        bcX = str(bxs[i])
        bcY = str(bys[i])
        
        #Changing LAUNCH coordinates
        newWaypoint3[8] = coordX
        newWaypoint3[9] = coordY
        newWaypoint4[8] = coordX
        newWaypoint4[9] = coordY
        newWaypoint5[8] = coordX
        newWaypoint5[9] = coordY
        
    
        lines[4][0] = "\t".join(newWaypoint3)
        lines[5][0] = "\t".join(newWaypoint4)
        lines[6][0] = "\t".join(newWaypoint5)
        
        
        #Changing LAND A coordinates
        newWaypoint21[8] = acX
        newWaypoint21[9] = acY
        newWaypoint23[8] = acX
        newWaypoint23[9] = acY
        newWaypoint25[8] = acX
        newWaypoint25[9] = acY
        newWaypoint26[8] = acX
        newWaypoint26[9] = acY

        lines[22][0] = "\t".join(newWaypoint21)
        lines[24][0] = "\t".join(newWaypoint23)
        lines[26][0] = "\t".join(newWaypoint25)
        lines[27][0] = "\t".join(newWaypoint26)


        #Changing LAND B coordinates
        newWaypoint29[8] = bcX
        newWaypoint29[9] = bcY
        newWaypoint31[8] = bcX
        newWaypoint31[9] = bcY
        newWaypoint33[8] = bcX
        newWaypoint33[9] = bcY
        newWaypoint34[8] = bcX
        newWaypoint34[9] = bcY

        lines[30][0] = "\t".join(newWaypoint29)
        lines[32][0] = "\t".join(newWaypoint31)
        lines[34][0] = "\t".join(newWaypoint33)
        lines[35][0] = "\t".join(newWaypoint34)

        #Splyce it all together
        writer = csv.writer(open(filename, 'w'))
        writer.writerows(lines)
        print("Creation of " + filename + "complete!\n")




    #Conner I don't know why I am typing on this high-speed computer
    #I'm going to laugh when they get these photos on the sit and I actually have no idea what I am typing
    #They are behid me now I hope you enjoy the novel I am leaving you on this computer
         




if __name__ == "__main__":
    main()