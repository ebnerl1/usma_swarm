#!/usr/bin/python
import serial
import time
import csv
from os.path import expanduser
import rospy



port = serial.Serial("/dev/rangefinder", baudrate=115200)

def logger():

    rospy.init_node('altitudeeyeNode', anonymous=True)

    # Create a log file for collected data    
    home = expanduser("~")
    altitudeeyeDir = home + "/scrimmage/usma/plugins/autonomy/python/"
    outfile = altitudeeyeDir + "AltitudeeyeLog_"+time.strftime("%Y%m%d_%H%M%S")+".csv"
    print("Data logged to: " + outfile)

     # if the connection was left open, then close it.
    if (port.isOpen()):
        port.close()
    # Open the serial connection
    port.open()


   # Create header line for file write in log file
    output="Altitude\n"    
    with open(outfile, 'a') as rfile:
        rfile.write(output)

    # Start the big while loop
    #rate = rospy.Rate(10) #The altitude takes 1 second to update

    while not rospy.is_shutdown():
        data_raw = port.readline()
        data_raw = data_raw.split()
        data_raw = data_raw[0]
       
        i = 0
        number = 0.00
        
        if len(data_raw) == 4:

            while i <= len(data_raw):
                if i ==2:
                    number = number + float(data_raw[i])/10
                elif i ==3:
                    number = number + float(data_raw[i])/100
                elif i ==0:
                    if data_raw[i] == '\x00':
                        pass
                    else:
                        number = number + float(data_raw[i])
                i+=1

            output=str(number)
            output+="\n"
            print(output)

            with open(outfile, 'a') as rfile:
                rfile.write(output)

            with open(altitudeeyeDir + "altitudefile.csv", 'w') as csv:
                csv.write(output)
                  

        else:
            pass

        
        
        

if __name__ == '__main__':
    try:
        logger()
    except rospy.ROSInterruptException:
        pass
       
