#!/usr/bin/python
import serial
import time
import csv

'''
Code written by:  MAJ Dominic Larkin
Date: 27 FEB2018

Sources:
WEB PAGE: https://www.thermofisher.com/order/catalog/product/4250630
Communications: RadEye_RemoteControlCommands Dt-029e_.pdf
All pages listed in comments refer to the above PDF

'''

# This function parses a date time used in some messages.
def makeTimeReadable(rawTime):
    year = rawTime >> 26
    month = (rawTime >> 22) & 15
    day = (rawTime >> 17) & 31
    hour = (rawTime >> 12) & 31
    minutes = (rawTime >> 6) & 63
    seconds = (rawTime >> 0) & 63
    return(year,month,day,hour,minutes,seconds)

#if the return value is length 0, then serial communication timed out.
def sendCommand(cmd):
    out = ''
    ser.write('00')
    ser.read(1)
    # time.sleep(5/10000000)
    time.sleep(0.5)
    ser.write(cmd)
    time.sleep(.01)
    while ser.inWaiting() > 0:
        out += ser.read(1)
        time.sleep(.01)
    return out

if __name__ == "__main__":
    controlBytes=['\x02','\x03']
    # Establish a serail port connection IAW specification on page 2-1
    ser = serial.Serial(
        # port='/dev/ttyUSB0',
        port='/dev/ttyUSB0',
        baudrate=9600,
        parity=serial.PARITY_EVEN,
        stopbits=serial.STOPBITS_TWO,
        bytesize=serial.SEVENBITS,
        xonxoff=False,
        rtscts=False,
        dsrdtr=True
    )
    3
    # if the connection was left open, then close it.
    if (ser.isOpen()):
        ser.close()
    # Open the serial connection
    ser.open()

    # Send the commands for the device to repeateadly send values every second.
    ser.write('00')
    ser.read(1)
    # time.sleep(5/10000000) #This line and two above are needed to alert the device that a control command is coming.
    time.sleep(0.5)
    cmd='X1\r\n' # This is the command to make the device start streaming sensor values
    ser.write(cmd)
    time.sleep(.01)

    while True: # Keep running until CTRL + C is pressed.
        out = ''
        while ser.inWaiting() > 0: # While there is a byte in the buffer waiting to be read.
            byte = ser.read() # Read one byte at a time. Readline was not a good solution as it included control bytes.
            if (byte not in controlBytes): # this prevents control bytes from being added to the output.
                out += byte # build the output
        out=out.split(' ') # separate the output into a list, we want the 2nd and 4th element in the list.
        print out
        if len(out) == 9: # if the list is not this long then likely it is garbage
            print ("Top number: %s, Bottom number: %s" %(out[1],out[3]))
            print("file written")
            with open('radfile.csv', 'w') as rfile:
                # rfile.write(str(out[1]) + '\n')
                rfile.write(str(out[1]))
        time.sleep(0.9) # The device updates every second. We can go a little faster to prevent the buffer filling up over time.
