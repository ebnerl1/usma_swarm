import socket
import sys
import subprocess
import csv
import procname
import time
from datetime import datetime
import json
import urllib
import rasterio
import numpy as np
import os

procname.setprocname("serverSide")

######################## VARIABLES ########################

outfile = "parsed_data.js" # Output file as a .geojson
archive_path = "archive/archive_" + str(datetime.now()) + ".csv"
radcap = 1 # Highest amount of radiation to scale color gradient
interval = 5 # Time in seconds in between scans
mapalt = 0
online = True
elevationFile = 'srtm_14_04.tif'
log = []
heatmapdata = []

#create a list of waypoints that must be visited 
finishedwp = set([])
wpfile = "wp_data.txt"

###########################################################

def elevationOnline(lat, lng):
    apikey = "AIzaSyDvuEAYeb9xoSun0PHXVkM7oxl_sRZD2H4"
    url = "https://maps.googleapis.com/maps/api/elevation/json"
    request = urllib.urlopen(url+"?locations="+str(lat)+","+str(lng)+"&key="+apikey)
    try:
        results = json.load(request).get('results')
        if 0 < len(results):
            elevation = results[0].get('elevation')
            # ELEVATION
            return elevation
        else:
            print ('HTTP GET Request failed.')
    except ValueError:
        print ('JSON decode failed: '+str(request))

def elevationOffline(lat, lng):
    global elevationFile
    coords = [(lat,lng)]
    elevation = 0
    with rasterio.open(elevationFile) as src:
        for val in src.sample(coords):
            elevation = val
        return elevation[0]
        
def countsconvert(rawcounts, absalt, mapalt):
    heightAboveGround = absalt - mapalt
    conv = rawcounts * ((heightAboveGround)**2)
    return conv
    
def writeArchive(log):
    global archive_path
    script_dir = os.path.dirname(__file__)
    absolute_path = os.path.join(script_dir, archive_path)

    with open(archive_path, 'w') as outf:
        outfwriter = csv.writer(outf)
        for i in log:
            outfwriter.writerow(i)

def listen():
    global finishedwp
    global behaviorflag
    global log
    global heatmapdata
    global radcap
    #create a TCP/IP socket
    sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

    #bind the socket to the port. SENSOR STATION IS 203!!
    #192.168.11.202
    serverflag = 0
    if (serverflag == 1):
        server_address = ('192.168.11.202',10000)
    else: 
        server_address = ('127.0.0.1',10000)

    #server_address = ('192.168.11.202',10000)
    print >>sys.stderr, 'Starting up on %s port %s...' % server_address
    sock.bind(server_address)

    #Listen for incoming connections. You can increase this but you must also allow threading.
    sock.listen(10)
    active_connections = 0
    savedata = None
    while True:
        print >>sys.stderr, 'Waiting for a connection... \n'
        connection, client_address = sock.accept()
        active_connections += 1
        try:
            while True:
                data=connection.recv(4096)
                #print("Connected!")
                #newdata = eval(data)
                if (len(data) > 1):
                
                    timeNow = str(datetime.now())
                    newdata = eval(data)
                    
                    print >>sys.stderr, 'Connection from UAS#%s on Port %s' % (newdata[0],client_address[1])
                    print("Working WP List: " + str(newdata[1]))
                    print("Finished WP: " + str(newdata[2]))
                    if (newdata[2] < 10000):                  
                      finishedwp.add(newdata[2])
                    print("Finished WP Set: " + str(finishedwp))
                    print("# of Finished: " + str(len(finishedwp))) + "/" + str(newdata[4])
                    print("At Index: " + str(newdata[3]+1) + "/" + str(newdata[1]))
                    
                    #sendall argument must be string or buffer, not a list
                    print("Sending back a message...")
                    if ((len(finishedwp)) == newdata[4]):
                      print("FINISHED!")
                      quit()
                    #sendbackmsg = [newdata[4],newdata[3]]
                    connection.sendall(str(finishedwp))
                    # Heatmap Portion
                    
                    droneID = newdata[0]
                    lat = float(newdata[5])
                    lon = float(newdata[6])
                    rawcounts = float(newdata[7])
                    absalt = float(newdata[8])
              
                    if online:
                        mapalt = float(elevationOnline(lat, lon))
                    else:
                        mapalt = float(elevationOffline(lat,lon))
                    convcounts = countsconvert(rawcounts, absalt, mapalt) 
                    
                    log.append([timeNow, droneID, lat, lon, absalt, mapalt, rawcounts, convcounts])
                    heatmapdata.append([lat, lon, convcounts])
                    
                    with open(outfile, 'w') as outf:
                        radcap = 1
                        for i in range(len(heatmapdata)):
                            if heatmapdata[i][2] > radcap:
                                radcap = heatmapdata[i][2]
                                
                        outfwriter = csv.writer(outf)
                        for i in heatmapdata:
                            outfwriter.writerow([i[0], i[1], i[2]/radcap])

                    writeArchive(log)

                    with open(wpfile, 'a') as outf:
                        outf.write(str(finishedwp) + '\n')

                else:
                    break
                
        finally:
            connection.close()
        
if __name__ == "__main__":
    try:
        listen()
    except KeyboardInterrupt:
        pass
