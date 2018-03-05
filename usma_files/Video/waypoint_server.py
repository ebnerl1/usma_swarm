import socket
import sys
import subprocess

infile = "sample_data.csv" # Input file as a .csv
outfile = "sample_gj.geojson" # Output file as a .geojson
radcap = 100 # Highest amount of radiation to scale color gradient
sqrrad = 0.0005 # Area covered by one drone as a "square radius"
interval = 5 # Time in seconds in between scans

#create a list of waypoints that must be visited 
waypoints = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]

#############################################################################################################

geoempty = True

# Decides color based on ratio of gradient to radcap, RGB
def colorSelect(rads):
    const = (rads/radcap) * 255
    color = (int(const), int(255 - const), 0)
    return '#%02x%02x%02x' % color

# Converts sample counts to actual counts
def altConvert(sample, alt):
    return (sample * (alt**2))

# Iterates through entire parsed file to overwrite .geojson
def writeGJ(data):
    
    data = data.split(' ')
    entry = [data[0],data[1],colorSelect(altConvert(data[3], data[2]))]

    outf = open(outfile, 'r')
    lines = outf.readlines()
    outf.close()
    outf = open(outfile, 'w')
    outf.writelines([item for item in lines[:-2]])
    outf.close()
  
    global geoempty

    with open(outfile, 'a') as outf:

        # Is supposed to deal with the difference between the first 
        if geoempty == False:
            outf.write(',\n')
        else:
            outf.write('{\n')
            outf.write('  "type": "FeatureCollection",\n')
            outf.write('  "features": [\n')
            geoempty = False

        outf.write('    {\n')
        outf.write('      "type": "Feature",\n')
        outf.write('      "properties": {\n')
        outf.write('        "stroke": "#999999",\n')
        outf.write('        "stroke-width": 0.5,\n')
        outf.write('        "stroke-opacity": 0.8,\n')
        outf.write('        "fill": "{0}",\n'.format(entry[3]))
        outf.write('        "fill-opacity": 0.6\n')
        outf.write('      },\n')
        outf.write('      "geometry": {\n')
        outf.write('        "type": "Polygon",\n')
        outf.write('        "coordinates": [\n')
        outf.write('          [\n')
        outf.write('            [{0},{1}],\n'.format(float(entry[0]) - sqrrad, float(entry[1]) - sqrrad))
        outf.write('            [{0},{1}],\n'.format(float(entry[0]) + sqrrad, float(entry[1]) - sqrrad))
        outf.write('            [{0},{1}],\n'.format(float(entry[0]) + sqrrad, float(entry[1]) + sqrrad))
        outf.write('            [{0},{1}],\n'.format(float(entry[0]) - sqrrad, float(entry[1]) + sqrrad))
        outf.write('            [{0},{1}]\n'.format(float(entry[0]) - sqrrad, float(entry[1]) - sqrrad))
        outf.write('          ]\n')
        outf.write('        ]\n')
        outf.write('      }\n')
        outf.write('    }\n')
        outf.write('  ]\n')
        outf.write('}')

#############################################################################################################

#updates the waypoints based on the quads updates
def update_waypoints(data,quad_num):
    global waypoints
    if(data.find(quad_num) >= 0):
        waypoint = int(data.split(' ')[1])
        try:
            waypoints.index(waypoint)
            waypoints.remove(waypoint)
            process = "./snap-odroid-image.sh " +str(quad_num)+" " + str(waypoint)
            subprocess.call(process,shell=True)
        except:
            pass
        
def list2string(list1):
    output = ""
    for i in range(0,len(list1)-1):
        output = output + str(list1[i]) + " "
    output = output + str(list1[len(list1)-1])
    return output
    
def listen():
    global waypoints
    #create a TCP/IP socket
    sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

    #bind the socket to the port. SENSOR STATION IS 203!!
    server_address = ('192.168.11.203',10000)
    print >>sys.stderr, 'starting up on %s port %s' % server_address
    sock.bind(server_address)

    #Listen for incoming connections. You can increase this but you must also allow threading.
    sock.listen(10)
    active_connections = 0
    while True:
        print >>sys.stderr, 'waiting for a connection'
        connection, client_address = sock.accept()
        active_connections += 1
        try:
            quad_num = str(client_address[0]).split('.')[3]
            print >>sys.stderr, 'connection from UAS#%s on port %s' % (quad_num,client_address[1])
        
            while True:
                data=connection.recv(32)
                if (len(data) > 1):
                    print >>sys.stderr, 'Message received from UAS#%s: %s' % (quad_num, data)
                    #update_waypoints(data,quad_num)
                    #print >>sys.stderr, 'sending updated waypoint list back to UAS#',quad_num
                    print >>sys.stderr, 'Adding data to geojson'
                    writeGJ(data)
                    print >>sys.stderr, 'Done'
                    #senall argument must be string or buffer, not a list
                    connection.sendall(list2string(waypoints))
                else:
                    print >>sys.stderr, 'no more data from UAS#', quad_num
                    break
                
        finally:
            connection.close()
        
if __name__ == "__main__":
    try:
        listen()
    except KeyboardInterrupt:
        pass
