import socket
import sys
import subprocess

outfile = "raw_data.csv"

#create a list of waypoints that must be visited
waypoints = dict()
behaviorflag = False

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
    global behaviorflag
    #create a TCP/IP socket
    sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

    #bind the socket to the port. SENSOR STATION IS 203!!
    server_address = ('192.168.11.202',11000)
    print >>sys.stderr, 'starting up on %s port %s' % server_address
    sock.bind(server_address)

    #Listen for incoming connections. You can increase this but you must also allow threading.
    sock.listen(10)
    active_connections = 0
    while True:
        print >>sys.stderr, 'Waiting for a connection... \n'
        connection, client_address = sock.accept()
        active_connections += 1
        try:
            while True:
                data=connection.recv(1024)
                #newdata = eval(data)
                if (len(data) > 1):
                    print("Message Received:")
                    data = data.split(' ')
                    print(str(data))
                    with open(outfile, 'a') as outf:
                        entry = "{0},{1},{2},{3}\n".format(data[0],data[1],data[2],data[3],data[4])
                        outf.write(entry)
                else:
                    break
        finally:
            connection.close()

if __name__ == "__main__":
    try:
        listen()
    except KeyboardInterrupt:
        pass
