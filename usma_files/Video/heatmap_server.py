import socket
import sys
import subprocess

#create a list of waypoints that must be visited
waypoints = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]

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
    server_address = ('192.168.11.202',10000)
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
                    update_waypoints(data,quad_num)
                    print >>sys.stderr, 'sending updated waypoint list back to UAS#',quad_num

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
