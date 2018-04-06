import socket
import sys
import subprocess

#create a list of waypoints that must be visited 
finishedwp = set([])
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
    global finishedwp
    global behaviorflag
    #create a TCP/IP socket
    sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

    #bind the socket to the port. SENSOR STATION IS 203!!
    #192.168.11.202
    server_address = ('192.168.11.202',10000)
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
            #quad_num = str(client_address[0]).split('.')[3]
            #print >>sys.stderr, 'connection from UAS#%s on port %s' % (quad_num,client_address[1])
        
            while True:
                data=connection.recv(1024)
                print("Connected!")
                #newdata = eval(data)
                if (len(data) > 1):
                    newdata = eval(data)
                    savedata = eval(data)
                    print >>sys.stderr, 'Connection from UAS#%s on Port %s' % (newdata[0],client_address[1])
                    print("Working WP List: " + str(newdata[1]))
                    print("Finished WP: " + str(newdata[2]))
                    finishedwp.add(newdata[2])
                    print("Finished WP Set: " + str(finishedwp))
                    print("At Index: " + str(newdata[3]))
                    
                    #sendall argument must be string or buffer, not a list
                    print("Sending back a message...")
                    #sendbackmsg = [newdata[4],newdata[3]]
                    connection.sendall(str(finishedwp))

                else:
                    #print >>sys.stderr, 'no more data from UAS#', quad_num
                    #sendbackmsg = [savedata[4],savedata[3]]
                    #connection.sendall(str(sendbackmsg))
                    break
                
        finally:
            connection.close()
        
if __name__ == "__main__":
    try:
        listen()
    except KeyboardInterrupt:
        pass
