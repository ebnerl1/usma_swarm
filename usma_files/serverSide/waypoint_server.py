import socket
import sys
import subprocess
import csv

#create a list of waypoints that must be visited 
finishedwp = set([])
outfile = "raw_data.csv"
wpfile = "wp_data.txt"

#subprocess.Popen(["python", "geogen.py"])

def listen():
    global finishedwp
    global behaviorflag
    #create a TCP/IP socket
    sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

    #bind the socket to the port. SENSOR STATION IS 203!!
    #192.168.11.202
    #server_address = ('127.0.0.1',10000)
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
                    # Heatmap Portion
                    with open(outfile, 'a') as outf:
                        outfwriter = csv.writer(outf)
                        outfwriter.writerow([newdata[5],newdata[6],newdata[7],newdata[8],newdata[0]])
                    with open(wpfile, 'w') as outf:
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
