import socket
import sys
import subprocess
import csv
import procname

procname.setprocname("serverSide")

#create a list of waypoints that must be visited 
finishedwp = set([])
outfile = "raw_data.csv"

#subprocess.Popen(["python", "geogen2.py"])

wpfile = "wp_data.txt"

#subprocess.Popen(["python", "geogen.py"])

def listen():
    global finishedwp
    global behaviorflag
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
            msgbuffer = ''
            data = True
            while data:
                data=connection.recv(1024)
                msgbuffer += data
                print("Connected!")
                print(data)
                #msg = eval(data)
            print(msgbuffer)
                
            if (len(msgbuffer) > 1):
                msg = eval(msgbuffer)
                #print(msg)
                print >>sys.stderr, 'Connection from UAS#%s on Port %s' % (msg[0],client_address[1])
                print("Working WP List: " + str(msg[1]))
                print("Finished WP: " + str(msg[2]))
                if (msg[2] < 10000):                  
                   finishedwp.add(msg[2])
                print("Finished WP Set: " + str(finishedwp))
                print("# of Finished: " + str(len(finishedwp))) + "/" + str(msg[4])
                print("At Index: " + str(msg[3]+1) + "/" + str(msg[1]))
                    
                #sendall argument must be string or buffer, not a list
                print("Sending back a message...")
                connection.sendall(str(finishedwp))
                if ((len(finishedwp)) == msg[4]):
                   print("FINISHED!")
                   quit()
                    #sendbackmsg = [msg[4],msg[3]]
            elif (len(msgbuffer) == 1):
                print("one")
                connection.sendall(str(finishedwp))
            else:
                break                
                    # Heatmap Portion
            with open(outfile, 'a') as outf:
                 outfwriter = csv.writer(outf)
                 outfwriter.writerow([msg[5],msg[6],msg[7],msg[8],msg[0]])

            with open(wpfile, 'a') as outf:
                 outf.write(str(finishedwp) + '\n')

                  
        finally:
            connection.close()
        
if __name__ == "__main__":
    try:
        listen()
    except KeyboardInterrupt:
        pass
