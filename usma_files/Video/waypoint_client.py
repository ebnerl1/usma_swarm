import socket
import sys

waypoints = []

#create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

#THE SENSOR STATION MUST BE .201
#connect the socket to the port where the server is listening
server_address = ('127.0.0.1',10000)
print >>sys.stderr, 'connecting to %s port %s' % server_address
sock.connect(server_address)

try:
    #send data
    message = '1 2 3 4 5 6'
    print >>sys.stderr, 'sending %s' % message
    sock.sendall(message)

    #look for response
    amount_received = 0
    num_symbols = 64
    delta = 64
    while delta == num_symbols:
        data = sock.recv(num_symbols)
        delta = len(data)
        print >>sys.stderr,'received %s' % data

finally:
    print >>sys.stderr, 'closing socket'
    sock.close()
