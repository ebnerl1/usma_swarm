#!/usr/bin/python

import socket
import thread
from threading import Lock
import sys
import subprocess
import procname
import time

class Server(object):
	
	def __init__(self, timeout = -1):
		self.timeout = timeout
		self.connectionLock = Lock()
		self.connections = list()
		self.socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

	def start(self, ipAddress, port):
		try:
			self.listen(ipAddress, port)
		except KeyboardInterrupt:
			self.close()


	def close(self):
		for connection in self.connections:
			print "SERVER: Closing Connection"
			connection.close()
		if (self.socket != None):
			self.socket.close()


	def listen(self, ipAddress, port):
		self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

		print "SERVER: Starting server on %s:%s" % (ipAddress, port)
		self.socket.bind((ipAddress, port))

		print "SERVER: Waiting for a connection..."
		self.socket.listen(10)
		
		while True:
			connection, addr = self.socket.accept()

			# Start a new thread to handle the socket
			self.connectionLock.acquire()
			self.connections.append(connection)
			self.connectionLock.release()
			print "SERVER: Num Connections: ", len(self.connections)

			if (self.timeout > -1):
				connection.settimeout(self.timeout)
			thread.start_new_thread(self.handleNewClient, (connection, addr))

	
	def broadcast(self, msg):
		self.connectionLock.acquire()
		for connection in self.connections:
			connection.sendall(str(msg))
		self.connectionLock.release()


	def handleNewClient(self, connection, addr):
		try:
			while True:
				data=connection.recv(4096)
				if (len(data) <= 1):
					break
				parsedData = eval(data)

				returnMessage = self.handleMessageData(parsedData)

				connection.sendall(str(returnMessage))
		except socket.error:
			pass
		finally:
			print "SERVER: Closing Connection"
			connection.close()
			self.connectionLock.acquire()
			self.connections.remove(connection)
			self.connectionLock.release()

	
	# This message must be defined!
	def handleMessageData(self, parsedData):
		pass


