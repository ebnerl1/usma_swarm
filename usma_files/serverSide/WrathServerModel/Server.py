#!/usr/bin/python

import socket
import thread
from threading import Lock
import struct
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
		self.dataList = list()

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
		msgLength = struct.pack("!l", len(msg) + 4)
		msg = msgLength + msg

		self.connectionLock.acquire()
		for connection in self.connections:			
			connection.sendall(msg)
		self.connectionLock.release()


	def handleNewClient(self, connection, addr):
		try:
			while True:
				data=connection.recv(4096)
				if (len(data) == 0):
					break

				self.dataList.extend(data)
				self.checkCompleteMessage()
		except socket.error:
			pass
		finally:
			print "SERVER: Closing Connection"
			connection.close()
			self.connectionLock.acquire()
			self.connections.remove(connection)
			self.connectionLock.release()


	def checkCompleteMessage(self):
		numBytes = self.parseNumBytes()[0]

		while len(self.dataList) >= numBytes and numBytes != -1:
			self.handleMessageData(self.dataList[4:numBytes])
			self.dataList = self.dataList[numBytes:]
			numBytes = self.parseNumBytes()


	def parseNumBytes(self):
		if len(self.dataList) < 4:
			return -1

		return struct.unpack("!l", "".join(self.dataList[:4]))

	
	# This message must be defined!
	def handleMessageData(self, data):
		pass


