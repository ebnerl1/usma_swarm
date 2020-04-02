#!/usr/bin/python

# gregory.zogby@westpoint.edu 

import socket
import thread
from threading import Lock
from threading import Timer
import struct
import sys
import subprocess
import procname
import time
import logging

from MessageHandler import MessageHandler

class HeartbeatMessage(object):
    def pack(self):
        return struct.pack("!l", -1)

class Server(object):
	
	def __init__(self, timeout = -1):
		self.timeout = timeout
		self.connectionLock = Lock()
		self.connections = list()
		self.socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		self.dataList = list()
		self.messageHandler = MessageHandler()
		
		self.byteFmt = "!l"
		self.byteLen = struct.calcsize(self.byteFmt)

	def start(self, ipAddress, port):
		try:
			self.listen(ipAddress, port)
		except KeyboardInterrupt:
			self.close()


	def close(self):
		for connection in self.connections:
			logging.info("SERVER: Closing Connection")
			connection.close()
		if (self.socket != None):
			self.socket.close()


	def listen(self, ipAddress, port):
		self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)

		logging.info("SERVER: Starting server on %s:%s" % (ipAddress, port))
		self.socket.bind((ipAddress, port))

		logging.info("SERVER: Waiting for a connection...")
		self.socket.listen(10)
		
		# Timer(0.5, self.sendHeartbeat).start()

		while True:
			connection, addr = self.socket.accept()

			# Start a new thread to handle the socket
			self.connectionLock.acquire()
			self.connections.append(connection)
			self.connectionLock.release()
			logging.info("SERVER: Num Connections: " + str(len(self.connections)))

			if (self.timeout > -1):
				connection.settimeout(self.timeout)

			thread.start_new_thread(self.handleNewClient, (connection, addr))

	
	def broadcast(self, msg):
		data = msg.pack()
		msgLength = struct.pack(self.byteFmt, len(data) + self.byteLen)
		data = msgLength + data

		self.connectionLock.acquire()
		for connection in self.connections:
			connection.sendall(data)
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
			logging.info("SERVER: Closing Connection")
			connection.close()
			self.connectionLock.acquire()
			self.connections.remove(connection)
			self.connectionLock.release()


	def checkCompleteMessage(self):
		numBytes = self.parseNumBytes()

		while len(self.dataList) >= numBytes + self.byteLen and numBytes != -1:
			logging.info("SERVER: Received Message " + str(numBytes) + " " + str(struct.unpack_from("!l", "".join(self.dataList[self.byteLen:self.byteLen*2]))))
			try:
				string = "".join(self.dataList[self.byteLen:numBytes])
				self.dataList = self.dataList[numBytes:]
				self.messageHandler.processMessage(string)
				numBytes = self.parseNumBytes()
			except struct.error:
				logging.error("Num Bytes: " + str(numBytes) + " Message: " + "".join(self.dataList[self.byteLen:numBytes]))


	def parseNumBytes(self):
		if len(self.dataList) < self.byteLen:
			return -1

		return struct.unpack(self.byteFmt, "".join(self.dataList[:self.byteLen]))[0]


	def sendHeartbeat(self):
		self.broadcast(HeartbeatMessage())
		# Timer(0.5, self.sendHeartbeat).start()


	def registerMessageCallback(self, id, callback):
		self.messageHandler.registerCallback(id, callback)

