#!/usr/bin/python

from WrathServerModel import Server

# Messages Drone to Server:
# Phase 2: m[0] = 1
#	Starting Initial Pass: m[1] = 1
#	Radiation Data:        m[1] = 2
#		TODO: Radiation Data Format
#	Finished Initial Pass: m[1] = 3
# Phase 3: m[0] = 2
#
# Messages Server to Drone:
# Phase 2: m[0] = 1
#	Finished Init Contour: m[1] = 1
#		TODO: Initial Contour Format
# Phase 3: m[0] = 1
class RadDetectionServer(Server.Server):

	def __init__(self, simulationData = None):
		super(RadDetectionServer, self).__init__()
		self.state = 1
		self.numDronesInSwarm = 0
		self.dronesFinished = 0
		if (simulationData != None):
			self.IS_SIMULATION = True
			self.simulationData = simulationData
		else:
			self.IS_SIMULATION = False


	def handleMessageData(self, data):
		if (data[0] == 0):
			return [0]
		if (self.state == 0 and data[0] == 1):
			print "MODEL: State Change: Initial Pass!"
			self.state = 1
		if (self.state == 1):
			self.handleInitialPassData(data)
			if (self.dronesFinished == self.numDronesInSwarm):
				self.state = 2
				print "MODEL: State Change: Lane Generation!"
				print "MODEL: Sending points: ", self.simulationData
				if self.IS_SIMULATION:
					self.broadcast(str([1, 1, self.simulationData]))
				else:
					# Broadcast real rad data
					pass
		return [0]


	def handleInitialPassData(self, data):
		if data[0] != 1:
			raise Exception("Invalid Message Type!!!")
		if data[1] == 1:
			print "MODEL: Added a drone!"
			self.numDronesInSwarm += 1
		elif data[1] == 2:
			# TODO: Handle radiation data coming in
			pass
		elif data[1] == 3:
			print "MODEL: A drone finished!"
			self.dronesFinished += 1

