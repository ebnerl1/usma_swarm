# Peforms lane assignment while sending/receiving data 
# to/from waypoint_server.py
# Based on class InitialPass
#
# Ashley Rivera
# USMA
# Last updated: 15 APR 2019

import rospy
import numpy as np
import math
import autopilot_bridge.msg as apbrg
import ap_msgs.msg as apmsg
from autopilot_bridge.msg import LLA
import ap_lib.gps_utils as gps
import ap_lib.ap_enumerations as enums
import ap_lib.sasc_scrimmage as ss
import spud_unified as usma_enums
import createLanes
import timeit
import time
import socket
import sys
import subprocess
from os.path import expanduser
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

DIST2WP_QUAD = 1
DIST2WP_ZEPH = 10
DIST_START_DESCENT = 2
TIME_AT_WP = 1
TIME_PASSED = 0.02
SAFE_ALT = 100
SURVEY_ALT = 20
BUFFER = 20000
SERVER_FLAG = 1
GROUND_TEST = 1

# If GPS Simulator used, expand distance since quad won't move
if (GROUND_TEST == 1):
    DIST2WP_QUAD = 1000000
    DIST_START_DESCENT = 1000000
    BUFFER = 200000
    TIME_AT_WP = 3

class InitialPass(ss.Tactic):

	
	def init(self, params):

		self.lanes = createLanes.createLanes()
		self._totalList = []
		self._totalList_id = 0
		for lane in self.lanes:
			for coord in lane:
				self._totalList.append(coord)
		self._lane_id = 0
		self._id = int(params['id'])
		self._target_id = -1
		self._wp = np.array([0, 0, 0])
		self._own_pose = apbrg.Geodometry()
		self._blues = dict()
		self._shot = set()
		self._safe_waypoint = np.array([0, 0, 0])
		self._last_ap_wp = np.array([0, 0, 0])
		self._action = ["None", 0]
		self._vehicle_type = int(params['vehicle_type'])
		self._spud = int(params['Survey Number'])
		self.home = expanduser("~")
		self.radeyeDir = self.home + "/scrimmage/usma/plugins/autonomy/python/"
		self._enumList = dict()
		if (self._spud == 1):
			self._enumList = usma_enums.WP_LOC_rivercourt
		elif (self._spud == 2):
			self._enumList = usma_enums.WP_LOC_Range11
		self._name = 'InitialPass'
		self._radType = 0

		# Initialize Variables for Waypoint Assignment
		self._subswarm_id = 0
		self._id_in_subswarm = []
		self._first_tick = True
		self._subswarm_num_to_assign = []
		self._subswarm_wp_assignment = dict()
		self._wp_assigned = []
		self._lane_assigned = []
		for i in range(0, len(self.lanes)):
			self._wp_assigned.append(False)

		# Initialize Variables for Sequencing between Waypoints
		self._wp_id_list = [] # List of WPs to be assigned to this UAS
		for i in range(0, len(self.lanes*2)):
			self._wp_id_list.append(i) # Place holder for other logic
		self._wp_id_list_id = 0 # Index within assigned list of WPs
		self._loc = self._wp_id_list[self._wp_id_list_id]
		self._desired_lat = float(self._enumList[self._loc][0])
		self._desired_lon = float(self._enumList[self._loc][1])
		self._desired_alt = self._last_ap_wp[2]
		self._original_alt = SAFE_ALT
		self._time_at_wp = 0
		self._time_passed = 0
		self._time_start = 0
		self._at_wp = False
		self._base_alt = int(params['Base Altitude'])
		self._alt_change = int(params['Altitude Change'])

		self._altitude = 0.0
		self._radiation = 0.0
		topic='/autopilot/acs_pose'
		outstring = "THIS IS MODIFIED TO FIND error logs"
		print >>sys.stderr, "TESTINGTESTING1 "+ outstring
		rospy.Subscriber('/tactic_interface/altitude', PointStamped, self.altitude_cb)
		rospy.Subscriber('/tactic_interface/radiation', PointStamped, self.radeye_cb)

	def radeye_cb(self, msg):
		self._radiation = msg.point.x
		self._radType = msg.point.y

	# Sends data to .ros/logs on odroid
	def altitude_cb(self, msg):
		self._altitude = msg.point.x
		self._parent.log_info("The altitude is: %f" %msg.point.x)


	def step_autonomy(self, t, dt):

		# Execute this portion of the code on the loop only
		if self._first_tick:
			raddir = self.radeyeDir + 'radeye.py'
			subprocess.Popen(["python", raddir]) # Runs Radeye in the background

			self._first_tick = False
			finishedset = set([])
			#create a TCP/IP socket
			sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			if (SERVER_FLAG == 1):
			  server_address = ('192.168.11.202',10000)
			else: 
			  server_address = ('127.0.0.1',10000)
			print >>sys.stderr, '---connecting to %s port %s---' % server_address
			sock.connect(server_address)

			# Initialize data sent to server
			messageArray = [self._id, 100000, 100000, 99999, 100000, self._desired_lat, self._desired_lon, 0, self._desired_alt, "PRDER", 100000, 0, 0]
			message = str(messageArray)
			print(message)

			try:
				sock.sendall(message)

				# Look for response from server
				num_symbols = 4096
				delta = 4096
				while delta == num_symbols:
					data = sock.recv(num_symbols)
					delta = len(data)
					print >>sys.stderr,'Received: %s' % data
				temp_finishedset = eval(data)
				finishedset = temp_finishedset[0]

			finally:
				print >>sys.stderr, '---closing socket---'
				sock.close()

			# Set initial altitude settings
			self._desired_alt = self._last_ap_wp[2]
			self._original_alt = self._last_ap_wp[2]

			# Determine your own subswarm ID
			   #key     #value
			for blue_id, blue in self._blues.iteritems():
				if blue.vehicle_id == self._id:
					self._subswarm_id = blue.subswarm_id
					break
			print "subswarm_id: ", self._subswarm_id
			print(finishedset)

			# Build a list of all vehicle IDs within your subswarm
			blue_in_subswarm = dict()
			i = 0
			for blue_id, blue in self._blues.iteritems():
				if blue.subswarm_id == self._subswarm_id:
					self._id_in_subswarm.append(blue.vehicle_id)
					self._subswarm_num_to_assign.append(0)
					blue_in_subswarm[i] = blue
					i = i+1
			print "id_in_subswarm: ", self._id_in_subswarm

			if isinstance(finishedset, list):
				for n in finishedset:
					self._wp_assigned[n] = True

			# Divide # of lanes by # of vehicles and create empty bundle of lanes for each 
			num_in_subwarm = len(self._id_in_subswarm)
			for i in range(0, num_in_subwarm):
				self._subswarm_num_to_assign[i] = int(math.floor(len(self.lanes)/num_in_subwarm))
				if i < (len(self.lanes) % num_in_subwarm):
					self._subswarm_num_to_assign[i] = self._subswarm_num_to_assign[i] + 1
			print "num_to_assign: ", self._subswarm_num_to_assign 

			# Perform sequencial greedy wpt assignment. Loop over each UAS in subswarm.
			place_in_lane = 0
			for i in range(num_in_subwarm):
				# Set the start location to current UAS position
				if i > 0:
					place_in_lane = self._subswarm_num_to_assign[i-1]+place_in_lane

				assignment_list = []
				# Loop over each element of the lanes bundle 
				for j in range(self._subswarm_num_to_assign[i]): 
					print "subswarm num to assign is : ", self._subswarm_num_to_assign
					# Loop over each end of lane defined in the mission
					if i > 0:
						for k in self.lanes[j+place_in_lane]:     
							coord_to_assign = k
							assignment_list.append(coord_to_assign)
							self._subswarm_wp_assignment[i] = assignment_list
					else:
						for k in self.lanes[j]: 
							coord_to_assign = k # full coordinate
							assignment_list.append(coord_to_assign)
							self._subswarm_wp_assignment[i] = assignment_list

				# Assign yourself your own bundle of lanes
				if self._id == blue_in_subswarm[i].vehicle_id:
					self._wp_id_list = self._subswarm_wp_assignment[i]

			# Proceed to the first Waypoint in the bundle
			self._loc = self._wp_id_list[0]
			self._desired_lat = float([self._loc][0][0])
			self._desired_lon = float([self._loc][0][1])

		self._parent.log_info("The altitude global is: %f and radiation is: %f" %(self._altitude,self._radiation))

		self._wp = np.array([self._desired_lat, self._desired_lon,
							 self._desired_alt])

		pos = self._own_pose.pose.pose.position
		dist = gps.gps_distance(pos.lat, pos.lon, self._desired_lat, self._desired_lon)

		# Detect whether UAS has arrived at WP (within threshold distance)
		# Set threshold distance for Quads (type 1), much smaller

		SURVEY_ALT = self._base_alt + (self._alt_change * self._id_in_subswarm.index(self._id)) 

		if (self._vehicle_type == 1 and dist < DIST2WP_QUAD):
			self._at_wp = True
		else:
			self._at_wp = False

		# Sets data reading to every 2 seconds
		if self._time_start == 0:
			self._time_start = time.clock()
		self._time_passed = time.clock() - self._time_start
		if self._time_passed > TIME_PASSED:
			self._time_start = time.clock()

			lat = pos.lat
			lon = pos.lon
			alt = pos.alt 
			if (self._radType == 1): radtype = "PRDER"
			else: radtype = "GN+"

			#create a TCP/IP socket
			sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

			if (SERVER_FLAG == 1):
			  server_address = ('192.168.11.202',10000)
			else: 
			  server_address = ('127.0.0.1',10000)
			print >>sys.stderr, '---connecting to %s port %s---' % server_address
			sock.connect(server_address)

			num_lanes = len(self._wp_id_list)/2
			messageArray = [self._id, len(self._wp_id_list), self._totalList_id, self._wp_id_list_id, len(self._totalList), lat, lon, self._radiation, self._altitude, radtype, self._lane_id, num_lanes, alt] 
			message = str(messageArray)
			print(message)

			try:

				print("UAV ID: " + str(self._id))
				print("Sending Message to Base Station...")
				sock.sendall(message)

			finally:
				print >>sys.stderr, '---closing socket---'
				sock.close()

		# If you reach your point, proceed to the next one
		if self._at_wp == True:
			self._wp_id_list_id = self._wp_id_list_id + 1

			index = 0
			for point in self._totalList:
				if point == self._loc:
					self._totalList_id = index
				else:
					index += 1

			if self._totalList_id > (len(self._totalList)-1):
				self._totalList_id = 0
			
			if self._lane_id > (len(self.lanes)-1):
				self._lane_id = 0
			if self._wp_id_list_id > 0 and self._wp_id_list_id % 2 == 0:
				self._lane_id += 1

			# If you get to the end of your bundle, repeat from its beginning 
			# and reset to original altitude
			if self._wp_id_list_id > (len(self._wp_id_list)-1):
				print("test") 
				self._wp_id_list_id = 0
				self._desired_alt = self._original_alt
			self._loc = self._wp_id_list[self._wp_id_list_id]
			self._desired_lat = float(self._loc[0])
			self._desired_lon = float(self._loc[1]) 
			# Reset these so that UAV knows it's no longer at its goal WP
			self._at_wp = False
			print "Going to coordinate: ", self._loc
			self._wp = np.array([self._desired_lat, self._desired_lon, self._desired_alt])

		return True