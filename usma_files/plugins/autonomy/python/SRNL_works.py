# Performs a sequencial greedy waypoint (WP) assignment:
# 1- Determine which UAVs are in your subswarm
# 2- Split up number of wpts to be assigned among subswarm
# 3- Each UAS individually plans for all UAS in the subswarm in sequence follows:
#    - Determine closest point to that UAS, add that point to your bundle
#    - Find next closest point to that point, add to bundle
#    - Repeat until bundle is full.
#    - As waypoints are added to bundle, make them unavailable to others
#    - Repeat this process for each UAS in sequence
# 4- Each UAS then assigns itself its own set of waypoints
# 5- Each UAS is sequenced to navigate through its bundle of waypoints
#
# Andrew Kopeikin
# USMA
# 27 Jan 2018


import numpy as np
import math
import autopilot_bridge.msg as apbrg
import ap_msgs.msg as apmsg
from autopilot_bridge.msg import LLA
import ap_lib.gps_utils as gps
import ap_lib.ap_enumerations as enums
import ap_lib.sasc_scrimmage as ss
#import spudX_enums as usma_enums
import spud_unified as usma_enums
#import rivercourt_enumerations as usma_enums
import timeit
import socket
import sys
import subprocess
from os.path import expanduser

DIST2WP_QUAD = 10000
DIST2WP_ZEPH = 10
DIST_START_DESCENT = 2
TIME_AT_WP = 1
SAFE_ALT = 100
SURVEY_ALT = 20
BUFFER = 20000
SERVER_FLAG = 1

class InitialPass(ss.Tactic):
	# Initialize survey area--one lane looks like [[[w1,x1],[y1,z1]],[[w2,x2],[y2,z2]]]
	lanes = False
	
    def init(self, params):
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
        if (self._spud == 1):
            self._enumList = usma_enums.WP_LOC_S1
        elif (self._spud == 0):
            self._enumList = usma_enums.WP_LOC_SX
        elif (self._spud == 4):
            self._enumList = usma_enums.WP_LOC_S4
        elif (self._spud == 5):
            self._enumList = usma_enums.WP_LOC_S5
        elif (self._spud == 6):
            self._enumList = usma_enums.WP_LOC_RiverCourt
        self._name = 'InitialPass'
        #self._location = int(params['location'])
        #self._desired_lat = float(self._enumList[self._location][0])
        #self._desired_lon = float(self._enumList[self._location][1])

        # Initialize Variables for Waypoint Assignment
        self._subswarm_id = 0
        self._id_in_subswarm = []
        self._first_tick = True
        self._subswarm_num_to_assign = []
        self._subswarm_wp_assignment = dict()
        self._wp_assigned = []
        for i in range(0, len(self._enumList)):
            self._wp_assigned.append(False)

		# Initialize Variables for Waypoint Assignment
		self._subswarm_id = 0
		self._id_in_subswarm = [] # number of vehicles?
		self._first_tick = True
		self._subswarm_num_to_assign = []
		self._subswarm_wp_assignment = dict()
		self._wp_assigned = []
		for i in range(0, len(usma_enums.WP_LOC)):
		    self._wp_assigned.append(False)


    def step_autonomy(self, t, dt):
        # Execute this portion of the code on the loop only
        if self._first_tick == True:
            raddir = self.radeyeDir + 'radeye.py'
            subprocess.Popen(["python", raddir]) # Runs Radeye in the background

            self._first_tick = False
            finishedset = set([])
            #create a TCP/IP socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            #serverflag = 0
            if (SERVER_FLAG == 1):
              server_address = ('192.168.11.202',10000)
            else: 
              server_address = ('127.0.0.1',10000)
            #server_address = ('192.168.11.202',10000) 
            #connect the socket to the port where the server is listening
            #server_address = ('127.0.0.1',10000)
            print >>sys.stderr, '---connecting to %s port %s---' % server_address
            sock.connect(server_address)
            try:

                messageArray = [self._id, 100000, 100000, 99999, 100000, self._desired_lat, self._desired_lon, 0, self._desired_alt, "PRDER"]
                message = str(messageArray)
                sock.sendall(message)

                #look for response
                num_symbols = 4096
                delta = 4096
                while delta == num_symbols:
                    data = sock.recv(num_symbols)
                    delta = len(data)
                    print >>sys.stderr,'Received: %s' % data
                finishedset = eval(data)

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

            for n in finishedset:
              self._wp_assigned[n] = True

            # Divide # of waypoints by # of vehicles and create empty bundle of wpts for each
            num_in_subwarm = len(self._id_in_subswarm)
            for i in range(0, num_in_subwarm):
                self._subswarm_num_to_assign[i] = (len(self._enumList)-len(finishedset))/(num_in_subwarm)
                if i < ((len(self._enumList)-len(finishedset)) % num_in_subwarm):
                    self._subswarm_num_to_assign[i] = self._subswarm_num_to_assign[i] + 1
            print "num_to_assign: ", self._subswarm_num_to_assign


            # Perform sequencial greedy wpt assignment.  Loop over each UAS in subswarm.
			for i in range(0, num_in_subwarm): #from 0 to 5
		        # Set the start location to current UAS position
				place_in_lane = 0
				if i > 0:
					if i == num_in_subwarm - 1:
						place_in_lane = i*self._subswarm_num_to_assign[i-1]
					else:
						place_in_lane = i*self._subswarm_num_to_assign[i] # [3,3,2]
					print ("place in lane :", place_in_lane)

				temp_lat = blue_in_subswarm[i].state.pose.pose.position.lat
				temp_lon = blue_in_subswarm[i].state.pose.pose.position.lon
				assignment_list = []
		        # Loop over each element of the lanes bundle 
				for j in range(0, self._subswarm_num_to_assign[i]): #self._sub_num_to_assign[0] = 6
		            # Loop over each end of lane defined in the mission
					if i > 0:
						for k in self.lanes[j+place_in_lane]:
							coord_to_assign = k
							assignment_list.append(coord_to_assign)
							self._subswarm_wp_assignment[i] = assignment_list
					else:
						for k in self.lanes[j]: #looking at both
							#coordinates in a single lane--should be 2
							coord_to_assign = k #full coordinate
							assignment_list.append(coord_to_assign)
							self._subswarm_wp_assignment[i] = assignment_list

				print "list for drone: ",i, ": ", self._subswarm_wp_assignment[i], "\n"
		        # Assign yourself your own bundle of lanes
				print "vehicle id: ", blue_in_subswarm[i].vehicle_id, "self._id: ", self._id
		        #if blue_in_subswarm[i].vehicle_id == self._id:
				if self._id == blue_in_subswarm[i].vehicle_id:
					self._wp_id_list = self._subswarm_wp_assignment[i]
					print "vehicle id: ", self._id, " waypoints: ", self._subswarm_wp_assignment[i]
					print "this is the i: ", i
					#self._loc = self._wp_id_list[self._wp_id_list_id]


        # Go to desired latitude, longitude, and maintain altitude
        # deconfliction:
        self._wp = np.array([self._desired_lat, self._desired_lon,
                             self._desired_alt])
        #self._wp = np.array([self._desired_lat, self._desired_lon,
                             #self._last_ap_wp[2]])

        pos = self._own_pose.pose.pose.position
        dist = gps.gps_distance(pos.lat, pos.lon, self._desired_lat, self._desired_lon)

        # Detect whether UAS has arrived at WP (within threshold distance), track time at WP
        # Zephyrs (type 2) loitersocket.error: [Errno 104] Connection reset by peer around point, so set threshold distance > loiter radius
        # Set threshold distance for Quads (type 1), much smaller

        SURVEY_ALT = self._base_alt + (self._alt_change * self._id_in_subswarm.index(self._id)) #redefine SURVEY_ALT within the loop to stack behavior

        if (self._vehicle_type == 2 and dist < DIST2WP_ZEPH) or (self._vehicle_type == 1 and dist < DIST2WP_QUAD):
            if self._at_wp == False:
                if (pos.rel_alt > SURVEY_ALT - BUFFER) and (pos.rel_alt < SURVEY_ALT + BUFFER):
                    self._time_start = timeit.default_timer()
            self._at_wp = True
            self._time_at_wp = timeit.default_timer() - self._time_start
        else:
            self._at_wp = False
            self._time_at_wp = 0



          ############################# HEATMAP VARS ##############################
            def getcounts():
                
                with open(self.radeyeDir + 'radfile.csv', 'r') as radfile:
                    firstline = radfile.readline().split(',')
                    print(firstline)
                    return((firstline[0],firstline[1])) 
            lat = pos.lat
            lon = pos.lon
            alt = pos.alt
            counts, radtype = getcounts()
            print(counts)
            print(radtype)
            #########################################################################
            #create a TCP/IP socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            #192.168.11.202
            #connect the socket to the port where the server is listening
            #server_address = ('192.168.11.202',10000)
            #server_address = ('127.0.0.1', 10000)
            #serverflag = 0
            if (SERVER_FLAG == 1):
              server_address = ('192.168.11.202',10000)
            else: 
              server_address = ('127.0.0.1',10000)
            print >>sys.stderr, '---connecting to %s port %s---' % server_address
            sock.connect(server_address)
            try:
                #send data
                #str(self._subswarm_wp_assignment)

                #messageArray = [self._id, [0,0,0,0,0], self._loc, self._wp_id_list.index(self._loc), [0,0,0,0], lat, lon, counts, alt]
                messageArray = [self._id, len(self._wp_id_list), self._loc, self._wp_id_list.index(self._loc), len(self._enumList), lat, lon, counts, alt, radtype] 
                message = str(messageArray)
                print("UAV ID: " + str(self._id))
                print("Sending Message to Base Station...")
                sock.sendall(message)


                #look for response 4194304
                amount_received = 0
                num_symbols = 4096
                delta = 4096
                while delta == num_symbols:
                    data = sock.recv(num_symbols)
                    delta = len(data)
                    #print >>sys.stderr,'Received: %s' % data


            finally:
                print >>sys.stderr, '---closing socket---'
                sock.close()

		# If you reach your point, proceed to the next one
		if self._at_wp == True:
			self._wp_id_list_id = self._wp_id_list_id + 1

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
