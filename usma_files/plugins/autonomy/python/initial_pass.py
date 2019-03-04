# Note: We will begin with a fixed number of drones and lanes. For our
# testing purposes, we will begin with 5 drones splitting 10 lanes
#
# 1- Determine which UAVs are in your subswarm
# 2- Split up number of lanes to be assigned among subswarm
# 3- Each UAS individually plans for all UAS in the subswarm
# 4- Each UAS then assigns itself its own set of lanes
# 5- Each UAS is sequenced to navigate through its bundle of lanes
#  
# Ashley Rivera
# USMA
# 28 Jan 2019

import numpy as np
import math
import autopilot_bridge.msg as apbrg
import ap_msgs.msg as apmsg
from autopilot_bridge.msg import LLA
import ap_lib.gps_utils as gps
import ap_lib.ap_enumerations as enums
import ap_lib.sasc_scrimmage as ss
import new_rivercourt_enumerations as usma_enums
import createLanes
import timeit

DIST2WP_QUAD = 1
DIST2WP_ZEPH = 75
DIST_START_DESCENT = 20
TIME_AT_WP = 10
SAFE_ALT = 25
SURVEY_ALT = 20

class InitialPass(ss.Tactic):
	# Initialize survey area--one lane looks like [[[w1,x1],[y1,z1]],[[w2,x2],[y2,z2]]]
	lanes = False
	
	def init(self, params):
		if self.lanes == False:
			self.lanes = createLanes.createLanes() #should come out to 10 lanes
		self._id = int(params['id'])
		self._target_id = -1
		self._wp = np.array([0, 0, 0])
		self._max_range = enums.MAX_RANGE_DEF
		self._fov_width = enums.HFOV_DEF
		self._fov_height = enums.VFOV_DEF
		self._own_pose = apbrg.Geodometry()
		self._blues = dict()
		#self._reds = dict()
		self._shot = set()
		self._safe_waypoint = np.array([0, 0, 0])
		self._last_ap_wp = np.array([0, 0, 0])
		self._action = ["None", 0]
		self._vehicle_type = int(params['vehicle_type'])
		self._name = 'InitialPass'
		#self._location = int(params['location']) #waypoint ID?
		self._desired_lat = float(self.lanes[0][0][0])
		self._desired_lon = float(self.lanes[0][0][1])
		self._speed = 10

		# Initialize Variables for Waypoint Assignment
		self._subswarm_id = 0
		self._id_in_subswarm = [] # number of vehicles?
		self._first_tick = True
		self._subswarm_num_to_assign = []
		self._subswarm_wp_assignment = dict()
		self._wp_assigned = []
		for i in range(0, len(usma_enums.WP_LOC)):
		    self._wp_assigned.append(False)  

		# Initialize Variables for Sequencing between Waypoints
		self._wp_id_list = []   # List of WPs to be assigned to this UAS
		for i in range(0,len(self.lanes)*2):
			self._wp_id_list.append([41.390863,-73.952947])
		self._wp_id_list_id = 0     # Index within assigned list of WPs
		self._loc = self._wp_id_list[self._wp_id_list_id]
		#self._desired_lat = float(usma_enums.WP_LOC[self._loc][0])
		#self._desired_lon = float(usma_enums.WP_LOC[self._loc][1])
		self._desired_alt = SAFE_ALT
		self._original_alt = SAFE_ALT
		self._at_wp = False
        
	def step_autonomy(self, t, dt):
		# Execute this portion of the code on the loop only
		if self._first_tick == True:
			self._first_tick = False			

			# Set initial altitude settings
			self._desired_alt = self._last_ap_wp[2]
			self._original_alt = self._last_ap_wp[2]

			# Determine your own subswarm ID
				#key     #value
			for blue_id, blue in self._blues.iteritems():
				print "vehicle id = ", blue.vehicle_id, "self._id = ", self._id
				if blue.vehicle_id == self._id:
					self._subswarm_id = blue.subswarm_id
					break
			print "subswarm_id: ", self._subswarm_id

			# Build a list of all vehicle IDs within your subswarm
			# produces the list (and therefore number) of specific swarm
			blue_in_subswarm = dict()
			i = 0
			for blue_id, blue in self._blues.iteritems():
				if blue.subswarm_id == self._subswarm_id:
					self._id_in_subswarm.append(blue.vehicle_id)
					self._subswarm_num_to_assign.append(0) 
					blue_in_subswarm[i] = blue
					i = i+1        
			print "id_in_subswarm: ", self._id_in_subswarm

			# Divide # of lanes by # of vehicles and create empty bundle of lanes for each 
			num_in_subwarm = len(self._id_in_subswarm) # in our case 5 vehicles
			for i in range(0, num_in_subwarm):
				self._subswarm_num_to_assign[i] = int(math.floor(len(self.lanes)/num_in_subwarm))
				if i < (len(self.lanes) % num_in_subwarm):
					self._subswarm_num_to_assign[i] = self._subswarm_num_to_assign[i] + 1
			print "num_to_assign: ", self._subswarm_num_to_assign #should be 2
		    
		    # Logic for assigning lanes and movement of subswarm
		    # Loop over each UAS in subswarm.
			for i in range(0, num_in_subwarm): #from 0 to 5
		        # Set the start location to current UAS position
				place_in_lane = 0
				if i > 0:
					place_in_lane += self._subswarm_num_to_assign[i-1]

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

				#print "list for drone: ",i, ": ", self._subswarm_wp_assignment[i], "\n"
		        # Assign yourself your own bundle of lanes
				print "vehicle id: ", blue_in_subswarm[i].vehicle_id, "self._id: ", self._id
		        #if blue_in_subswarm[i].vehicle_id == self._id:
				if self._id == blue_in_subswarm[i].vehicle_id:
					self._wp_id_list = self._subswarm_wp_assignment[i]
						#print "vehicle id: ", self._id, " waypoints: ", self._subswarm_wp_assignment[i]
					print "it has reached this point"
					#self._loc = self._wp_id_list[self._wp_id_list_id]

			print "way point id list :", self._wp_id_list
			# Proceed to the first Waypoint in the bundle
			self._loc = self._wp_id_list[0]
			self._desired_lat = float([self._loc][0][0])
			self._desired_lon = float([self._loc][0][1])
		    
			print "subswarm_wp_assignment: ", self._subswarm_wp_assignment, "\n"
		    # Proceed to the first coordinate in the lanes bundle

			print "Going to coordinate: ", self._loc
			# Go to desired latitude, longitude, and maintain altitude
			# deconfliction:
			self._wp = np.array([self._desired_lat, self._desired_lon, 
								self._desired_alt])

		# Determine current position
		pos = self._own_pose.pose.pose.position
		dist = gps.gps_distance(pos.lat, pos.lon, self._desired_lat, self._desired_lon)

		#if close enough to a hotspot (using hotspot threshold as test), send info to one drone on standby
		#call hotspot_identified function
		
		if (self._vehicle_type == 2 and dist < DIST2WP_ZEPH) or (self._vehicle_type == 1 and dist < DIST2WP_QUAD):
			if self._at_wp == False:
				self._time_start = timeit.default_timer()
			self._at_wp = True
			self._time_at_wp = timeit.default_timer() - self._time_start
		else:
			self._at_wp = False
			self._time_at_wp = 0

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