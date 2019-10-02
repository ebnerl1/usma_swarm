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
import ap_lib.swarm_helper_v2 as pursueBytes
from autopilot_bridge.msg import LLA
import ap_lib.gps_utils as gps
import ap_lib.ap_enumerations as enums
import ap_lib.sasc_scrimmage as ss
import trial_enumerations as usma_enums
import timeit

#Implementation:
import socket
from os.path import expanduser
import subprocess
import sys
from geometry_msgs.msg import PointStamped#BB
import rospy
import map_around_central_point as hotspot_grid

DIST2WP_QUAD = 4
DIST2WP_ZEPH = 75
DIST_START_DESCENT = 20
TIME_AT_WP = 3
SAFE_ALT = 100
SURVEY_ALT = 20

#Implementation:
BUFFER = 2
SERVER_FLAG = 0
GROUND_TEST = 0
grid = 0

#If GROUND_TEST, expand distance since quad won't move
if (GROUND_TEST == 1):
    DIST2WP_QUAD = 1000000
    DIST_START_DESCENT = 1000000
    BUFFER = 200000
    TIME_AT_WP = 1

class Savidgeswarm(ss.Tactic):

    def init(self, params):
        self._id = int(params['id'])
        self._target_id_new = -1
        self._wp = np.array([0, 0, 0])
        self._max_range = enums.MAX_RANGE_DEF
        self._fov_width = enums.HFOV_DEF
        self._fov_height = enums.VFOV_DEF
        self._own_pose = apbrg.Geodometry()
        self._blues = dict()
        self._shot = set()
        self._safe_waypoint = np.array([0, 0, 0])
        self._last_ap_wp = np.array([0, 0, 0])
        self._action = ["None", 0]
        self._vehicle_type = int(params['vehicle_type'])
        self._name = 'SavidgeSwarm'

#Implementation:
        self.home = expanduser("~")
        self.radeyeDir = self.home + "/scrimmage/usma/plugins/autonomy/python/"
        self._altitude = 0.0
        self._radiation = 0.0
        topic='/autopilot/acs_pose'
        outstring = "THIS IS MODIFIED TO FIND error logs"
        print >>sys.stderr, "TESTINGTESTING1 "+ outstring
        rospy.Subscriber('/tactic_interface/altitude', PointStamped, self.altitude_cb)
        rospy.Subscriber('/tactic_interface/radiation', PointStamped, self.radeye_cb)
        self._radType = 0
        self._enumList = usma_enums.WP_LOC

        # Initialize Variables for Waypoint Assignment
        self._subswarm_id = 0
        self._id_in_subswarm = []
        self._first_tick = True
        self._subswarm_num_to_assign = []
        self._subswarm_wp_assignment = dict()
        self._wp_assigned = []
        for i in range(0, len(usma_enums.WP_LOC)):
            self._wp_assigned.append(False)  

        # Initialize Variables for Sequencing between Waypoints
        self._wp_id_list = []   # List of WPs to be assigned to this UAS
        for i in range(0, len(usma_enums.WP_LOC)):
            self._wp_id_list.append(i)  # Place holder for other logic
        self._wp_id_list_id = 0     # Index within assigned list of WPs
        self._loc = self._wp_id_list[self._wp_id_list_id]
        self._desired_lat = float(usma_enums.WP_LOC[self._loc][0])
        self._desired_lon = float(usma_enums.WP_LOC[self._loc][1])
        self._desired_alt = self._last_ap_wp[2]
        self._original_alt = SAFE_ALT
        self._time_at_wp = 0
        self._time_start = 0
        self._at_wp = False
        self._base_alt = int(params['Base Altitude']) #sam g new
        self._alt_change = int(params['Altitude Change']) #sam g new

        self._belief = []   #Predicted values of waypoints
        self._previous_value = 0    #Place holder for future use

        self._new_WPs = []  #WPs read in last iteration
        self._previous_lowest = 99999

        self._Known_WP_LOC = []     #Known WP values
        for i in range(0,len(usma_enums.WP_LOC)):
            self._Known_WP_LOC.append(-1)
            self._belief.append(0)

        self._lat_points = usma_enums.lat_points    #Number of latitudinal points
        self._long_points = usma_enums.long_points  #Number of longitudinal points

        self._perimeter_WP = []     #Points on the perimeter
        for i in range(0,self._long_points):
            self._perimeter_WP.append(i)
            self._perimeter_WP.append((self._lat_points-1)*self._long_points+i)
        for i in range(1,self._lat_points):
            self._perimeter_WP.append(i*self._long_points)
            self._perimeter_WP.append(i*self._long_points+self._long_points-1)

        self._time = 0  #Keeps track of time starting from 0
        self._num_UAVs = 0  #Initialized for later use
        self._blue_in_subswarm = dict() #Initialized for later use
        self._propogation_decrease = 2  #Decrease from one WP to the next
        self.pursuit_status = True  #For later use

        #For sending and receiving messages        
        try:
            topic = 'network/send_swarm_behavior_data'
            # we dont need to create a subscriber here because this is taken care of in 
            self.pubs[topic] = self._parent._behavior_data_publisher 
        except AttributeError:
            self.pubs[topic] = \
                self.createPublisher(topic, apmsg.BehaviorParameters, 1)
            self.subs[topic] = \
                self.createSubscriber(
                    topic, apmsg.BehaviorParameters, self._process_swarm_data_msg
                )

    #Function for sending messages
    def pursuit_status_update(self):
        parser = pursueBytes.PursuitMessageParser()
        parser.friendly_id = self._id

        parser.target_id_new = self._loc
        parser.target_distance = self._Known_WP_LOC[self._loc]

        parser.pursuit_status = True

        data = parser.pack()
        net_msg = apmsg.BehaviorParameters()
        net_msg.id = pursueBytes.PURSUIT_MESSAGE
        net_msg.params = data
        for _ in range(1): # Hope to get at least one through
            self.pubs['network/send_swarm_behavior_data'].publish(net_msg)

    def radeye_cb(self, msg):#BB
    	self._radiation = msg.point.x
        self._radType = msg.point.y

    def altitude_cb(self, msg):#BB
        self._altitude = msg.point.x
        #self._parent.log_info("The altitude issss: %f" %msg.point.x)
        
    def step_autonomy(self, t, dt):
        # Execute this portion of the code on the loop only
        if self._first_tick == True:
            self._first_tick = False    #Makes sure this only happens once
            self._time = timeit.default_timer()     #Sets start time
            
            #Implementation:
            raddir = self.radeyeDir + 'radeye.py' # NEEDED?
            subprocess.Popen(["python", raddir]) # Runs Radeye in background NEEDED?
            finishedset = set([])  # Comment this if want it from wpt_server

            ### SOCKET COMS ##########################################
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

                messageArray = [self._id, 100000, 100000, 99999, 100000, self._desired_lat, self._desired_lon, 0, self._desired_alt, "PRDER", 100000, 0, 0]
                #messageArray = [self._id, 100000, 100000, 99999, 100000, 99.99, 99.99, 0, self._desired_alt, "PRDER", 100000, 0, 0]
                message = str(messageArray)
                sock.sendall(message)

                #look for response
                num_symbols = 4096
                delta = 4096
                while delta == num_symbols:
                    data = sock.recv(num_symbols)
                    delta = len(data)
                    print >>sys.stderr,'Received: %s' % data
                temp_finishedset = eval(data)
                finishedset = eval(temp_finishedset[0])
                print "temp finishedset: ", temp_finishedset
                #self._enumList = hotspot_grid.createGrid(float(temp_finishedset[1]),float(temp_finishedset[2])) #Uncomment if want it from server
                #print("length of hotspot grid list: ",len(self._enumList))
            finally:
                print >>sys.stderr, '---closing socket---'
                sock.close()
            ###--END SOCKET COMS ########################################

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
 
            # Build a list of all vehicle IDs within your subswarm
            blue_in_subswarm = dict()
            i = 0
            for blue_id, blue in self._blues.iteritems():
                if blue.subswarm_id == self._subswarm_id:
                    self._id_in_subswarm.append(blue.vehicle_id)
                    self._subswarm_num_to_assign.append(0) 
                    blue_in_subswarm[i] = blue
                    self._blue_in_subswarm[i] = blue
                    i = i+1        
            print "id_in_subswarm: ", self._id_in_subswarm

            # Divide # of waypoints by # of vehicles and create empty bundle of wpts for each 
            num_in_subwarm = len(self._id_in_subswarm)
            self._num_UAVs = num_in_subwarm
            for i in range(0, num_in_subwarm):
                self._subswarm_num_to_assign[i] = int(math.floor(len(usma_enums.WP_LOC)/num_in_subwarm))
                if i < (len(usma_enums.WP_LOC) % num_in_subwarm):
                    self._subswarm_num_to_assign[i] = self._subswarm_num_to_assign[i] + 1
            print "num_to_assign: ", self._subswarm_num_to_assign

            # Perform sequencial greedy wpt assignment.  Loop over each UAS in subswarm.
            for i in range(0, num_in_subwarm):
                # Set the start location to current UAS position
                temp_lat = blue_in_subswarm[i].state.pose.pose.position.lat
                temp_lon = blue_in_subswarm[i].state.pose.pose.position.lon
                assignment_list = []
                # Loop over each element of the waypoint bundle 
                for j in range(0, self._subswarm_num_to_assign[i]):
                    min_dist = 99999 #initialize as large number
                    new_wp_assigned = False
                    # Loop over each waypoint defined in the mission  
                    for k in range(0, len(usma_enums.WP_LOC)):
                        # Skip to next if that waypoint is already assigned
                        if self._wp_assigned[k] == False:
                            # Set the end location to that waypoint
                            temp2_lat = float(usma_enums.WP_LOC[k][0])
                            temp2_lon = float(usma_enums.WP_LOC[k][1])
                            # Check if start to end location distance is new minimum, if so mark
                            # it for assignment
                            temp_dist = gps.gps_distance(temp_lat, temp_lon, temp2_lat, temp2_lon)
                            if temp_dist < min_dist:
                                min_dist = temp_dist
                                wp_to_assign = k
                                new_wp_assigned = True

                    # Add the next closest waypoint to the bundle
                    if new_wp_assigned == True:
                        assignment_list.append(wp_to_assign)
                        self._subswarm_wp_assignment[i] = assignment_list
                        # Mark that waypoint as "assigned" so unavailable to others
                        self._wp_assigned[wp_to_assign] = True
                        # Update the start location to that waypoint
                        temp_lat = float(usma_enums.WP_LOC[wp_to_assign][0])
                        temp_lon = float(usma_enums.WP_LOC[wp_to_assign][1])
                # Assign yourself your own bundle of waypoints
                if blue_in_subswarm[i].vehicle_id == self._id:
                    self._wp_id_list = self._subswarm_wp_assignment[i]
            print "subswarm_wp_assignment: ", self._subswarm_wp_assignment
            # Proceed to the first Waypoint in the bundle
            self._loc = self._wp_id_list[0]
            self._desired_lat = float(usma_enums.WP_LOC[self._loc][0])
            self._desired_lon = float(usma_enums.WP_LOC[self._loc][1]) 
            print "Going to wp: ", self._loc


        # Go to desired latitude, longitude, and maintain altitude
        # deconfliction:
        self._wp = np.array([self._desired_lat, self._desired_lon, 
                             self._desired_alt])

        pos = self._own_pose.pose.pose.position
        dist = gps.gps_distance(pos.lat, pos.lon, self._desired_lat, self._desired_lon)
         
        # Detect whether UAS has arrived at WP (within threshold distance), track time at WP
        # Zephyrs (type 2) loiter around point, so set threshold distance > loiter radius
        # Set threshold distance for Quads (type 1), much smaller
              
        if (self._vehicle_type == 2 and dist < DIST2WP_ZEPH) or (self._vehicle_type == 1 and dist < DIST2WP_QUAD):
            if self._at_wp == False:
                self._time_start = timeit.default_timer()
            self._at_wp = True
            self._time_at_wp = timeit.default_timer() - self._time_start
        else:
            self._at_wp = False
            self._time_at_wp = 0

        # Detect if Quad is close enough to first WP to descend to survey altitude
        if self._vehicle_type == 1 and dist < DIST_START_DESCENT: 
            self._desired_alt = SURVEY_ALT
             
        # After X time has elapsed at WP, move onto next WP in your bundle
        if self._time_at_wp > TIME_AT_WP:
#Implementation:

          ############################# HEATMAP VARS ##############################
            # Legacy code to be stripped out
            #def getcounts():
                
            #    with open(self.radeyeDir + 'radfile.csv', 'r') as radfile:
            #        firstline = radfile.readline().split(',')
            #        print(firstline)
            #        return((firstline[0],firstline[1])) 
            lat = pos.lat
            lon = pos.lon
            alt = pos.alt
            if (self._radType == 1): radtype = "PRDER"
            else: radtype = "GN+"
            #counts, radtype = getcounts()
            #print(counts)
            #print(radtype)
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
                messageArray = [self._id, len(self._wp_id_list), self._loc, self._wp_id_list.index(self._loc), len(self._enumList), lat, lon, self._radiation, self._altitude, radtype, 0, 0, alt]
                #messageArray = [self._id, len(self._wp_id_list), self._totalList_id, self._wp_id_list_id, len(self._totalList), lat, lon, self._radiation, self._altitude, radtype, self._lane_id, 0, alt] 
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
            #########################################################################

#Implementation:
            # Calculate Radiation Value from Readings and Alt
            if radtype == "PRDER":
                background = 15.6
                rval = 0.66
            else:
                background = 9.6
                rval = 0.52
            newcounts = self._radiation - background
            if newcounts < 0:
                print("No radiation detected here")
                newcounts = 0
            print("heightaboveground: " + str(self._altitude))
            if self._altitude < 0:
                print("Net height is negative")
                self._altitude = 0
            conv = (newcounts * ((self._altitude)**2) * 4 * math.pi) / rval
            if conv <= 0:
                conv = 0.0000001

            self._Known_WP_LOC[self._loc] = conv #Reads value at WP        
            self._new_WPs.append(self._loc) #Adds WP to newly read list
            self.pursuit_status_update()    #Sends message to update other UAVs
            self._loc = self._wp_id_list[self._wp_id_list_id]

            self._wp_id_list_id = self._wp_id_list_id + 1
            # If you get to the end of your bundle, repeat from its beginning 
            # and reset to original altitude
            if self._wp_id_list_id > (len(self._wp_id_list)-1):
                self._wp_id_list_id = 0
                self._desired_alt = self._original_alt
            self._loc = self._wp_id_list[self._wp_id_list_id]
            self._desired_lat = float(usma_enums.WP_LOC[self._loc][0])
            self._desired_lon = float(usma_enums.WP_LOC[self._loc][1]) 
            # Reset these so that UAV knows it's no longer at its goal WP
            self._at_wp = False
            self._time_at_wp = 0
            print "Going to wp: ", self._loc
    
        if timeit.default_timer()-self._time >= 60: #The process will update every number of specified seconds
            #First need to update beliefs based on newly read values
            for new_wp in range(0,len(self._new_WPs)):
                q = 1 #Keeps track of how many WPs belief's update
                i = 1 #number of WPs away from center
                while q > 0:
                    q = 0
                    for j in range(-i,i+1):
                        for k in range(-i,i+1):
                            outside = "False"
                            #These checks ensure we're not propogating outside of the perimeter
                            if j <= 0:
                                if k <= 0:
                                    for outsidek in range(k,1):
                                        for outsidej in range(j,1):
                                            if new_wp+outsidek*self._long_points+outsidej in self._perimeter_WP or new_wp+outsidek*self._long_points+outsidej < 0 or new_wp+outsidek*self._long_points+outsidej > len(self._Known_WP_LOC)-1:
                                                outside = "True"
                                elif k > 0:
                                    for outsidek in range(1,k+1):
                                        for outsidej in range(j,1):
                                            if new_wp+outsidek*self._long_points+outsidej in self._perimeter_WP or new_wp+outsidek*self._long_points+outsidej < 0 or new_wp+outsidek*self._long_points+outsidej >= len(self._Known_WP_LOC)-1:
                                                outside = "True"
                            elif j > 0:
                                if k <= 0:
                                    for outsidek in range(k,1):
                                        for outsidej in range(1,j+1):
                                            if new_wp+outsidek*self._long_points+outsidej in self._perimeter_WP or new_wp+outsidek*self._long_points+outsidej < 0 or new_wp+outsidek*self._long_points+outsidej > len(self._Known_WP_LOC)-1:
                                                outside = "True"
                                elif k > 0:
                                    for outsidek in range(1,k+1):
                                        for outsidej in range(1,j+1):
                                            if new_wp+outsidek*self._long_points+outsidej in self._perimeter_WP or new_wp+outsidek*self._long_points+outsidej < 0 or new_wp+outsidek*self._long_points+outsidej > len(self._Known_WP_LOC)-1:
                                                outside = "True"                
                            #Update propogated belief as long as it's not outside perimeter and greater than previous belief
                            if outside == "False":  
                                if self._Known_WP_LOC[new_wp]/self._propogation_decrease**i > self._belief[new_wp+self._long_points*k+j]:                                    
                                    self._belief[new_wp+self._long_points*k+j] = self._Known_WP_LOC[new_wp]/self._propogation_decrease**i
                                    q = q+1
                    i = i+1                                
            self._new_WPs = []  #Resets the newly read WPs array so they don't later get double counted

            #For setting loewst read value to the rest of the WP belief values
            lowest_WP_value = 99999
            for i in range(0,len(usma_enums.WP_LOC)):
                if self._Known_WP_LOC[i] < lowest_WP_value and self._Known_WP_LOC[i] != -1:
                    lowest_WP_value = self._Known_WP_LOC[i]
            for i in range(0,len(usma_enums.WP_LOC)):
                if self._belief[i] == self._previous_lowest:
                    self._belief[i] = lowest_WP_value
            self._previous_lowest = lowest_WP_value

            #Then update behavior based on new beliefs          
            # Perform sequencial greedy wpt assignment. Loop over each UAS in subswarm.
            for i in range(0,self._num_UAVs):
                self._subswarm_num_to_assign[i] = 0
            self._subswarm_wp_assignment = dict()
            self._wp_id_list = []   # List of WPs to be assigned to this UAS
            for i in range(0, len(usma_enums.WP_LOC)):
                self._wp_id_list.append(i)
                self._wp_assigned[i] = False
            assign_WPs = 0 
            #Check which WPs haven't been read:
            for x in range(0,len(self._Known_WP_LOC)):
                if self._Known_WP_LOC[x] == -1:
                    assign_WPs = assign_WPs+1
            #These 2 lines intended to prevent issue of running out of WPs
            if assign_WPs < self._num_UAVs:
                return True
            for i in range(0, self._num_UAVs):
                self._subswarm_num_to_assign[i] = int(math.floor(assign_WPs/self._num_UAVs))
                if i < (assign_WPs % self._num_UAVs):
                    self._subswarm_num_to_assign[i] = self._subswarm_num_to_assign[i] + 1
                print self._subswarm_num_to_assign[i]           
            for i in range(0, self._num_UAVs):
                # Set the start location to current UAS position
                temp_lat = self._blue_in_subswarm[i].state.pose.pose.position.lat
                temp_lon = self._blue_in_subswarm[i].state.pose.pose.position.lon
                assignment_list = []
                # Loop over each element of the waypoint bundle 
                for j in range(0, self._subswarm_num_to_assign[i]):
                    max_score = -1 #initialize as -1
                    new_wp_assigned = False
                    # Loop over each waypoint defined in the mission  
                    for k in range(0, len(usma_enums.WP_LOC)):
                        # Skip to next if that waypoint is already assigned
                        if self._wp_assigned[k] == False and self._Known_WP_LOC[k] == -1:
                            # Set the end location to that waypoint
                            temp2_lat = float(usma_enums.WP_LOC[k][0])
                            temp2_lon = float(usma_enums.WP_LOC[k][1])
                            # Check if score of WP is better than max, if so mark it for assignment
                            temp_dist = gps.gps_distance(temp_lat, temp_lon, temp2_lat, temp2_lon)
                            if temp_dist == 0:
                                max_score = 999999999
                                wp_to_assign = k
                                new_wp_assigned = True                            
                            elif self._belief[k]/temp_dist > max_score:
                                max_score = self._belief[k]/temp_dist
                                wp_to_assign = k
                                new_wp_assigned = True

                    # Add the next highest score waypoint to the bundle
                    if new_wp_assigned == True:
                        assignment_list.append(wp_to_assign)
                        self._subswarm_wp_assignment[i] = assignment_list
                        # Mark that waypoint as "assigned" so unavailable to others
                        self._wp_assigned[wp_to_assign] = True
                        # Update the start location to that waypoint
                        temp_lat = float(usma_enums.WP_LOC[wp_to_assign][0])
                        temp_lon = float(usma_enums.WP_LOC[wp_to_assign][1])
                # Assign yourself your own bundle of waypoints
                if self._blue_in_subswarm[i].vehicle_id == self._id:
                    self._wp_id_list = self._subswarm_wp_assignment[i]
            print "subswarm_wp_assignment: ", self._subswarm_wp_assignment
            # Proceed to the first Waypoint in the bundle
            self._loc = self._wp_id_list[0]
            self._wp_id_list_id = 0
            self._desired_lat = float(usma_enums.WP_LOC[self._loc][0])
            self._desired_lon = float(usma_enums.WP_LOC[self._loc][1]) 
            print "Going to wp: ", self._loc

            #Update self._time parameter to reset to determine next time interval
            self._time = timeit.default_timer()

        return True

    #To receive and process messages
    def _process_swarm_data_msg(self, msg):
        if msg.id == pursueBytes.PURSUIT_MESSAGE:
            parser = pursueBytes.PursuitMessageParser()
            parser.unpack(msg.params)

            newly_read_wp = parser.target_id_new
            self._Known_WP_LOC[newly_read_wp] = parser.target_distance
            print parser.target_distance
            self._new_WPs.append(newly_read_wp)
            

#Implementation:
'''
    def countsconvert(rawcounts, absalt, mapalt, radtype):
        background = 0
        rval = 0
        newcounts = 0
        if radtype == "PRDER":
            background = 15.6
            rval = 0.66
        else:
            background = 9.6
            rval = 0.52
        newcounts = rawcounts - background
        if newcounts < 0:
            print("No radiation detected here")
            newcounts = 0
        heightaboveground = absalt - mapalt
        print("heightaboveground: " + str(heightaboveground))
        if heightaboveground < 0:
            print("Net height is negative")
            heightaboveground = 0
        conv = (newcounts * ((heightaboveground)**2) * 4 * math.pi) / rval
        return conv

    def listen():
        global finishedwp
        global log
        global heatmapdata
        global radcap
        #create a TCP/IP socket
        sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    
        #bind the socket to the port. SENSOR STATION IS 203!!
        #192.168.11.202
        serverflag = 1
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
                while True:
                    data=connection.recv(4096)
                    #print("Connected!")
                    #newdata = eval(data)
                    if (len(data) > 1):
                    
                        timeNow = str(datetime.now())
                        newdata = eval(data)
                        
                        print >>sys.stderr, 'Connection from UAS#%s on Port %s' % (newdata[0],client_address[1])
                        print("Working WP List: " + str(newdata[1]))
                        print("Finished WP: " + str(newdata[2]))
                        if (newdata[2] < 10000):                  
                          finishedwp.add(newdata[2])
                        print("Finished WP Set: " + str(finishedwp))
                        print("# of Finished: " + str(len(finishedwp))) + "/" + str(newdata[4])
                        print("At Index: " + str(newdata[3]+1) + "/" + str(newdata[1]))
                        
                        #sendall argument must be string or buffer, not a list
                        print("Sending back a message...")
                        if ((len(finishedwp)) == newdata[4]):
                          print("FINISHED!")
                          quit()
                        #sendbackmsg = [newdata[4],newdata[3]]
                        connection.sendall(str(finishedwp))
                        # Heatmap Portion
                        
                        droneID = newdata[0]
                        lat = float(newdata[5])
                        lon = float(newdata[6])
                        rawcounts = float(newdata[7])
                        absalt = float(newdata[8])
                        radtype = str(newdata[9])
                        print("droneID: " + str(droneID))
                        print("rawcounts: " + str(rawcounts))
                        print("radtype: " + str(radtype))
                        print("absalt: " + str(absalt))
                  
                        if online:
                            mapalt = float(elevationOnline(lat, lon))
                        else:
                            mapalt = float(elevationOffline(lat,lon))
                        convcounts = countsconvert(rawcounts, absalt, mapalt, radtype) 
                        return convcounts

                    else:
                        break
                    
            finally:
                connection.close()
'''
