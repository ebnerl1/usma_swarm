'''
Tactic Name: Greedy Shooter Interceptor (GreedyShooterInt)
Description:  This tactic is a variation of the TutorialGreedyShooter in tutorial_greedy_shooter_1.py
              The tactic calculates the intercept point between the self and it's designated target at
              every interation.  It does this by calculating the time of intercept based on the target
              heading and speed, along with its own heading and speed.  It adjusts its bearing to meet the intercept
              If an intercept solution is not found (e.g. pointed the wrong way and time to intercept is negative),
              it will calculate a new bearing as is done in TutorialGreedyShooter.
Date:		7 Dec 2016
Changes:	23 Jan 2017: Changed "_reds_shot" to "_shot"
Authors:	USMA SASC Team
'''

import numpy as np
import autopilot_bridge.msg as apbrg
import ap_msgs.msg as apmsg
from autopilot_bridge.msg import LLA
import ap_lib.gps_utils as gps
import ap_lib.ap_enumerations as enums
import math
import ap_lib.sasc_scrimmage as ss

class GreedyShooterIntMsg(ss.Tactic):

    def init(self, params):
        self._id = int(params['id'])
        self._last_lat = None
        self._last_lon = None
        self._speed = None
        self._heading = None
        self._target_id = None
        self._target_count = 0
        self._target_dist = np.inf
        self._target_range = np.inf     # added to keep track of current target range
        self._target_last_lat = None
        self._target_last_lon = None
        self._target_heading = None
        self._target_speed = None
        self._taken = set()     # this is the set that this UAV will use to keep track of "taken" enemy
        self._wp = np.array([0, 0, 0])
        self._max_range = enums.MAX_RANGE_DEF
        self._fov_width = enums.HFOV_DEF
        self._fov_height = enums.VFOV_DEF
        self._own_pose = apbrg.Geodometry()
        self._blues = dict()
        self._reds = dict()
        self._shot = set()				# changed from _reds_shot
        self._safe_waypoint = np.array([0, 0, 200])
        self._action = ["None", 0]
        self._name = 'GreedyShooterInt'

        # This is the new messaging logic (from the documentation)
        self.pubs['my_topic'] = self.createPublisher('my_topic', apmsg.MsgStat, 1)
        self.subs['my_topic'] = self.createSubscriber('my_topic', apmsg.MsgStat, self.receive_msg_stat)

    def receive_msg_stat(self, msg):
        #print '{} received message from {} with latency {}'.format(self._id, msg.id, msg.latency)
        print '{} received message from {}:  Targeting {} at distance {}'.format(self._id, msg.id, msg.count, msg.latency)
        self._taken.add(msg.count)
        if msg.count == self._target_id:
            if msg.latency < self._target_range:
                self._target_id = None
                self._target_range = np.inf
                print '{} dropped UAV {} due to {} being closer at distance {}'.format(self._id, msg.count, msg.id, msg.latency)
            else:
                self._taken.discard(msg.count)  # remove from my list if I am close

    def step_autonomy(self, t, dt):

        # Reset the action and target ID every loop
        self._action = ["None", 0]

        msg = apmsg.MsgStat()
        msg.id = self._id

############################################################
# If self._target_id is not None and in the shot group
# then reset to None, otherwise set to last_target
############################################################

        min_dist = self._target_dist
        for uav in self._reds.values():
            if uav.vehicle_id not in self._shot:
                if uav.vehicle_id not in self._taken:
                    uav_lat = uav.state.pose.pose.position.lat
                    uav_lon = uav.state.pose.pose.position.lon
                    uav_alt = uav.state.pose.pose.position.rel_alt
                    d = gps.gps_distance(self._own_pose.pose.pose.position.lat,\
                                     self._own_pose.pose.pose.position.lon,\
                                     uav_lat, uav_lon)

                    if d < min_dist:
                        min_dist = d
                        self._target_id = uav.vehicle_id
                        self._target_range = d


        if self._target_id in self._shot:
            self._target_id = None
            self._target_dist = np.inf
            self._target_count = 0
            self._target_last_lat = None
            self._target_last_lon = None
            self._target_heading = None
            self._target_speed = None

        # If a target has been identified, move towards it
        if self._target_id != None:
            msg.count = self._target_id
            msg.latency = self._target_range
            self.pubs['my_topic'].publish(msg)
            print '{} targeting UAV {} at a distance of {}'.format(self._id, msg.count, msg.latency)
            print '{} has taken list of {}'.format(self._id, self._taken)

            own_lat = self._own_pose.pose.pose.position.lat
            own_lon = self._own_pose.pose.pose.position.lon

            if self._last_lat is not None:
                self._speed = gps.gps_distance(own_lat, own_lon,\
                                     self._last_lat, self._last_lon) / 0.1
                self._heading = gps.gps_bearing(self._last_lat, self._last_lon, own_lat, own_lon)

            self._last_lat = own_lat
            self._last_lon = own_lon

            target_pos = self._reds[self._target_id].state.pose.pose.position
            tgt_lat = target_pos.lat
            tgt_lon = target_pos.lon
            tgt_alt = target_pos.rel_alt

            d = gps.gps_distance(self._own_pose.pose.pose.position.lat,\
                                     self._own_pose.pose.pose.position.lon,\
                                     tgt_lat, tgt_lon)

            if self._target_last_lat is not None:
                self._target_speed = gps.gps_distance(tgt_lat, tgt_lon,\
                                     self._target_last_lat, self._target_last_lon) / 0.1
                self._target_heading = gps.gps_bearing(self._target_last_lat, self._target_last_lon, tgt_lat, tgt_lon)
                #print "ID: %d   Target: %d" % (self._id, self._target_id)
                #print "Lat: %f  Lon: %f  Head: %f   T_lat: %f  T_lon %f  T_head: %f" % (own_lat, own_lon, self._heading, \
                    #tgt_lat, tgt_lon, self._target_heading)

            self._target_last_lat = tgt_lat
            self._target_last_lon = tgt_lon

#############################################################################
# Calc intercept point
# Reference:  http://jaran.de/goodbits/2011/07/17/calculating-an-intercept-course-to-a-target-with-constant-direction-and-velocity-in-a-2-dimensional-plane/
# Translated from Java to Python
#############################################################################
            t = None
            if self._target_heading is not None: 
                s = self._speed
                ox = tgt_lat - own_lat
                oy = tgt_lon - own_lon
                vx = self._target_speed * math.cos(self._target_heading) # sin
                vy = self._target_speed * math.sin(self._target_heading) # cos
                h1 = vx**2 + vy**2 - s**2
                h2 = ox*vx + oy*vy

                if h1 == 0:
                    t = -(ox**2 + oy**2)/(2*h2)
                else:
                    minusPHalf = -h2/h1
                    disc = minusPHalf**2 - (ox**2 + oy**2)/h1
                    if disc < 0:
                        t = None
                    else:
                        root = math.sqrt(disc)
                        t1 = minusPHalf + root
                        t2 = minusPHalf - root
                        tmin = t1 if t1 < t2 else t2
                        tmax = t1 if t1 > t2 else t2

                        t = tmin if tmin > 0 else tmax

                        if t < 0 or d < self._max_range:  # close enough so go right to him (switch to greedy shooter)
                            t = None
                        else:    
                            t *= 1.53

            if t is not None:
                int_lat = tgt_lat + vx * t
                int_lon = tgt_lon + vy * t
                int_dist = gps.gps_distance(own_lat, own_lon, int_lat, int_lon)
                bearing = gps.gps_bearing(own_lat, own_lon, int_lat, int_lon)
                lat, lon = gps.gps_newpos(own_lat, own_lon, bearing, int_dist)
                #print "ID: %d using intercept bearing %f to Lat: %f  Lon: %f  Dist: %f  Time: %f" % (self._id, bearing, lat, lon, int_dist, t*10000)  #  I_lat: %f  I_lon: %f  t: %f
            else:
                bearing = gps.gps_bearing(own_lat, own_lon, tgt_lat, tgt_lon)
                dist = gps.gps_distance(own_lat, own_lon, tgt_lat, tgt_lon)
                lat, lon = gps.gps_newpos(own_lat, own_lon, bearing, dist+250)   # range was 1000
                #print "ID: %d using direct bearing %f to Lat: %f  Lon: %f  Dist: %f" % (self._id, bearing, lat, lon, dist)

            # Set the waypoint past the current target, so we don't go into a loiter mode

            self._wp = np.array([lat, lon, tgt_alt])

            # Determine if the target is within firing parameters
            if gps.hitable(self._own_pose.pose.pose.position.lat, \
                           self._own_pose.pose.pose.position.lon, \
                           self._own_pose.pose.pose.position.rel_alt, \
                           (self._own_pose.pose.pose.orientation.x, \
                            self._own_pose.pose.pose.orientation.y, \
                            self._own_pose.pose.pose.orientation.z, \
                            self._own_pose.pose.pose.orientation.w ), \
                           self._max_range, self._fov_width, self._fov_height,\
                           tgt_lat, tgt_lon, tgt_alt):
                self._action = ["Fire", self._target_id]
                #print "GS Int=============================>ID: %d shot at Target %d" % (self._id, self._target_id)

        else:
            # If a target hasn't been identified, return to the safe waypoint
            self._wp = self._safe_waypoint
        self._taken.clear()         # clear the taken list in case it's full of old data
        return True
