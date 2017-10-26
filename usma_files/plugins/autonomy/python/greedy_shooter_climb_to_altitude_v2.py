'''
Tactic Name: Greedy Shooter Climb to Altitude v2 (greedy_shooter_climb_to_altitude_v2)
Description:  This tactic is a variation of the GreedyShooterSingle in greedy_shooter_climb_to_altitude_v1.py
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
import ap_lib.sasc_scrimmage as ss
import random

MIN_DIST = 300

class GreedyShooterSingle(ss.Tactic):

    def init(self, params):
        self._id = int(params['id'])
        self._target_id = -1
        self._last_target = None
        self._wp = np.array([0, 0, 0])
        self._max_range = enums.MAX_RANGE_DEF
        self._fov_width = enums.HFOV_DEF
        self._fov_height = enums.VFOV_DEF
        self._own_pose = apbrg.Geodometry()
        self._blues = dict()
        self._reds = dict()
        self._shot = set()
        self._safe_waypoint = np.array([0, 0, 0])
        self._action = ["None", 0]
        self._name = 'PythonGreedyAltitude'
        self._last_update = 0
        self._got_target = False
        self._min_dist = np.inf
        self._reach_alt = 0
        self._descent = 0
        self._tgt_find_count = 0
        self._tracked = []
        self._cur_track = 0


    def step_autonomy(self, t, dt):

        # Reset the action and target ID every loop
        self._action = ["None", 0]
        self._target_id = self._last_target

        # Search for the closest target every frame if you don't already have one
        min_dist = np.inf   # self._min_dist
        if self._got_target is False:
            for uav in self._reds.values():
                if uav.vehicle_id not in self._shot:
                    d = gps.gps_distance(self._own_pose.pose.pose.position.lat,\
                                     self._own_pose.pose.pose.position.lon,\
                                     uav.state.pose.pose.position.lat,\
                                     uav.state.pose.pose.position.lon)


                    if self._tgt_find_count >= 100:
                        self._tracked = []
                        self._tgt_find_count = 0
			print "=======================ID %d reseting targets\n" % (self._id)

                    if d < min_dist:
                        min_dist = d
                        self._target_id = uav.vehicle_id
                        if self._target_id not in self._tracked:
                            self._last_target = self._target_id

                            if d < MIN_DIST:
                                self._got_target = True
                                print "ID: %d locked onto Target ID: %d\n" % (self._id, self._target_id)
                                self._tracked.append(self._target_id)
                                break
                        else:
                            print "ID: %d Found Last Target %d\n" % (self._id, self._tgt_find_count)
                            self._tgt_find_count += 1
                            self._got_target = False
        else:
            self._got_target = False
            self._target_id = None
            for uav in self._reds.values():
                if uav.vehicle_id not in self._shot:
                    if self._last_target == uav.vehicle_id:
                        d = gps.gps_distance(self._own_pose.pose.pose.position.lat,\
                                     self._own_pose.pose.pose.position.lon,\
                                     uav.state.pose.pose.position.lat,\
                                     uav.state.pose.pose.position.lon)
                        if d < MIN_DIST:
                            self._target_id = self._last_target
                            self._got_target = True
                            print "ID: %d tracking Target ID: %d\n" % (self._id, self._target_id)
                        else:
                            print "ID: %d lost Target ID: %d\n" % (self._id, self._last_target)
                            self._got_target = False
                            #self._target_id = None
                            #self._last_target = None

        # If a target has been identified, move towards it
        if self._target_id != None:
	    self._cur_track += 1
            target_pos = self._reds[self._target_id].state.pose.pose.position
            # print "*************************************"
            #print "ID: %d   Target ID: %d\n" % (self._id, self._target_id)

            own_lat = self._own_pose.pose.pose.position.lat
            own_lon = self._own_pose.pose.pose.position.lon
            own_alt = self._own_pose.pose.pose.position.rel_alt
            tgt_lat = target_pos.lat
            tgt_lon = target_pos.lon
            tgt_alt = target_pos.rel_alt

	    dist = gps.gps_distance(own_lat, own_lon, tgt_lat, tgt_lon)

            # Set the waypoint past and above the current target, so we don't go into a
            # loiter mode, and can attack from above
            bearing = gps.gps_bearing(own_lat, own_lon, tgt_lat, tgt_lon)
            if self._cur_track >= 100:
		print "ID: %d swapping target\n" % (self._id)
                self._cur_track = 0
                attack_alt = 250
                self._got_target = False
                lat, lon = gps.gps_newpos(tgt_lat, tgt_lon, bearing, 2000)


            else:
		lat, lon = gps.gps_newpos(tgt_lat, tgt_lon, bearing, 1000)
		if dist < 500:
			attack_alt = tgt_alt
		else:
		        if tgt_alt > self._reach_alt:
		            self._reach_alt = tgt_alt+100

		        if self._descent == 1:
		            attack_alt = own_alt-50
		            if own_alt <= 500:
		                self._descent = 0
		        else:
		            if own_alt >= 1000:
		                attack_alt = self._reach_alt/2
		                self._descent = 1
		            elif own_alt < self._reach_alt:
		                attack_alt = own_alt+100
		            else:
		                attack_alt = own_alt-50

            self._wp = np.array([lat+(.00002*self._id) , lon+(.00002*self._id), attack_alt ])  # np.array([lat, lon, target_pos.rel_alt])

            # Determine if the target is within firing parameters
            if gps.hitable(self._own_pose.pose.pose.position.lat, \
                           self._own_pose.pose.pose.position.lon, \
                           self._own_pose.pose.pose.position.rel_alt, \
                           (self._own_pose.pose.pose.orientation.x, \
                            self._own_pose.pose.pose.orientation.y, \
                            self._own_pose.pose.pose.orientation.z, \
                            self._own_pose.pose.pose.orientation.w ), \
                           self._max_range, self._fov_width, self._fov_height,\
                           target_pos.lat, target_pos.lon, target_pos.rel_alt):
                self._action = ["Fire", self._target_id]
                print "GS Single==========================>ID: %d shot at Target %d" % (self._id, self._target_id)
                #self._last_target = None
                #self._got_target = False

        else:
            # Start at the safe waypoint
            self._wp = self._safe_waypoint

        return True
