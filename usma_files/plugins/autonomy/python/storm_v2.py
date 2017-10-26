import numpy as np
import math
import autopilot_bridge.msg as apbrg
import ap_msgs.msg as apmsg
from autopilot_bridge.msg import LLA
import ap_lib.gps_utils as gps
import ap_lib.ap_enumerations as enums
import ap_lib.sasc_scrimmage as ss
import random

MIN_DIST = 300

#swarmCount = 0

class StormShooter(ss.Tactic):

    def init(self, params):
        self._id = int(params['id'])
        self._target_id = -1
        self._speed = None
        self._heading = None
        self._target_count = 0
        self._last_target = None
        self._target_dist = np.inf
        self._target_heading = None
        self._target_speed = None
        self._wp = np.array([0, 0, 0])
        self._goal = np.array([0, 0, 0])
        self._max_range = enums.MAX_RANGE_DEF
        self._fov_width = enums.HFOV_DEF
        self._fov_height = enums.VFOV_DEF
        self._own_pose = apbrg.Geodometry()
        self._blues = dict()
        self._reds = dict()
        self._shot = set()
	    #self._safe_waypoint_reserve = np.array([250, 125, 250])
        #self._safe_waypoint_nw = np.array([125, 375, 250])
        #self._safe_waypoint_sw = np.array([125, 125, 300])
        #self._safe_waypoint_ne = np.array([375, 375, 200])
        #self._safe_waypoint_se = np.array([375, 125, 350])
        self._action = ["None", 0]
        self._name = 'StormShooter'
        self._got_target = False
        #self._in_swarm = False
        self._team = None
        #self._min_dist = np.inf
        self._goal_lat = None
        self._goal_lon = None

	

    def step_autonomy(self, t, dt):
        
        #swarmCount = 0
        # Reset the action and target ID every loop
        self._action = ["None", 0]
        #self._target_id = self._last_target

        if self._team is None:
            if self._id in self._blues:
                self._team = 'blue'
            else:
                self._team = 'red'
        
            (self._goal_lat, self._goal_lon, _) = enums.GOAL_POSITS[self._team]
            print "lat %f lon %f" % (self._goal_lat, self._goal_lon)
        
        #go to waypoints for storm formation
        
        if self._got_target is False and self._goal_lat is not None:
            #self._in_swarm = True
            if self._id%5 == 0:
                storm_lat, storm_lon = gps.gps_newpos(self._goal_lat, self._goal_lon, math.radians(enums.BATTLE_CUBE_ORIENT) + math.pi/2.0, 0)     
                self._wp = np.array([storm_lat, storm_lon, 500])
                #self._safe_waypoint_reserve 35.720785, -120.765860
                print "ID: %d sent RESERVE %f, %f\n" % (self._id, storm_lat, storm_lon)

            if self._id%5 == 1:
                storm_lat, storm_lon = gps.gps_newpos(self._goal_lat, self._goal_lon, math.radians(enums.BATTLE_CUBE_ORIENT), 100)
                self._wp = np.array([storm_lat, storm_lon, 500])
                #self._wp = np.array([125, 375, 250])#self._safe_waypoint_nw 35.720785, -120.765860
                print "ID: %d sent NW %f, %f\n" % (self._id, storm_lat, storm_lon)

            if self._id%5 == 2:
                storm_lat, storm_lon = gps.gps_newpos(self._goal_lat, self._goal_lon, math.radians(enums.BATTLE_CUBE_ORIENT) + math.pi, 100)
                self._wp = np.array([storm_lat, storm_lon, 500])
                #self._wp = np.array([125, 125, 300])#self._safe_waypoint_sw 35.719981, -120.769101
                print "ID: %d sent SW %f, %f\n" % (self._id, storm_lat, storm_lon)

            if self._id%5 == 3:
                storm_lat, storm_lon = gps.gps_newpos(self._goal_lat, self._goal_lon, math.radians(enums.BATTLE_CUBE_ORIENT) + math.pi/4.0, 100)
                self._wp = np.array([storm_lat, storm_lon, 500])
                #self._wp = np.array([375, 375, 200])#self._safe_waypoint_ne 35.720785, -120.765860
                print "ID: %d sent NE %f, %f\n" % (self._id, storm_lat, storm_lon)

            if self._id%5 == 4:
                storm_lat, storm_lon = gps.gps_newpos(self._goal_lat, self._goal_lon, math.radians(enums.BATTLE_CUBE_ORIENT) + 3.0*math.pi/4.0, 100)
                self._wp = np.array([storm_lat, storm_lon, 500])
                #self._wp = np.array([375, 125, 350])#self._safe_waypoint_se 35.719981, -120.769101
                print "ID: %d sent SE %f, %f\n" % (self._id, storm_lat, storm_lon)   
     

        # Search for the closest target every frame if you don't already have one
        min_dist = self._target_dist   # self._min_dist
        if self._got_target is False:
            for uav in self._reds.values():
                if uav.vehicle_id not in self._shot:
                    d = gps.gps_distance(self._own_pose.pose.pose.position.lat,\
                                     self._own_pose.pose.pose.position.lon,\
                                     uav.state.pose.pose.position.lat,\
                                     uav.state.pose.pose.position.lon)

                    if d < min_dist:
                        min_dist = d
                        self._target_id = uav.vehicle_id
                        self._last_target = self._target_id
                        
                        if d < self._max_range:
                            self._got_target = True
                            self._in_swarm = False
                            print "UAV %d within range of Target %d\n" % (self._id, self._target_id)
                            break
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
                        if d < self._max_range:
                            self._target_id = self._last_target
                            if self._got_target is False:   # only print once
                                print "UAV %d within range of Target %d\n" % (self._id, self._target_id)
                                self._got_target = True

                        else:   # was a match, but d > MIN_DIST, reset all
                            print "UAV %d out of range of Target %d\n" % (self._id, self._last_target)
                            self._got_target = False
                            self._target_id = None
                            self._last_target = None

      # If a target has been identified, move towards it
        if self._target_id != None:
            target_pos = self._reds[self._target_id].state.pose.pose.position
            #print "*************************************"
            #print "ID: %d   Target ID: %d\n" % (self._id, self._target_id)
            
            own_lat = self._own_pose.pose.pose.position.lat
            own_lon = self._own_pose.pose.pose.position.lon
            tgt_lat = target_pos.lat
            tgt_lon = target_pos.lon
            tgt_alt = target_pos.rel_alt

            # Set the waypoint past the current target, so we don't go into a
            # loiter mode
            if self._got_target is True:
                bearing = gps.gps_bearing(own_lat, own_lon, tgt_lat, tgt_lon)
                lat, lon = gps.gps_newpos(own_lat, own_lon, bearing, 1000)
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
                           target_pos.lat, target_pos.lon, target_pos.rel_alt):
                self._action = ["Fire", self._target_id]
                #self._action = ["None", 0]
                print "Storm Shooter==========================>UAV %d shot at Target %d" % (self._id, self._target_id)
                #self._last_target = None
                #self._got_target = False

        #else:
            # Start at the safe waypoint
            #self._wp = self._safe_waypoint

        return True
