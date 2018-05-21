'''
Tactic Name: Greedy Shooter All (GreedyShooterAll)
Description:  This tactic is a variation of the HelloGreedyShooter in hello_greedy_shooter.py
              The tactic evaluates all enemy as it finds the closest one and runs the "hittable"
                algorithm to see if any are in the weapons fan.  The purpose of this is to avoid
                being myoptic on the closest enemy and shoot the first available, presumably before
                they shoot back.
Date:		12 Apr 2017
Changes:	
Authors:	USMA SASC Team
'''
import numpy as np
import math
import autopilot_bridge.msg as apbrg
import ap_msgs.msg as apmsg
from autopilot_bridge.msg import LLA
import ap_lib.gps_utils as gps
import ap_lib.ap_enumerations as enums
import ap_lib.sasc_scrimmage as ss

class GreedyShooterAll(ss.Tactic):

    def init(self, params):
        self._id = int(params['id'])
        self._team_id = int(params['team_id'])
        self._target_id = -1
        self._last_target_id = 0
        self._target_count = 0
        self._target_dist = float(params['target_dist'])
        self._max_count = int(params['max_count'])
        self._chase_safe = float(params['chase_safe'])
        self._min_dist = float(params['min_dist'])
        self._wp = np.array([0, 0, 0])
        self._max_range = enums.MAX_RANGE_DEF
        self._fov_width = enums.HFOV_DEF
        self._fov_height = enums.VFOV_DEF
        self._own_pose = apbrg.Geodometry()
        self._blues = dict()
        self._reds = dict()
        self._shot = set()
        self._safe_waypoint = np.array([0, 0, 0])
        self._last_ap_wp = np.array([0, 0, 0])
        self._action = ["None", 0]
        self._name = 'GreedyShooterAll'
        try:
            self._vehicle_type = int(params['vehicle_type'])
        except KeyError:
            self._vehicle_type = enums.AC_FIXED_WING            

    def step_autonomy(self, t, dt):

        # Reset the action and target ID every loop
        self._action = ["None", 0]
        self._target_id = None

        # Search for the closest target every frame
        min_dist = self._min_dist      # np.inf

        own_pos = self._own_pose.pose.pose.position
        own_orientation = self._own_pose.pose.pose.orientation
        own_lat = own_pos.lat
        own_lon = own_pos.lon
        own_alt = own_pos.rel_alt

        for uav in self._reds.values():
            if uav.vehicle_id not in self._shot and (uav.game_status == enums.GAME_ACTIVE_DEFENSE or uav.game_status == enums.GAME_ACTIVE_OFFENSE):
                target_pos = self._reds[uav.vehicle_id].state.pose.pose.position
                tgt_lat = target_pos.lat
                tgt_lon = target_pos.lon
                tgt_alt = target_pos.rel_alt
                d = gps.gps_distance(own_lat, own_lon, tgt_lat, tgt_lon)

                if d <= self._max_range:
                    if gps.hitable(own_lat, own_lon, own_alt, \
                           (own_orientation.x, own_orientation.y, \
                           own_orientation.z, own_orientation.w ), \
                           self._max_range, self._fov_width, self._fov_height,\
                           tgt_lat, tgt_lon, tgt_alt):
                        self._action = ["Fire", uav.vehicle_id]
                        print 'GSA========(in loop)===========> ID: {} shot at UAV: {}'.format(self._id, uav.vehicle_id)
                        #self._action = ["None", 0]
                if d < min_dist:
                    min_dist = d
                    self._target_id = uav.vehicle_id

        if self._target_id in self._shot:
            self._target_id = None

        if self._last_target_id == self._target_id:
            total_count = self._max_count*(1 + self._chase_safe)/self._chase_safe
            if d < self._target_dist:  # how close do we have to be before I call chasing
                self._target_count += 1
                if self._target_count < self._max_count:
                    pass
                    #print 'ID: {} chasing UAV: {} for {} iterations'.format(self._id, self._target_id, self._target_count)
                elif self._target_count < total_count:
                    pass
                    #print 'ID: {} going to safe wp for {} iterations'.format(self._id, total_count - self._target_count)
                else:
                    self._target_count = 0
                    self._target_id = None

        # If a target has been identified, move towards it
        if self._target_id != None and self._target_count < self._max_count:
            self._last_target_id = self._target_id
            target_pos = self._reds[self._target_id].state.pose.pose.position
            tgt_lat = target_pos.lat
            tgt_lon = target_pos.lon
            tgt_alt = target_pos.rel_alt
            
            lat = tgt_lat
            lon = tgt_lon

            if self._vehicle_type == enums.AC_FIXED_WING:
                # When using fixed wing, Set the waypoint past the current
                # target, so we don't go into a loiter mode
                bearing = gps.gps_bearing(own_lat, own_lon, tgt_lat, tgt_lon)
                lat, lon = gps.gps_newpos(own_lat, own_lon, bearing, 1000)

            self._wp = np.array([lat, lon, target_pos.rel_alt])

            # Determine if the target is within firing parameters
            if gps.hitable(own_lat, own_lon, own_alt, \
                   (own_orientation.x, own_orientation.y, \
                   own_orientation.z, own_orientation.w ), \
                   self._max_range, self._fov_width, self._fov_height,\
                   tgt_lat, tgt_lon, tgt_alt):
                self._action = ["Fire", self._target_id]
                print 'GSA============================> ID: {} shot at UAV: {}'.format(self._id, self._target_id)
        else:
            # If a target hasn't been identified, return to the safe waypoint
            self._wp = self._safe_waypoint

        # Maintain altitude
        self._wp[2] = self._last_ap_wp[2]
            
        return True

    def receive_firing_report(self, msg):
        print '{} received that someone is targeting {}' \
              .format(self._id, msg.report.target_id)
