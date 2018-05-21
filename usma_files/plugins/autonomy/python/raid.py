'''
Tactic Name: Raid (Raid)
Description:  This tactic is an offensive tactic that sends the UASs to a target location and then return
                to a home location. It uses the waypoints that are defined in usma_enumerations.py
                that form at 4x4 grid with WP 0 at the NW corner of the battlecube, WP 3 at the SW corner,
                and continuing until WP 15 at the SE corner of the battlecube.

                |===================|
                |   0   4   8   12  |
                |   1   5   9   13  |
                |   2   6  10   14  |
                |   3   7  11   15  |
                |===================|

Date:		27 Apr 2017
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
import usma_enumerations as usma_enums

class Raid(ss.Tactic):

    def init(self, params):
        self._id = int(params['id'])
        self._target_id = -1
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
        self._vehicle_type = int(params['vehicle_type'])
        self._name = 'Raid'
        self._home = int(params['home'])
        self._desired_lat = float(usma_enums.WP_LOC[self._home][0])
        self._desired_lon = float(usma_enums.WP_LOC[self._home][1])
        self._tloc = int(params['target'])
        self._tlat = float(usma_enums.WP_LOC[self._tloc][0])
        self._tlon = float(usma_enums.WP_LOC[self._tloc][1]) 
        self._pass = 0       
        
    def step_autonomy(self, t, dt):
        self._target_id = -1
        self._action = ["None", 0]
        self._pass += 1

        if self._pass <= 400:
          self._wp = np.array([self._tlat, self._tlon, self._last_ap_wp[2]])

        elif self._pass < 600 and self._pass > 400:
          self._wp = np.array([self._desired_lat, self._desired_lon, self._last_ap_wp[2]])

        else:
          self._pass = 0
          
        # Fire at any reds that are within firing parameters
        for target_id, red in self._reds.iteritems():
          if (red.game_status == enums.GAME_ACTIVE_DEFENSE or red.game_status == enums.GAME_ACTIVE_OFFENSE):
            if gps.hitable(self._own_pose.pose.pose.position.lat, \
                           self._own_pose.pose.pose.position.lon, \
                           self._own_pose.pose.pose.position.rel_alt, \
                           (self._own_pose.pose.pose.orientation.x, \
                           self._own_pose.pose.pose.orientation.y, \
                           self._own_pose.pose.pose.orientation.z, \
                           self._own_pose.pose.pose.orientation.w ), \
                           self._max_range, self._fov_width, self._fov_height,\
                           red.state.pose.pose.position.lat, 
                           red.state.pose.pose.position.lon, 
                           red.state.pose.pose.position.rel_alt):
              self._target_id = target_id
              self._action = ["Fire", self._target_id]
              return True

        return True


