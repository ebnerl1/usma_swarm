import numpy as np
import math
import autopilot_bridge.msg as apbrg
import ap_msgs.msg as apmsg
from autopilot_bridge.msg import LLA
import ap_lib.gps_utils as gps
import ap_lib.ap_enumerations as enums
import ap_lib.sasc_scrimmage as ss
import usma_enumerations as usma_enums

class GotoMult(ss.Tactic):

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
        self._name = 'GotoMult'
        self._wp_dist = float(params['wp_dist']) # sets the distance you have to get to desired wp before going to next
        self._loc = []
        self._loc.append(int(params['loc1']))    # always contains a valid location number
        self._loc.append(int(params['loc2']))    # may contain a location number, or other meaning
        self._loc.append(int(params['loc3']))    # e.g. -1 means repeat path from loc1
        self._loc.append(int(params['loc4']))    # e.g. 99 means orbit at last location forever
        self._desired_lat = float(usma_enums.WP_LOC[self._loc[0]][0])
        self._desired_lon = float(usma_enums.WP_LOC[self._loc[0]][1])
        self._step = 0      # start at loc1 or loc[0]
        
    def step_autonomy(self, t, dt):
        # Go to desired latitude, longitude, and maintain altitude
        # deconfliction:
       
        d = gps.gps_distance(self._own_pose.pose.pose.position.lat, self._own_pose.pose.pose.position.lon, self._desired_lat, self._desired_lon)

        if d < self._wp_dist:
            if self._step < 3:
                if (self._loc[self._step + 1]) in range(0,16): # wp 0-15
                    self._step += 1
                elif (self._loc[self._step + 1]) < 0:  # -1
                    self._step = 0
                elif (self._loc[self._step + 1]) > 16:
                    pass        # stay at current waypoint forever
            else:
                self._step = 0  # go back to beginning
        
        self._desired_lat = float(usma_enums.WP_LOC[self._loc[self._step]][0])
        self._desired_lon = float(usma_enums.WP_LOC[self._loc[self._step]][1])

        self._wp = np.array([self._desired_lat, self._desired_lon, self._last_ap_wp[2]])

        return True
