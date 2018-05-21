import numpy as np
import math
import autopilot_bridge.msg as apbrg
import ap_msgs.msg as apmsg
from autopilot_bridge.msg import LLA
import ap_lib.gps_utils as gps
import ap_lib.ap_enumerations as enums
import ap_lib.sasc_scrimmage as ss
import usma_enumerations as usma_enums

class Goto2(ss.Tactic):

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
        self._name = 'Goto2'
        self._location = int(params['location'])
        self._desired_lat = float(usma_enums.WP_LOC[self._location][0])
        self._desired_lon = float(usma_enums.WP_LOC[self._location][1])
        
    def step_autonomy(self, t, dt):
        # Go to desired latitude, longitude, and maintain altitude
        # deconfliction:
        self._wp = np.array([self._desired_lat, self._desired_lon, 
                             self._last_ap_wp[2]])
        
        return True
