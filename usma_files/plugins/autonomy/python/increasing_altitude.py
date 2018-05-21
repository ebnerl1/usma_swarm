import numpy as np
import math
import autopilot_bridge.msg as apbrg
import ap_msgs.msg as apmsg
from autopilot_bridge.msg import LLA
import ap_lib.gps_utils as gps
import ap_lib.ap_enumerations as enums
import ap_lib.sasc_scrimmage as ss

class IncreasingAltitude(ss.Tactic):

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
        self._action = ["None", 0]
        self._name = 'HelloGreedyShooter'        
        self._last_update = 0

    def step_autonomy(self, t, dt):                        
        # Reset the action and target ID every loop
        self._action = ["None", 0]                        
        self._target_id = None
        
        if t < 10:
            # Start at the safe waypoint
            self._wp = self._safe_waypoint
            
        elif t > (self._last_update + 1):
            # Slowly increment the waypoint's altitude
            self._wp[2] += 5
            self._last_update = t
        
        return True
