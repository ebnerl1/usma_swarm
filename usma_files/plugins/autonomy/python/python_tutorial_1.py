import numpy as np
import math
import autopilot_bridge.msg as apbrg
import ap_msgs.msg as apmsg
from autopilot_bridge.msg import LLA
import ap_lib.gps_utils as gps
import ap_lib.ap_enumerations as enums
import sasc_scrimmage

class HelloTactic(sasc_scrimmage.Tactic):
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

    def step_autonomy(self, t, dt):
        # Reset the action and target ID every loop
        self._action = ["None", 0]
        self._target_id = None

        # Remain at the safe waypoint
        self._wp = self._safe_waypoint

        return True
