import numpy as np
import math
import autopilot_bridge.msg as apbrg
import ap_msgs.msg as apmsg
from autopilot_bridge.msg import LLA
import ap_lib.gps_utils as gps
import ap_lib.ap_enumerations as enums
import ap_lib.sasc_scrimmage as ss

class GotoOrbit(ss.Tactic):

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
        self._name = 'GotoOrbit'
        self._desired_lat = float(params['lat'])
        self._desired_lon = float(params['lon'])
        
    def step_autonomy(self, t, dt):
        # Go to desired latitude, longitude, and maintain altitude
        # deconfliction:
        
        own_pos = self._own_pose.pose.pose.position
        own_lat = own_pos.lat
        own_lon = own_pos.lon
        own_alt = own_pos.rel_alt
        own_orientation = self._own_pose.pose.pose.orientation

        # Calculate bearing to target and from target to self
        bearing = gps.normalize(gps.gps_bearing(own_lat, own_lon, self._desired_lat, self._desired_lon))
        lat = self._desired_lat
        lon = self._desired_lon
        dist = gps.gps_distance(own_lat, own_lon, self._desired_lat, self._desired_lon)
        bearing_adj = 0.0
        if dist > 100:
            bearing_adj = math.pi/4 * (-1 + self._id%2 * 2)
            lat, lon = gps.gps_newpos(own_lat, own_lon, bearing + bearing_adj, 1000)
            if bearing_adj > 0:
                print 'ID: {} orbiting CW'.format(self._id)
            else:
                print 'ID: {} orbiting CCW'.format(self._id)

        self._wp = np.array([lat, lon, self._last_ap_wp[2]])
        
        return True
