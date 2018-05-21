import numpy as np
import autopilot_bridge.msg as apbrg
import ap_msgs.msg as apmsg
from autopilot_bridge.msg import LLA
import ap_lib.gps_utils as gps
import ap_lib.ap_enumerations as enums
import ap_lib.sasc_scrimmage as ss

class TutorialGreedyShooter(ss.Tactic):

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
        self._name = 'TutorialGreedyShooter'

    def step_autonomy(self, t, dt):

        # Reset the action and target ID every loop
        self._action = ["None", 0]
        self._target_id = None

        # Search for the closest target every frame
        min_dist = np.inf
        for uav in self._reds.values():
            if uav.vehicle_id not in self._shot:
                # print uav.vehicle_id,
                d = gps.gps_distance(self._own_pose.pose.pose.position.lat,\
                                     self._own_pose.pose.pose.position.lon,\
                                     uav.state.pose.pose.position.lat,\
                                     uav.state.pose.pose.position.lon)

                if d < min_dist:
                    min_dist = d
                    self._target_id = uav.vehicle_id
        
        #print self._blues.values()

        # If a target has been identified, move towards it
        if self._target_id != None:
            #print self._reds[self._target_id]
            target_pos = self._reds[self._target_id].state.pose.pose.position
            # s = ", ".join(str(e) for e in self._shot)
            # print "Reds Shot: "
            # print s
            #print "ID: %d   Target ID: %d   Dist: %d" % (self._id, self._target_id, min_dist)
            own_lat = self._own_pose.pose.pose.position.lat
            own_lon = self._own_pose.pose.pose.position.lon
            tgt_lat = target_pos.lat
            tgt_lon = target_pos.lon

            # Set the waypoint past the current target, so we don't go into a
            # loiter mode
            bearing = gps.gps_bearing(own_lat, own_lon, tgt_lat, tgt_lon)
            lat, lon = gps.gps_newpos(own_lat, own_lon, bearing, 1000)

            self._wp = np.array([lat, lon, target_pos.rel_alt])

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
                #print "=====>ID: %d shot at Target %d" % (self._id, self._target_id)
        else:
            # If a target hasn't been identified, return to the safe waypoint
            self._wp = self._safe_waypoint
        return True
