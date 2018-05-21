import numpy as np
import math
import autopilot_bridge.msg as apbrg
import ap_msgs.msg as apmsg
from autopilot_bridge.msg import LLA
import ap_lib.gps_utils as gps
import ap_lib.ap_enumerations as enums
import ap_lib.sasc_scrimmage as ss
import rospy


class Vortex(ss.Tactic):
    def init(self, params):
        self._id = int(params['id'])
        self._target_id = -1
        self._wp = np.array([0, 0, 0])
        self._weapons_range = enums.MAX_RANGE_DEF
        self._fov_width = enums.HFOV_DEF
        self._fov_height = enums.VFOV_DEF
        self._bboxes = enums.get_battleboxes()
        self._own_pose = apbrg.Geodometry()
        self._blues = dict()
        self._reds = dict()
        self._shot = set()
        self._safe_waypoint = np.array([0, 0, 0])
        self._action = ["None", 0]
        self._name = 'Vortex'


    def step_autonomy(self, t, dt):

        # Reset the action every loop
        self._action = ["None", 0]
        self._target_id = None

        own_lat = self._own_pose.pose.pose.position.lat
        own_lon = self._own_pose.pose.pose.position.lon
        own_alt = self._own_pose.pose.pose.position.alt


        # Search for the closest target
        min_dist = np.inf
        for uav in self._reds.values():
            if uav.vehicle_id not in self._shot:
                d = gps.gps_distance(own_lat, \
                                     own_lon, \
                                     uav.state.pose.pose.position.lat, \
                                     uav.state.pose.pose.position.lon)

                if d < min_dist:
                    min_dist = d
                    self._target_id = uav.vehicle_id

        # If a target has been identified, move towards it
        if self._target_id != None:
            target_pos = self._reds[self._target_id].state.pose.pose.position

            tgt_lat = target_pos.lat
            tgt_lon = target_pos.lon

            distance_to_target = gps.gps_distance(own_lat, own_lon, tgt_lat, tgt_lon)

            # Set bearing to look at the enemy vehicle
            bearing = gps.gps_bearing(own_lat, own_lon, tgt_lat, tgt_lon)

            # Set waypoint to move forward and strafe sideways once the enemy is within range
            self._evasion_range = self._weapons_range * 2
            self._engagement_range = self._weapons_range * 2 / 3
            if distance_to_target <= self._engagement_range:
                lat, lon = gps.gps_newpos(own_lat, own_lon, gps.normalize(bearing - self._fov_width/2), 0.1)
            elif distance_to_target <= self._evasion_range:
                # Check to see if sidestep right waypoint is outside the battlebox
                # (only need to check distance_to_target/2 since enemy is closing)
                lat_right, lon_right = gps.gps_newpos(own_lat, own_lon, \
                                                      gps.normalize(bearing + (math.pi / 3)), distance_to_target/2)
                if self._bboxes[0].contains(lat_right, lon_right, own_alt):
                    lat, lon = gps.gps_newpos(own_lat, own_lon, \
                                                      gps.normalize(bearing + (math.pi / 3)), distance_to_target)
                else:
                    # If right sidestep is outside the battlebox, sidestep to the left
                    lat, lon = gps.gps_newpos(own_lat, own_lon, gps.normalize(bearing - (math.pi / 3)), distance_to_target)
            else:
                lat, lon = gps.gps_newpos(own_lat, own_lon, bearing, 1000)

            self._wp = np.array([lat, lon, target_pos.rel_alt])

            # Determine if the target is within firing parameters
            if gps.hitable(self._own_pose.pose.pose.position.lat, \
                           self._own_pose.pose.pose.position.lon, \
                           self._own_pose.pose.pose.position.rel_alt, \
                           (self._own_pose.pose.pose.orientation.x, \
                            self._own_pose.pose.pose.orientation.y, \
                            self._own_pose.pose.pose.orientation.z, \
                            self._own_pose.pose.pose.orientation.w), \
                           self._weapons_range, self._fov_width, self._fov_height, \
                           target_pos.lat, target_pos.lon, target_pos.rel_alt):
                self._action = ["Fire", self._target_id]
        else:
            # If a target hasn't been identified, return to the safe waypoint
            self._wp = self._safe_waypoint

        return True

    def receive_firing_report(self, msg):
        print '{} received that someone is targeting {}' \
            .format(self._id, msg.report.target_id)
