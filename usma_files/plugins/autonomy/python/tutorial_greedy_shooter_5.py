import numpy as np
import autopilot_bridge.msg as apbrg
import ap_msgs.msg as apmsg
from autopilot_bridge.msg import LLA
import ap_lib.gps_utils as gps
import ap_lib.ap_enumerations as enums
import ap_lib.sasc_scrimmage as ss

class RedGreedyShooter(ss.Tactic):

    _shot = set()
    _reds_taken = set()

    def init(self, params):
        self._id = int(params['id'])
        self._target_id = None
        self._last_target = None
        self._wp = np.array([0, 0, 0])
        self._max_range = enums.MAX_RANGE_DEF
        self._fov_width = enums.HFOV_DEF
        self._fov_height = enums.VFOV_DEF
        self._own_pose = apbrg.Geodometry()
        self._blues = dict()
        self._reds = dict()
        self._shot = set()
        self._reds_last_lat = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self._safe_waypoint = np.array([0, 0, 0])
        self._action = ["None", 0]
        self._name = 'RedGreedyShooter'

    def step_autonomy(self, t, dt):

        # Reset the action and target ID every loop
        self._action = ["None", 0]

        if self._target_id in RedGreedyShooter._shot:
            RedGreedyShooter._reds_taken.remove(self._target_id)
            self._target_id = None
        else:
             self._target_id = self._last_target

        min_dist = np.inf
        for uav in self._reds.values():
            uav_lat = uav.state.pose.pose.position.lat
            uav_lon = uav.state.pose.pose.position.lon
            d = gps.gps_distance(self._own_pose.pose.pose.position.lat,\
                                     self._own_pose.pose.pose.position.lon,\
                                     uav_lat,\
                                     uav_lon)

            

            if uav.vehicle_id not in RedGreedyShooter._shot:
                if uav_lat == self._reds_last_lat[uav.vehicle_id]:     # check to see if target moved since last time
                    RedGreedyShooter._shot.add(uav.vehicle_id)    # add to dead list
                    print "===============================>Enemy ID: %d added to dead list" % (uav.vehicle_id)
                    if uav.vehicle_id in RedGreedyShooter._reds_taken:
                        RedGreedyShooter._reds_taken.remove(uav.vehicle_id)
                else:
                    self._reds_last_lat[uav.vehicle_id] = uav_lat      # yes, update position
                    if uav.vehicle_id not in RedGreedyShooter._reds_taken:
                       if d < min_dist:
                           min_dist = d
                           self._target_id = uav.vehicle_id
                           self._last_target = self._target_id

        # If a target has been identified, move towards it
        if self._target_id != None:
            #if self._target_id is not self._last_target:
                #RedGreedyShooter._reds_taken.remove(self._last_target)

            RedGreedyShooter._reds_taken.add(self._target_id)
            target_pos = self._reds[self._target_id].state.pose.pose.position
            
            own_lat = self._own_pose.pose.pose.position.lat
            own_lon = self._own_pose.pose.pose.position.lon
            tgt_lat = target_pos.lat
            tgt_lon = target_pos.lon
            tgt_alt = target_pos.rel_alt

            d = gps.gps_distance(self._own_pose.pose.pose.position.lat,\
                                     self._own_pose.pose.pose.position.lon,\
                                     tgt_lat,\
                                     tgt_lon)

            print "ID: %d   Target ID: %d   Dist: %d" % (self._id, self._target_id, d)
            s = ", ".join(str(e) for e in RedGreedyShooter._reds_taken)
            print "Reds Taken: "
            print s

            # Set the waypoint past the current target, so we don't go into a
            # loiter mode
            bearing = gps.gps_bearing(own_lat, own_lon, tgt_lat, tgt_lon)
            lat, lon = gps.gps_newpos(own_lat, own_lon, bearing, 200)   # range was 1000

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
                           tgt_lat, tgt_lon, tgt_alt):
                self._action = ["Fire", self._target_id]
                print "=====>ID: %d shot at Target %d" % (self._id, self._target_id)

            '''if tgt_lat != self._reds_last_lat[self._target_id]:     # check to see if target moved since last time
                self._reds_last_lat[self._target_id] = tgt_lat      # yes, update position
            else:
                RedGreedyShooter._shot.add(self._target_id)    # add to dead list
                print "=====>Enemy ID: %d added to dead list" % (self._target_id)
                RedGreedyShooter._reds_taken.remove(self._target_id)
                self._last_target = None
                s = ", ".join(str(e) for e in RedGreedyShooter._shot)
                print "Reds Shot: "
                print s
            '''
            # RedGreedyShooter._reds_taken.remove(self._target_id)    # release enemy ID from set
        else:
            # If a target hasn't been identified, return to the safe waypoint
            self._wp = self._safe_waypoint
        return True
