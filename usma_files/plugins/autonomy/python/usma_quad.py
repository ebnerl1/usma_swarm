'''
Tactic Name: USMA Quad (USMAQuad)
Description:  This tactic is a variation of the Vortex clas in vortex.py
              The tactic uses the "side-stepping" logic, but changed to point directly at the target
              (was "lagging" and one-on-ones would not result in a kill, depending on the "rotation").
              Also added logic to keep the quad on the defensive side (checks if the new lat/lon would
              take it across the center line.
              Also fixed a bug that always put the copter on the 'red' team.
Date:		4 March 2017
Changes:	1. (10 Mar 2017): Changed gps.get_battleboxes() to enums.get_battleboxes
            2. (10 Mar 2017): Implemented a scan-fire algorithm to modify the bearing by 2.5 degrees per cycle
                Description should now be:
              The tactic uses "side-stepping" logic, but engages the enemy at full range and uses a "scan-fire"
              algorithm to vary the pointing angle to the enemy by 2.5 degrees per time slice, allowing a lead/lag
              of the enemy automatically.
            3. (10 Mar 2017): Commented out section on determining weapon range; always the same
            4. (16 Mar 2017): Minor bug; had alt instead of rel_alt in pose
            5. (24 Mar 2017): Deleted all the obsolete code about teams, weapon ranges
            6. (6 Apr 2017): changed altitude for wp to own_alt (i.e. don't change alt)
            7. (14 Apr 2017): Checks hittable on all uavs in loop; calculates a "lead" bearing adjustment based
                on direction of enemy travel (CW or CCW) if targeting more than one interation; uses a "random"
                sidestep direction calculated by id to avoid bunching up
            8. (23 Apr 2017):  Added fix to only look for active enemy
Authors:	USMA SASC Team
'''

import numpy as np
import math
import random
import autopilot_bridge.msg as apbrg
import ap_msgs.msg as apmsg
from autopilot_bridge.msg import LLA
import ap_lib.gps_utils as gps
import ap_lib.ap_enumerations as enums
import ap_lib.sasc_scrimmage as ss

UAV_SPEED = 30.0

class USMAQuad(ss.Tactic):
    def init(self, params):
        self._id = int(params['id'])
        self._target_id = -1
        self._last_target_id = None
        self._target_bearing = 0.0
        self._last_target_bearing = 0.0
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
        self._name = 'USMA Quad'
        self._team = None
        self._bearing_adj = -(self._fov_width/2)

    def step_autonomy(self, t, dt):

        # Get our current position
        own_lat = self._own_pose.pose.pose.position.lat
        own_lon = self._own_pose.pose.pose.position.lon
        own_alt = self._own_pose.pose.pose.position.rel_alt     # was just alt

        # Reset the action every loop
        self._action = ["None", 0]
        self._last_target_id = self._target_id
        self._last_target_bearing = self._target_bearing
        self._target_id = None
        self._target_bearing = 0.0
        bearing_adj = -0.7     # default adjust for CCW circling enemy
        # Search for the closest target
        min_dist = np.inf
        for uav in self._reds.values():
            if uav.vehicle_id not in self._shot and (uav.game_status == enums.GAME_ACTIVE_DEFENSE or uav.game_status == enums.GAME_ACTIVE_OFFENSE):
                target_pos = self._reds[uav.vehicle_id].state.pose.pose.position
                d = gps.gps_distance(own_lat, \
                                     own_lon, \
                                     uav.state.pose.pose.position.lat, \
                                     uav.state.pose.pose.position.lon)
                if gps.hitable(self._own_pose.pose.pose.position.lat, \
                           self._own_pose.pose.pose.position.lon, \
                           self._own_pose.pose.pose.position.rel_alt, \
                           (self._own_pose.pose.pose.orientation.x, \
                            self._own_pose.pose.pose.orientation.y, \
                            self._own_pose.pose.pose.orientation.z, \
                            self._own_pose.pose.pose.orientation.w), \
                           self._weapons_range, self._fov_width, self._fov_height, \
                           target_pos.lat, target_pos.lon, target_pos.rel_alt):
                    self._action = ["Fire", uav.vehicle_id]
                    print "Quad=======(loop)=======> ID: %d shot at enemy %d" % (self._id, uav.vehicle_id)

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
            self._target_bearing = bearing

            if self._target_id == self._last_target_id:
                bearing_diff = self._target_bearing - self._last_target_bearing
                if bearing_diff < 0.0:
                    bearing_adj = math.sin(-UAV_SPEED/distance_to_target)
                else:
                    bearing_adj = math.sin(UAV_SPEED/distance_to_target)

            # Set waypoint to move forward and strafe sideways once the enemy is within range
            self._evasion_range = self._weapons_range * 3
            self._engagement_range = self._weapons_range * 2 / 3
            if distance_to_target <= self._engagement_range:
                lat, lon = gps.gps_newpos(own_lat, own_lon, gps.normalize(bearing + bearing_adj), 0.1)   # was - self._fov_width/2
                #print "UAV %d in engagement range with enemy %d bearing %f distance %f using %f bearing adj" % (self._id, self._target_id, math.degrees(bearing), distance_to_target, bearing_adj)
            elif distance_to_target <= self._evasion_range:
                # Check to see if sidestep right waypoint is outside the battlebox
                # (only need to check distance_to_target/2 since enemy is closing)
                evade_sign = -2*(self._id % 2) + 1
                #print "UAV %d in evasion range with enemy %d" % (self._id, self._target_id)
                lat, lon = gps.gps_newpos(own_lat, own_lon, gps.normalize(bearing - evade_sign*(math.pi / 3)), distance_to_target)
                if not self._bboxes[0].contains(lat, lon, own_alt):
                    lat, lon = gps.gps_newpos(own_lat, own_lon, gps.normalize(bearing + evade_sign*(math.pi / 3)), distance_to_target)
                    #print "UAV %d sidestepping left" % self._id                
            else:
                lat, lon = gps.gps_newpos(own_lat, own_lon, bearing, 1000)   # was 1000

            self._wp = np.array([lat, lon, own_alt])  # don't change alt, was target_pos.rel_alt

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
                print "Quad====================> ID: %d shot at enemy %d" % (self._id, self._target_id)
        else:
            # If a target hasn't been identified, return to the safe waypoint
            self._wp = self._safe_waypoint

        return True

    def receive_firing_report(self, msg):
        print '{} received that someone is targeting {}' \
            .format(self._id, msg.report.target_id)
