'''
Tactic Name: Evader Attacker (EvadeAttack)
Description:  This tactic tries to avoid flying into the "kill box" of the enemy.  Each enemy UAV
                is evaluated and orientation with respect to the enemy is calculated.  The tactic
                veers left or right if flying directly toward the enemy (at least in his extended
                "kill box") and also determines if I am directly in front of the UAV as well to veer
                left or right to avoid the "kill box".
Date:		18 April 2017
Changes:	1. (22 Apr 2017):  Added filtering to make sure any enemy being pursued is actually in
            the game box to avoid tareting UAVs that are powered up in TOC, on runway, etc.
            2. (23 Apr 2017):  Used Kevin DeMarco's fix with checking UAV game status
Authors:	USMA SASC Team
'''
import numpy as np
import math
from ap_lib import quaternion_math as qmath
import autopilot_bridge.msg as apbrg
import ap_msgs.msg as apmsg
from autopilot_bridge.msg import LLA
import ap_lib.gps_utils as gps
import ap_lib.ap_enumerations as enums
import ap_lib.sasc_scrimmage as ss

ADJ_ANGLE = math.pi/4

class EvadeAttack(ss.Tactic):

    def init(self, params):
        self._id = int(params['id'])
        self._target_count = 0
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
        self._name = 'EvadeOrAttack'
        self._fov_width_radians = math.radians(enums.HFOV_DEF)

    def step_autonomy(self, t, dt):

        # Reset the action and target ID every loop
        self._action = ["None", 0]

        # Search for the closest target every frame
        for uav in self._reds.values():
            if uav.vehicle_id not in self._shot and (uav.game_status == enums.GAME_ACTIVE_DEFENSE or uav.game_status == enums.GAME_ACTIVE_OFFENSE):
                # Calculate target and own position and orientation
                target_pos = self._reds[uav.vehicle_id].state.pose.pose.position
                target_orientation = self._reds[uav.vehicle_id].state.pose.pose.orientation
                tgt_lat = target_pos.lat
                tgt_lon = target_pos.lon
                tgt_alt = target_pos.rel_alt

                own_pos = self._own_pose.pose.pose.position
                own_lat = own_pos.lat
                own_lon = own_pos.lon
                own_alt = own_pos.rel_alt
                own_orientation = self._own_pose.pose.pose.orientation

                # Calculate bearing to target and from target to self
                bearing = gps.normalize(gps.gps_bearing(own_lat, own_lon, tgt_lat, tgt_lon))
                tgt_bearing = gps.normalize(gps.gps_bearing(tgt_lat, tgt_lon, own_lat, own_lon))

                # Calculate heading/yaw of self and target
                own_rpy = qmath.quat_to_euler((own_orientation.x,\
                                       own_orientation.y, \
                                       own_orientation.z, \
                                       own_orientation.w))
                own_heading = gps.normalize(own_rpy[2])

                tgt_rpy = qmath.quat_to_euler((target_orientation.x,\
                                       target_orientation.y, \
                                       target_orientation.z, \
                                       target_orientation.w))
                tgt_heading = gps.normalize(tgt_rpy[2])

                # Calculate distance to target
                dist = gps.gps_distance(own_lat, own_lon, tgt_lat, tgt_lon)
                
                bearing_adj = 0.0

                # Calculate offset between bearing and target heading (measures degree of "head on")
                # ----------------------------------------------------------------------------
                heading_diff = own_heading - tgt_heading
                tgt_bearing_diff = gps.normalize_pi( tgt_heading - tgt_bearing )
                if math.fabs(heading_diff) > math.pi/2:
                    # moving towards each other
                    if tgt_bearing > (tgt_heading - self._fov_width_radians/2) or tgt_bearing < (tgt_heading + self._fov_width_radians/2):
                        # we are in the "fan"
                        if dist > 2*self._max_range and math.fabs(tgt_bearing_diff) < self._fov_width_radians/2:                
                            bearing_adj = math.copysign(ADJ_ANGLE, tgt_bearing_diff)
                            #print 'ID: {} toward UAV: {}: adjusting bearing by {}'.format(self._id, uav.vehicle_id, math.degrees(bearing_adj))
                        elif dist < self._max_range and math.fabs(tgt_bearing_diff) > self._fov_width_radians*3:
                            # along side each other
                            bearing_adj = math.copysign(3*ADJ_ANGLE, tgt_bearing_diff)
                            #print 'ID: {} along side UAV: {}: adjusting bearing by {}'.format(self._id, uav.vehicle_id, math.degrees(bearing_adj))
                else:
                    # heading in same general direction
                    if math.fabs(tgt_bearing_diff) < math.pi/2:
                        # I am in front of target
                        if dist < 2*self._max_range and math.fabs(tgt_bearing_diff) < self._fov_width_radians/2:                      
                            bearing_adj = -math.copysign(ADJ_ANGLE, tgt_bearing_diff)
                            #print 'ID: {} away from UAV: {} adjusting bearing by {}'.format(self._id, uav.vehicle_id, math.degrees(bearing_adj))
                # ----------------------------------------------------------------------------
                lat, lon = gps.gps_newpos(own_lat, own_lon, bearing + bearing_adj, 1000)
                self._wp = np.array([lat, lon, own_alt])    # keep own alt

                # Determine if the target is within firing parameters
                if gps.hitable(own_lat, own_lon, own_alt, \
                               (own_orientation.x, \
                                own_orientation.y, \
                                own_orientation.z, \
                                own_orientation.w ), \
                               self._max_range, self._fov_width, self._fov_height,\
                               tgt_lat, tgt_lon, tgt_alt):
                    self._action = ["Fire", uav.vehicle_id]
                    print "EA=======================>ID: %d shot at Target %d" % (self._id, uav.vehicle_id)

        return True
