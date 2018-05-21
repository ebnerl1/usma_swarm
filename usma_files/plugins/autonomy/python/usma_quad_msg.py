'''
Tactic Name: USMA Quad Messaging (USMAQuadMsg)
Description:  This tactic is a variation of the USMAQuad in usma_quad.py
              The tactic incorporates the messaging using in GreedyShooterMsg to track "taken"
              enemy UAVs and try to target a unique ID.
Date:		17 March 2017
Changes:	1. (17 Mar 2017)  Changed topic name to 'taken' (was 'my_topic')
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

MIN_DIST = 300      # range to start looking for enemy
class USMAQuadMsg(ss.Tactic):
    def init(self, params):
        self._id = int(params['id'])
        self._target_id = -1
        self._target_range = np.inf     # added to keep track of current target range
        self._wp = np.array([0, 0, 0])
        self._weapons_range = enums.MAX_RANGE_DEF
        self._fov_width = enums.HFOV_DEF
        self._fov_height = enums.VFOV_DEF
        self._bboxes = enums.get_battleboxes()
        self._own_pose = apbrg.Geodometry()
        self._blues = dict()
        self._reds = dict()
        self._shot = set()
        self._taken = set()     # this is the set that this UAV will use to keep track of "taken" enemy
        self._t_id = list()     # sets up lists of found enemy
        self._t_range = list()
        self._safe_waypoint = np.array([0, 0, 0])
        self._action = ["None", 0]
        self._name = 'USMAQuadMsg'
        self._bearing_adj = 0

        # This is the new messaging logic (from the documentation)
        self.pubs['taken'] = self.createPublisher('taken', apmsg.MsgStat, 1)
        self.subs['taken'] = self.createSubscriber('taken', apmsg.MsgStat, self.receive_msg_stat)

    def receive_msg_stat(self, msg):
        #print '{} received message from {} with latency {}'.format(self._id, msg.id, msg.latency)
        print '{} received message from {}:  Targeting {} at distance {}'.format(self._id, msg.id, msg.count, msg.latency)
        self._taken.add(msg.count)
        if msg.count == self._target_id:
            if msg.latency < self._target_range:
                #self._target_id = None
                #self._target_range = np.inf
                t_id.pop()
                l = t_id.len() -1
                t_dist.pop()
                self._target_id = t_id[l]
                self._target_dist = t_dist[l]
                print '{} dropped UAV {} due to {} being closer at distance {}'.format(self._id, msg.count, msg.id, msg.latency)
                msg = apmsg.MsgStat()
                msg.id = self._id
                msg.count = self._target_id
                msg.latency = self._target_range
                self.pubs['taken'].publish(msg)
            else:
                self._taken.discard(msg.count)  # remove from my list if I am close

    def step_autonomy(self, t, dt):

        # Get our current position
        own_lat = self._own_pose.pose.pose.position.lat
        own_lon = self._own_pose.pose.pose.position.lon
        own_alt = self._own_pose.pose.pose.position.rel_alt     # was just alt

        # Reset the action every loop
        self._action = ["None", 0]
        self._target_id = None

        # Search for the closest target
        min_dist = MIN_DIST
        del t_id[:]
        del t_dist[:]
        for uav in self._reds.values():
            if (uav.vehicle_id not in self._shot) and (uav.vehicle_id not in self._taken):
                d = gps.gps_distance(own_lat, \
                                     own_lon, \
                                     uav.state.pose.pose.position.lat, \
                                     uav.state.pose.pose.position.lon)

                if d < min_dist:
                    min_dist = d
                    self._target_id = uav.vehicle_id
                    self._bearing_adj = -(self._fov_width/2)
                    self._target_range = d

        # If a target has been identified, move towards it
        if self._target_id != None:
            msg = apmsg.MsgStat()
            msg.id = self._id
            msg.count = self._target_id
            msg.latency = self._target_range
            self.pubs['taken'].publish(msg)
            print '{} targeting UAV {} at a distance of {}'.format(self._id, self._target_id, self._target_range)
            print '{} has taken list of {}'.format(self._id, self._taken)

            target_pos = self._reds[self._target_id].state.pose.pose.position

            tgt_lat = target_pos.lat
            tgt_lon = target_pos.lon

            distance_to_target = gps.gps_distance(own_lat, own_lon, tgt_lat, tgt_lon)

            # Set bearing to look at the enemy vehicle
            bearing = gps.gps_bearing(own_lat, own_lon, tgt_lat, tgt_lon)

            # Set waypoint to move forward and strafe sideways once the enemy is within range
            self._evasion_range = self._weapons_range * 2
            self._engagement_range = self._weapons_range
            if distance_to_target <= self._engagement_range:
                self._bearing_adj += 2.5
                if self._bearing_adj > self._fov_width:
                    self._bearing_adj = -self._fov_width
                bearing += math.radians(self._bearing_adj) # bearing_adj in degrees; convert to radians
                lat, lon = gps.gps_newpos(own_lat, own_lon, gps.normalize(bearing), 0.1)   # was - self._fov_width/2
                #print "UAV %d in engagement range with enemy %d bearing %f" % (self._id, self._target_id, math.degrees(bearing))
            elif distance_to_target <= self._evasion_range:
                # Check to see if sidestep right waypoint is outside the battlebox
                # (only need to check distance_to_target/2 since enemy is closing)
                #print "UAV %d in evasion range with enemy %d" % (self._id, self._target_id)
                lat_right, lon_right = gps.gps_newpos(own_lat, own_lon, \
                                                      gps.normalize(bearing + (math.pi / 3)), distance_to_target/2)
                if self._bboxes[0].contains(lat_right, lon_right, own_alt):
                    lat, lon = gps.gps_newpos(own_lat, own_lon, \
                                                      gps.normalize(bearing + (math.pi / 3)), distance_to_target)
                    #print "UAV %d sidestepping right" % self._id
                else:
                    # If right sidestep is outside the battlebox, sidestep to the left
                    lat, lon = gps.gps_newpos(own_lat, own_lon, gps.normalize(bearing - (math.pi / 3)), distance_to_target)
                    #print "UAV %d sidestepping left" % self._id
            else:
                lat, lon = gps.gps_newpos(own_lat, own_lon, bearing, 100)   # was 1000

            self._wp = np.array([lat, lon, target_pos.rel_alt])  # +random.randint(-10,10)]

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
                print "=================>ID: %d shot at Target %d" % (self._id, self._target_id)
        else:
            # If a target hasn't been identified, return to the safe waypoint
            self._wp = self._safe_waypoint
        self._taken.clear()         # clear the taken list in case it's full of old data
        return True

    def receive_firing_report(self, msg):
        print '{} received that someone is targeting {}' \
            .format(self._id, msg.report.target_id)
