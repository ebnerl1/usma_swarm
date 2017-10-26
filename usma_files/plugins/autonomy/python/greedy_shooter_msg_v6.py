'''
Tactic Name: Greedy Shooter Messaging version 5 (GreedyShooterMsg5)
Description:  This tactic is a variation of the TutorialGreedyShooter in tutorial_greedy_shooter_1.py
              The tactic uses a "score" that takes into account not only distance, but how well a 
              potential target is in the FOV envelope.  The UAV the score to list so that the best
              target is at the tail.  It communicates to all teammates this enemy ID and score, so they
              can negotiate who has the best score.  If a UAV loses, it selects the next best in the list.
              If there are no other options, it keeps the sole target.  The messaging is based on the
              tutorial to use the custom "RedTarget" message fields to hold the self._id, self._target_id,
              distance, and bearing.  It is a variation of GreedyShooterMsg4 in that there is a compound
              list that holds the enemy ID and the score.  It does not do optimization in the main loop,
              but does it after all the enemy have been added to the list and is sorted.  This keeps the
              first enemy with the lowest score from terminating the list build operation
Date:		11 Apr 2017
Changes:	
Authors:	USMA SASC Team
'''
import math
from ap_lib import quaternion_math as qmath
import numpy as np
import autopilot_bridge.msg as apbrg
import ap_msgs.msg as apmsg
from autopilot_bridge.msg import LLA
import ap_lib.gps_utils as gps
import ap_lib.ap_enumerations as enums
import ap_lib.sasc_scrimmage as ss
import usma_msg.RedTarget as redtgt

MIN_DIST = np.inf      # range to start looking for enemy
MIN_SCORE = np.inf      # product of distance and relative bearing metric
WEIGHT_DIST = 0.5       # give N% preference to the distance value for score; bearing gets 1.0-N%
WEIGHT_BEARING = 1.0 - WEIGHT_DIST
RAD2DEG = 57.296

class GreedyShooterMsg5(ss.Tactic):

    def init(self, params):
        self._id = int(params['id'])
        self._target_id = None  # -1
        self._wp = np.array([0, 0, 0])
        self._max_range = enums.MAX_RANGE_DEF
        self._target_dist = np.inf              # added to keep track of current target range
        self._target_bearing = np.inf
        self._target_score = 0
        self._fov_width = enums.HFOV_DEF
        self._fov_height = enums.VFOV_DEF
        self._own_pose = apbrg.Geodometry()
        self._blues = dict()
        self._reds = dict()
        self._num_reds = 0
        self._shot = set()
        self._tmp_list = list()     # sets up lists of found enemy
        self._sorted_list = list()  # sorted list
        self._target_list = list()
        self._target_geo_list = list()
        self._change = False
        self._safe_waypoint = np.array([0, 0, 0])
        self._action = ["None", 0]
        self._name = 'GreedyShooterMsg'

        # This is the new messaging logic (from the documentation)
        self.pubs['red_tgt'] = self.createPublisher('red_tgt', redtgt.RedTarget, 1)
        self.subs['red_tgt'] = self.createSubscriber('red_tgt', redtgt.RedTarget, self.receive_target_msg)

    def receive_target_msg(self, msg):
        #The msg has fields id, target_id, dist, and bearing
        if msg.id != self._id:           # don't run on our own message
            if msg.target_id == self._target_id:
                if msg.dist < self._target_dist and msg.bearing < self._target_score:
                    l = len(self._target_list)
                    if l > 1:
                        #print '{} dropped UAV {} due to {} being closer at distance {} and bearing {}'.format(self._id, self._target_id, msg.id, msg.dist, msg.bearing)
                        self._target_list.pop()
                        l = len(self._target_list)
                        (self._target_id, self._target_dist, self._target_score) = self._target_list[l-1]
                        print 'Change: {} now targeting UAV {} at distance {} score {}'.format(self._id, self._target_id, self._target_dist, self._target_score)
                        self._change = True

    def step_autonomy(self, t, dt):

        # Reset the action and target ID every loop
        self._action = ["None", 0]
        #self._change = False
        self._target_dist = np.inf
        self._target_score = np.inf
        del self._tmp_list[:]          # clear the search lists
        #del self._target_list[:]
        del self._target_geo_list[:]

        # Search for the closest target every frame
        min_dist = MIN_DIST     # range to start looking for enemy
        min_score = MIN_SCORE
        self._num_reds = 0
        for uav in self._reds.values():
            if (uav.vehicle_id not in self._shot):
                target_pos = self._reds[uav.vehicle_id].state.pose.pose.position
                own_lat = self._own_pose.pose.pose.position.lat
                own_lon = self._own_pose.pose.pose.position.lon
                own_alt = self._own_pose.pose.pose.position.rel_alt
                own_orientation = self._own_pose.pose.pose.orientation
                tgt_lat = target_pos.lat
                tgt_lon = target_pos.lon
                tgt_alt = target_pos.rel_alt

                # get distance and bearing to potential enemy
                d = gps.gps_distance(own_lat, own_lon, tgt_lat, tgt_lon)
                bearing = gps.gps_bearing(own_lat, own_lon, tgt_lat, tgt_lon)

                # Convert absolute angles to relative (shooter to target)
                pose_quat = (own_orientation.x, own_orientation.y, own_orientation.z, own_orientation.w)
                rpy = qmath.quat_to_euler(pose_quat)
                rel_bearing = math.fabs(gps.normalize_angle(bearing - rpy[2]))*RAD2DEG
                #print 'UAV {} to enemy {}:  Dist: {}  Bearing {}'.format(self._id, uav.vehicle_id), d, rel_bearing)

                # is the target UAV in front of me?
                score = d*rel_bearing   # lower is better
                if uav.vehicle_id not in self._tmp_list:
                    self._tmp_list.append((uav.vehicle_id, d, score))
                    #print 'UAV {} to enemy {}:  Dist: {}  Bearing {}  Score {}'.format(self._id, uav.vehicle_id, d, rel_bearing, score)         

                self._num_reds = self._num_reds + 1

        for uav in self._target_list:
            if uav[0] in self._shot:
                self._target_list.remove(uav)

        #target_list_len = len(self._target_list) - 1

        if not self._change:                    # target_list_len < 0 or target_list_len < (self._num_reds - 2):
            del self._target_list[:]
            self._target_list = sorted(self._tmp_list, key=lambda tgts: tgts[1], reverse=True)
            self._change = False

        target_list_len = len(self._target_list) - 1
        self._target_id = self._target_list[target_list_len][0]
        self._target_dist = self._target_list[target_list_len][1]
        self._target_score = self._target_list[target_list_len][2]

        #print 'UAV {} target list = {}'.format(self._id, self._target_list)

        # If a target has been identified, move towards it
        if self._target_id != None:
            
            own_lat = self._own_pose.pose.pose.position.lat
            own_lon = self._own_pose.pose.pose.position.lon
            own_alt = self._own_pose.pose.pose.position.rel_alt
            own_orientation = self._own_pose.pose.pose.orientation
            for uav in self._target_list:
                target_pos = self._reds[uav[0].state.pose.pose.position
                tgt_lat = target_pos.lat
                tgt_lon = target_pos.lon
                tgt_alt = target_pos.rel_alt
                d = gps.gps_distance(own_lat, own_lon, tgt_lat, tgt_lon)
                bearing = gps.gps_bearing(own_lat, own_lon, tgt_lat, tgt_lon)
                self._target_geo_list.append(uav[0], tgt_lat, tgt_lon, tgt_alt)

            msg = redtgt.RedTarget()
            msg.id = self._id
            msg.target_id = self._target_id
            msg.dist = self._target_dist
            msg.bearing = self._target_score    # re-use bearing field
            self.pubs['red_tgt'].publish(msg)
            print '{} targeting UAV {} at distance {} score {}'.format(self._id, self._target_id, self._target_dist, self._target_score)

            # Set the waypoint past the current target, so we don't go into a
            # loiter mode
            lat, lon = gps.gps_newpos(own_lat, own_lon, bearing, 100)

            self._wp = np.array([lat, lon, own_alt])     # don't climb or dive to target (was target_pos.rel_alt)

            # Determine if the target is within firing parameters
            if gps.hitable(own_lat, own_lon, own_alt, \
                           (own_orientation.x, \
                            own_orientation.y, \
                            own_orientation.z, \
                            own_orientation.w ), \
                           self._max_range, self._fov_width, self._fov_height,\
                           tgt_lat, tgt_lon, tgt_alt):
                self._action = ["Fire", self._target_id]
                print "GSM5======================> ID: %d shot at Target %d" % (self._id, self._target_id)
        else:
            # If a target hasn't been identified, return to the safe waypoint
            print 'UAV {} has no target ID!'.format(self._id)
            self._wp = self._safe_waypoint
        self._change = False
        return True

