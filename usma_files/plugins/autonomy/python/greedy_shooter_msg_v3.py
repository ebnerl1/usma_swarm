'''
Tactic Name: Greedy Shooter Messaging version 3(GreedyShooterMsg3)
Description:  This tactic is a variation of the TutorialGreedyShooter in tutorial_greedy_shooter_1.py
              The tactic uses a "score" that takes into account not only distance, but how well a 
              potential target is in the FOV envelope.  The UAV the score to list so that the best
              target is at the tail.  It communicates to all teammates this enemy ID and score, so they
              can negotiate who has the best score.  If a UAV loses, it selects the next best in the list.
              If there are no other options, it keeps the sole target.  The messaging is based on the
              tutorial to use the MsgStat message fields to hold the self._id, self._target_id, and
              self._target_score.
Date:		22 March 2017
Changes:	1. (24 Mar 2017): Reworked the description to match the actual algorithm; got rid of the
            "taken" list
            2. (6 Apr 2017): Changed appending of dist (d) to score.  Still not working as desired; will
            modify and resave as v4
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

MIN_DIST = np.inf      # range to start looking for enemy
MIN_SCORE = np.inf      # product of distance and relative bearing metric

class GreedyShooterMsg3(ss.Tactic):

    def init(self, params):
        self._id = int(params['id'])
        self._target_id = None  # -1
        self._wp = np.array([0, 0, 0])
        self._max_range = enums.MAX_RANGE_DEF
        self._target_range = np.inf     # added to keep track of current target range
        self._target_score = np.inf
        self._fov_width = enums.HFOV_DEF
        self._fov_height = enums.VFOV_DEF
        self._own_pose = apbrg.Geodometry()
        self._blues = dict()
        self._reds = dict()
        self._shot = set()
        self._t_id = list()     # sets up lists of found enemy
        self._t_score = list()
        self._safe_waypoint = np.array([0, 0, 0])
        self._action = ["None", 0]
        self._name = 'GreedyShooterMsg'

        # This is the new messaging logic (from the documentation)
        self.pubs['score'] = self.createPublisher('score', apmsg.MsgStat, 1)
        self.subs['score'] = self.createSubscriber('score', apmsg.MsgStat, self.receive_msg_stat)

    def receive_msg_stat(self, msg):
        #print '{} received message from {}:  Targeting {} with score {}'.format(self._id, msg.id, msg.count, msg.latency)
        if msg.count == self._target_id:
            if msg.latency < self._target_score:
                l = len(self._t_id)
                if l > 1:
                    self._t_id.pop()
                    self._t_score.pop()
                    self._target_id = self._t_id[l-2]
                    self._target_dist = self._t_score[l-2]
                    print '{} dropped UAV {} due to {} being better candidate with score {}'.format(self._id, msg.count, msg.id, msg.latency)
                    #print '{} now targeting UAV {} with score of {}'.format(self._id, self._target_id, self._target_dist)

    def step_autonomy(self, t, dt):

        # Reset the action and target ID every loop
        self._action = ["None", 0]
        self._target_id = None
        self._target_range = np.inf
        self._target_score = np.inf
        del self._t_id[:]             # clear the search lists
        del self._t_score[:]

        # Search for the closest target every frame
        min_dist = MIN_DIST     # range to start looking for enemy
        min_score = MIN_SCORE
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
                rel_bearing = math.fabs(gps.normalize_angle(bearing - rpy[2]))
                #print 'UAV {} to enemy {}:  Dist: {}  Bearing {}'.format(self._id, uav.vehicle_id), d, rel_bearing)

                # is the target UAV in front of me?
                score = d * rel_bearing   # lower is better
                print 'UAV {} to enemy {}:  Dist: {}  Bearing {}  Score {}'.format(self._id, uav.vehicle_id, d, rel_bearing, score)
                if score < min_score:
                    min_score = score
                    #self._target_score = score
                    #self._target_id = uav.vehicle_id
                    #self._target_range = d
                    if uav.vehicle_id not in self._t_id:
                        self._t_id.append(uav.vehicle_id)
                        self._t_score.append(score)         # whoops!  was d!
                        self._target_id = uav.vehicle_id
                        self._target_score = score
                        self._target_range = d
                        #print 'UAV {} to enemy {}:  Dist: {}  Bearing {}  Score {}'.format(self._id, uav.vehicle_id, d, rel_bearing, score)

        # If a target has been identified, move towards it
        if self._target_id != None:
            msg = apmsg.MsgStat()
            msg.id = self._id
            msg.count = self._target_id
            msg.latency = self._target_score
            self.pubs['score'].publish(msg)
            #print '{} targeting UAV {} with a score of of {}'.format(self._id, self._target_id, self._target_score)            
            
            target_pos = self._reds[self._target_id].state.pose.pose.position
            own_lat = self._own_pose.pose.pose.position.lat
            own_lon = self._own_pose.pose.pose.position.lon
            own_alt = self._own_pose.pose.pose.position.rel_alt
            own_orientation = self._own_pose.pose.pose.orientation
            tgt_lat = target_pos.lat
            tgt_lon = target_pos.lon
            tgt_alt = target_pos.rel_alt

            # Set the waypoint past the current target, so we don't go into a
            # loiter mode
            bearing = gps.gps_bearing(own_lat, own_lon, tgt_lat, tgt_lon)
            lat, lon = gps.gps_newpos(own_lat, own_lon, bearing, 1000)

            self._wp = np.array([lat, lon, target_pos.rel_alt])

            # Determine if the target is within firing parameters
            if gps.hitable(own_lat, own_lon, own_alt, \
                           (own_orientation.x, \
                            own_orientation.y, \
                            own_orientation.z, \
                            own_orientation.w ), \
                           self._max_range, self._fov_width, self._fov_height,\
                           tgt_lat, tgt_lon, tgt_alt):
                self._action = ["Fire", self._target_id]
                print "=============> Wing %d shot at Target %d" % (self._id, self._target_id)
        else:
            # If a target hasn't been identified, return to the safe waypoint
            self._wp = self._safe_waypoint
        return True

