'''
Tactic Name: Greedy Shooter Messaging (GreedyShooterMsg)
Description:  This tactic is a variation of the TutorialGreedyShooter in tutorial_greedy_shooter_1.py
              The tactic adds messaging between UAVs to tell each other which target they have.  This will
              be added upon to make a list of "taken" enemy so each UAV targets a unique enemy.
Date:		15 March 2017
Changes:	1. (16 Mar 2017) Changed angle in "hittable" discriminator to 180.0 so that the UAV targets
                anything in front of it (was 90.0)
            2. (17 Mar 2017)  Changed topic name to 'taken' (was 'my_topic')
Authors:	USMA SASC Team
'''
import numpy as np
import autopilot_bridge.msg as apbrg
import ap_msgs.msg as apmsg
from autopilot_bridge.msg import LLA
import ap_lib.gps_utils as gps
import ap_lib.ap_enumerations as enums
import ap_lib.sasc_scrimmage as ss

MIN_DIST = np.inf      # range to start looking for enemy

class GreedyShooterMsg(ss.Tactic):

    def init(self, params):
        self._id = int(params['id'])
        self._target_id = None  # -1
        self._wp = np.array([0, 0, 0])
        self._max_range = enums.MAX_RANGE_DEF
        self._target_range = np.inf     # added to keep track of current target range
        self._fov_width = enums.HFOV_DEF
        self._fov_height = enums.VFOV_DEF
        self._own_pose = apbrg.Geodometry()
        self._blues = dict()
        self._reds = dict()
        self._shot = set()
        self._taken = set()     # this is the set that this UAV will use to keep track of "taken" enemy
        self._safe_waypoint = np.array([0, 0, 0])
        self._action = ["None", 0]
        self._name = 'GreedyShooterMsg'

        # This is the new messaging logic (from the documentation)
        self.pubs['taken'] = self.createPublisher('taken', apmsg.MsgStat, 1)
        self.subs['taken'] = self.createSubscriber('taken', apmsg.MsgStat, self.receive_msg_stat)

    def receive_msg_stat(self, msg):
        print '{} received message from {}:  Targeting {} at distance {}'.format(self._id, msg.id, msg.count, msg.latency)
        self._taken.add(msg.count)
        if msg.count == self._target_id:
            if msg.latency < self._target_range:
                self._target_id = None
                self._target_range = np.inf
            else:
                self._taken.discard(msg.count)  # remove from my list if I am close

    def step_autonomy(self, t, dt):

        # Reset the action and target ID every loop
        self._action = ["None", 0]
        self._target_id = None
        self._target_range = np.inf
        # print set(self._blues)

        # Search for the closest target every frame
        min_dist = MIN_DIST     # range to start looking for enemy
        for uav in self._reds.values():
            if (uav.vehicle_id not in self._shot) and (uav.vehicle_id not in self._taken):
                target_pos = self._reds[uav.vehicle_id].state.pose.pose.position
                own_lat = self._own_pose.pose.pose.position.lat
                own_lon = self._own_pose.pose.pose.position.lon
                own_alt = self._own_pose.pose.pose.position.rel_alt
                own_orientation = self._own_pose.pose.pose.orientation
                tgt_lat = target_pos.lat
                tgt_lon = target_pos.lon
                tgt_alt = target_pos.rel_alt

                d = gps.gps_distance(own_lat, own_lon, tgt_lat, tgt_lon)

                # is the target UAV in front of me?
                if gps.hitable(own_lat, own_lon, own_alt, \
                    (own_orientation.x, \
                    own_orientation.y, \
                    own_orientation.z, \
                    own_orientation.w ), \
                    np.inf, 180.0, self._fov_height,\
                    tgt_lat, tgt_lon, tgt_alt):
                    if d < min_dist:
                        min_dist = d
                        self._target_id = uav.vehicle_id
                        self._target_range = d

        if (self._target_id in self._shot) or (self._target_id in self._taken):
            self._target_id = None
            self._target_range = np.inf
        
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
            lat, lon = gps.gps_newpos(own_lat, own_lon, bearing, 100)

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
                print "=================>ID: %d shot at Target %d" % (self._id, self._target_id)
        else:
            # If a target hasn't been identified, return to the safe waypoint
            self._wp = self._safe_waypoint
        self._taken.clear()         # clear the taken list in case it's full of old data
        return True

