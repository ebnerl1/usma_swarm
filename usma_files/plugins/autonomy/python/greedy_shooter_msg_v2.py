'''
Tactic Name: Greedy Shooter Messaging version 2(GreedyShooterMsg2)
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

class GreedyShooterMsg2(ss.Tactic):

    def init(self, params):
        self._id = int(params['id'])
        self._target_id = None  # -1
        self._wp = np.array([0, 0, 0])
        self._max_range = enums.MAX_RANGE_DEF
        self._target_dist = np.inf     # added to keep track of current target range
        self._best_dist = np.inf
        self._best_id = None
        self._fov_width = enums.HFOV_DEF
        self._fov_height = enums.VFOV_DEF
        self._own_pose = apbrg.Geodometry()
        self._blues = dict()
        self._reds = dict()
        self._shot = set()
        self._taken = set()     # this is the set that this UAV will use to keep track of "taken" enemy
        self._tmp_taken = set()
        self._reported = set()
        self._t_id = list()     # sets up lists of found enemy
        self._t_dist = list()
        self._t_lock = False
        self._safe_waypoint = np.array([0, 0, 0])
        self._action = ["None", 0]
        self._name = 'GreedyShooterMsg2'

        # This is the new messaging logic (from the documentation)
        self.pubs['taken'] = self.createPublisher('taken', apmsg.MsgStat, 1)
        self.subs['taken'] = self.createSubscriber('taken', apmsg.MsgStat, self.receive_msg_stat)

    def receive_msg_stat(self, msg):
        #self._reported.add(msg.id)
        if self._id != msg.id:
            self._taken.add(msg.count)
            #print '{} received message from {}: Targeting {} at distance {}'.format(self._id, msg.id, msg.count, msg.latency) 
            if msg.count == self._target_id:
                if msg.latency < self._target_dist: #self._best_dist:
                    target_pos = self._reds[msg.count].state.pose.pose.position
                    blue_lat = self._blues[msg.id].state.pose.pose.position.lat
                    blue_lon = self._blues[msg.id].state.pose.pose.position.lon
                    blue_alt = self._blues[msg.id].state.pose.pose.position.rel_alt
                    blue_orientation = self._blues[msg.id].state.pose.pose.orientation
                    tgt_lat = target_pos.lat
                    tgt_lon = target_pos.lon
                    tgt_alt = target_pos.rel_alt

                    if gps.hitable(blue_lat, blue_lon, blue_alt, \
                        (blue_orientation.x, \
                        blue_orientation.y, \
                        blue_orientation.z, \
                        blue_orientation.w ), \
                        np.inf, 45.0, self._fov_height,\
                        tgt_lat, tgt_lon, tgt_alt):
                        #self._best_dist = msg.latency
                        #self._best_id = msg.id
                        print '{} is closer than {} to UAV {} and is within 45 deg'.format(msg.id, self._id, msg.count)
                        l = len(self._t_id)
                        if l > 1:
                            self._t_id.pop()
                            self._t_dist.pop()
                            self._target_id = self._t_id[l-2]
                            self._target_dist = self._t_dist[l-2]
                            #self._taken.add(msg.count)
                            print '{} dropped UAV {} due to {} being better candidate at distance {}'.format(self._id, msg.count, msg.id, msg.latency)
                            print '{} now targeting UAV {} at a distance of {}'.format(self._id, self._target_id, self._target_dist)
                        else:
                            self._target_id = None
                            self._target_dist = np.inf
                            #self._taken.discard(msg.count)
                    else:
                        self._taken.discard(msg.count)

                '''if self._best_dist < self._target_dist:
                    l = len(self._t_id)
                    if l > 1:
                        self._t_id.pop()
                        self._t_dist.pop()
                        self._target_id = self._t_id[l-2]
                        self._target_dist = self._t_dist[l-2]
                        #self._taken.add(msg.count)
                        print '{} dropped UAV {} due to {} being better candidate at distance {}'.format(self._id, msg.count, self._best_id, self._best_dist)
                        print '{} now targeting UAV {} at a distance of {}'.format(self._id, self._target_id, self._target_dist)
                    else:
                        self._target_id = None
                        self._target_dist = np.inf
                        #self._taken.discard(msg.count)
                
                else:
                    self._taken.discard(msg.count)
                '''

        '''if self._reported == set(self._blues) or len(self._blues) == 1:
            #self._taken = self._tmp_taken.copy()
            #self._tmp_taken.clear()
            self._reported.clear()
            self._best_dist = np.inf
            self._best_id = None
            #self._t_lock = False
        '''
           
    def step_autonomy(self, t, dt):

        # Reset the action and target ID every loop
        self._action = ["None", 0]
        self._target_id = None
        #print 'My lat: {}'.format(self._blues[self._id].state.pose.pose.position.lat)
        # Search for the closest target every frame
        min_dist = MIN_DIST     # range to start looking for enemy
        del self._t_id[:]             # clear the search lists
        del self._t_dist[:]

        for uav in self._reds.values():
            #self._t_lock = True
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
                if d < min_dist:
                    min_dist = d
                    if gps.hitable(own_lat, own_lon, own_alt, \
                        (own_orientation.x, \
                        own_orientation.y, \
                        own_orientation.z, \
                        own_orientation.w ), \
                        np.inf, 180.0, self._fov_height,\
                        tgt_lat, tgt_lon, tgt_alt):

                        if uav.vehicle_id not in self._t_id:
                            self._t_id.append(uav.vehicle_id)
                            self._t_dist.append(d)
                            self._target_id = uav.vehicle_id
                            self._target_dist = d
        
        # If a target has been identified, move towards it
        if self._target_id != None:
            msg = apmsg.MsgStat()
            msg.id = self._id
            msg.count = self._target_id
            msg.latency = self._target_dist
            self.pubs['taken'].publish(msg)
            #print '{} has target list: {}'.format(self._id, self._t_id)
            #print 'at ranges: {}'.format(self._t_dist)
            print '{} targeting UAV {} at a distance of {}'.format(self._id, self._target_id, self._target_dist)
            print '{} has taken {}'.format(self._id, self._taken)            
            
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
            lat, lon = gps.gps_newpos(own_lat, own_lon, bearing, 500)
            #print '{} targeting UAV {} at a distance of {} bearing {}'.format(self._id, self._target_id, self._target_dist, bearing)
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

