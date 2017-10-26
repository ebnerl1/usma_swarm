import numpy as np
import math
import autopilot_bridge.msg as apbrg
import ap_msgs.msg as apmsg
from autopilot_bridge.msg import LLA
import ap_lib.gps_utils as gps
import ap_lib.ap_enumerations as enums
import ap_lib.sasc_scrimmage as ss

from angles import normalize

import local_behavior

import utm
import pyquaternion as quat

class LocalFlocking(local_behavior.LocalBehavior):
    ''' An extendend implementation of Boids model

    See http://www.red3d.com/cwr/boids/ for the original model.

    This flocking behavior takes into account:

    '''

    def init(self, params):
        super(LocalFlocking, self).init(params)
        self._min_wall_time = int(params['min_wall_time'])
        self._max_neighbors = int(params['max_neighbors'])
        self._target_gain = float(params['target_gain'])        
        self._name = 'LocalFlocking'

    def step_autonomy(self, t, dt):
        super(LocalFlocking, self).step_autonomy(t, dt)

        # If outside of battlebox, head towards own base!"
        if not self._battlebox.contains(self._own_pose.pose.pose.position.lat,
                                        self._own_pose.pose.pose.position.lon,
                                        self._own_pose.pose.pose.position.alt):
            self._wp = np.array([self._own_base[0],
                                 self._own_base[1],
                                 self._last_ap_wp[2]])            
            return True            

        # Compute distance to each blue uav, tuple: (id, distance)
        # Don't include ourself
        blue_dists = [ (id, np.linalg.norm(self.state.pos[:2]-state.pos[:2]))
                       for id, state in self.local_blues.iteritems() 
                       if id != self._id and 
                       self.state.in_field_of_view(state.pos, math.pi, math.pi) ]

        # Experimental: only use neighbors in front of own uav
        #

        # Sort blue distances
        blue_dists.sort(key=lambda x: x[1])

        # return nearest neighbors        
        neighbors = blue_dists[:self._max_neighbors]

        # Separation: steer to avoid crowding local flockmates
        sep_vec_sum = np.array([0, 0])

        # Alignment: steer towards the average heading of local flockmates
        yaw_sum = 0

        # Cohesion: steer to move toward the average position (centroid) of
        # local flockmates
        centroid_sum = np.array([0, 0])

        # Set gains for each behavior
        sep_gain = 0.05
        yaw_gain = 1
        coh_gain = 0.05
        tgt_gain = self._target_gain
        box_gain = 10

        # calculate running sum for each neighbor-based behavior
        for id, state in neighbors:
            # Calculate vector pointing from neighbor to own position
            vec_diff = (self.state.pos[:2] - self.local_blues[id].pos[:2])

            # Running sum of vectors pointing from neighbor to own position for
            # separation vector calculation
            sep_vec_sum = sep_vec_sum + vec_diff / np.linalg.norm(vec_diff)

            # Running sum of neighbor headings for heading alignment
            # calculation
            yaw_sum = yaw_sum + self.local_blues[id].yaw()

            # Running sum of neighbor positions for centroid calculation
            centroid_sum = centroid_sum + self.local_blues[id].pos[:2]

        # Initialize vectors to zero
        vec_result = np.array([0, 0])
        sep_vec = np.array([0, 0])
        yaw_vec = np.array([0, 0])
        coh_vec = np.array([0, 0])
        tgt_vec = np.array([0, 0])
        box_vec = np.array([0, 0])

        # Normalize / average neighbor-based behaviors
        if len(neighbors) > 0:
            sep_vec = sep_vec_sum / len(neighbors) # Normalize separation vec
            avg_yaw = yaw_sum / len(neighbors)     # Average yaw calc
            centroid = centroid_sum / len(neighbors) # Centroid of neighbors

            # Convert desired centroid to velocity pointing from own position
            # to centroid
            coh_vec = centroid - self.state.pos[:2]

            # Convert average yaw to vector pointing in direction for own uav
            # to achieve average yaw
            yaw_vec = np.array([math.cos(avg_yaw), math.sin(avg_yaw)])

        # Compute distance to each red uav, tuple: (id, distance)
        red_dists = [ (id, np.linalg.norm(self.state.pos[:2]-state.pos[:2])) \
                      for id, state in self.local_reds.iteritems() ]

        # Determine nearest enemy
        try:
            closest_enemy_id = min(red_dists, key = lambda t: t[1])[0]
            tgt_dir = self.local_reds[closest_enemy_id].pos[:2] - self.state.pos[:2]
            tgt_vec = tgt_dir / np.linalg.norm(tgt_dir)
        except ValueError:
            # No enemies exist, zero out target vector
            tgt_vec = np.array([0, 0])

        # Get time-to-breach closest battle cube wall.
        wall_time, wall_angle = self.closest_wall_time_angle()

        # If the time-to-breach closest wall is less than 10 seconds, calculate
        # avoidance velocity vector
        if wall_time < self._min_wall_time:
            box_yaw = 0
            if wall_angle > 0:
                # Turn hard left (increase angle) if the wall_angle is greater
                # than 0
                box_yaw = normalize(self.state.yaw() + math.pi-0.1,
                                    -math.pi, math.pi)
            else:
                # Turn hard right (decrease angle) if the wall_angle is greater
                # than 0
                box_yaw = normalize(self.state.yaw() - math.pi-0.1,
                                    -math.pi, math.pi)

            # Convert desired yaw to velocity vector
            box_vec = np.array([math.cos(box_yaw), math.sin(box_yaw)])

        # Perform linear combination of each flocking behavior
        vec_result = sep_gain * sep_vec + \
                     yaw_gain * yaw_vec + \
                     coh_gain * coh_vec + \
                     tgt_gain * tgt_vec + \
                     box_gain * box_vec

        # If the vector sum of all flocking behaviors is very small, set the
        # velocity to the current velocity
        if np.linalg.norm(vec_result) < 0.001:
            vec_result = self.state.vel[:2]

        # Convert local desired velocity vector to a GPS waypoint
        self._wp = self.vector_2_waypoint(vec_result, alt_deconflict=True)

        # Determine if any enemy aircraft can be tagged
        for uav_id in self.local_reds:
            if self.hitable(uav_id):
                self._action = ["Fire", uav_id]
                break

        return True
