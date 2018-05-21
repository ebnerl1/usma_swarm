import numpy as np
import math
import autopilot_bridge.msg as apbrg
import ap_msgs.msg as apmsg
from autopilot_bridge.msg import LLA
import ap_lib.gps_utils as gps
import ap_lib.ap_enumerations as enums
import ap_lib.sasc_scrimmage as ss
from ap_lib.sasc_state import SASCState, from_gps_heading, to_gps_heading

import utm
import pyquaternion as quat
from angles import normalize
import operator

def perp(a):
    ''' Create perpendicular vector to a
    '''
    b = np.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b

# line segment a given by endpoints a1, a2
# line segment b given by endpoints b1, b2
# return  
def seg_intersect(a1, a2, b1, b2):
    '''Calculate intersection point between two line segments

     line segment a given by endpoints a1, a2 line segment b given by endpoints
     b1, b2

    '''
    da = a2-a1
    db = b2-b1
    dp = a1-b1
    dap = perp(da)
    denom = np.dot( dap, db)    
    num = np.dot( dap, dp )
    return (num / denom.astype(float))*db + b1

class LocalBehavior(ss.Tactic):
    def init(self, params):
        self._id = int(params['id'])
        self._team_id = int(params['team_id'])
        self._target_id = -1
        self._wp = np.array([0, 0, 0])
        self._max_range = enums.MAX_RANGE_DEF
        self._fov_width = enums.HFOV_DEF
        self._fov_height = enums.VFOV_DEF
        self._own_pose = apbrg.Geodometry()
        self._blues = dict()
        self._reds = dict()
        self._shot = set()
        self._safe_waypoint = np.array([0, 0, 0])
        self._last_ap_wp = np.array([0, 0, 0])
        self._action = ["None", 0]
        self._name = 'LocalBehavior'
        self._battleboxes = gps.get_battleboxes()
        self._battlebox = self._battleboxes[0]
        self._vehicle_type = int(params['vehicle_type'])
        self._local_corners = None
        self._run_conversions = True # only modified by scrimmage
        
        if self._team_id == 1:
            self._own_base = enums.GOAL_POSITS['blue']
        else:
            self._own_base = enums.GOAL_POSITS['red']

        if self._team_id == 1:
            self._enemy_base = enums.GOAL_POSITS['red']
        else:
            self._enemy_base = enums.GOAL_POSITS['blue']

    def step_autonomy(self, t, dt): 
        # Reset the action and target ID every loop
        self._action = ["None", 0]
        self._target_id = None

        # Convert own pose to local cartesian
        self.state = SASCState(self._own_pose, self._id)
        
        # Convert all GPS coordinates to local cartesian coordinates
        if self._run_conversions:
            self.local_conversions()    
            
    def closest_wall_time_angle(self):   
        '''Calculate the time to reach the closest battle box wall 

        ... And the angle between the closest wall and the uav's velocity.

        '''
        # Create line segment out of uav's position and relative velocity that
        # extends past battle box
        uav_forw = self.state.pos + self.state.vel * 100000
        uav_back = self.state.pos + self.state.vel * -100000
    
        # Calculate intersections between the line formed by the uav's current
        # velocity and each line around the large battle box
        inters_0 = seg_intersect(uav_back[:2], uav_forw[:2], 
                                 np.array(self._local_corners[0][:2]), 
                                 np.array(self._local_corners[1][:2]))

        inters_1 = seg_intersect(uav_back[:2], uav_forw[:2], 
                                 np.array(self._local_corners[1][:2]), 
                                 np.array(self._local_corners[2][:2]))

        inters_2 = seg_intersect(uav_back[:2], uav_forw[:2], 
                                 np.array(self._local_corners[2][:2]), 
                                 np.array(self._local_corners[3][:2]))

        inters_3 = seg_intersect(uav_back[:2], uav_forw[:2], 
                                 np.array(self._local_corners[3][:2]), 
                                 np.array(self._local_corners[0][:2]))

        inters = {0:inters_0, 1:inters_1, 2:inters_2, 3:inters_3}
        
        # Calculate the distances from the uav's own position to the
        # intersection points on the battle box
        dists = { key:np.linalg.norm(self.state.pos[:2] - i) 
                  for key, i in inters.iteritems() }

        # Given the uav holds it's current velocity, calculate the time until
        # the uav intersects with the batttle box
        times = { key:i/np.linalg.norm(self.state.vel[:2]) 
                  for key, i in dists.iteritems() }
        
        # Sort the times dictionary
        sorted_times = sorted(times.items(), key=operator.itemgetter(1))
        
        # Filter out points that are behind the uav. If the dot product of the
        # uav's current velocity and the vector extending from the uav's own
        # position to the intersection point is greater than 0, then the
        # intersection point is "in front" of the uav
        closest_wall = sorted_times[0][0]
        v = self.state.vel[:2]
        for id, dist in sorted_times:            
            if np.dot(v, inters[id] - self.state.pos[:2]) > 0.0:
                closest_wall = id                
                break
        
        time = times[closest_wall]
                
        # Find angle between own velocity and closest wall. If the angle is
        # negative, the vehicle should turn "right" to avoid the wall.
        other_wall_id = closest_wall + 1
        if other_wall_id > 3:
            other_wall_id = 0
            
        wall_vec = np.array(self._local_corners[other_wall_id][:2]) - np.array(self._local_corners[closest_wall][:2])
        wall_perp = perp(wall_vec)
        angle = math.asin(np.cross(wall_perp, self.state.vel[:2]) / \
                          (np.linalg.norm(wall_perp) * np.linalg.norm(self.state.vel[:2])))
        
        return ( time, angle )
        
    def vector_2_waypoint(self, vec, alt_deconflict=True):
        '''Convert velocity in local cartesian frame to waypoint in GPS

        Convert a velocity in the local cartesian frame to a waypoint in GPS
        frame. if alt_deconflict is True, then the current altitude is
        maintained.

        '''
        lla = self.state.local_2_lla(self.state.pos[:2] + vec)
        
        if self._vehicle_type == enums.AC_FIXED_WING:
            # Get vector's direction
            theta = math.atan2(vec[1], vec[0])
        
            # Extend waypoint in direction of desired velocity
            lat, lon = gps.gps_newpos(lla[0], lla[1], from_gps_heading(theta), 1000)
                        
        wp = np.array([lat, lon, lla[2]])
        if alt_deconflict:                   
            wp[2] = self._last_ap_wp[2]
        return wp    
            
    def intercept_target(self, target_id, alt_deconflict=True):
        '''Returns a waypoint that will intercept the target.

        If the vehicle is a fixed-wing, the waypoint will be extended beyond
        the waypoint loiter distance

        '''
        own_lla = self.state.lla()
        tgt_lla = self.local_reds[target_id].lla()

        lat = tgt_lla[0]
        lon = tgt_lla[1]
        
        if self._vehicle_type == enums.AC_FIXED_WING:
            # When using fixed wing, Set the waypoint past the current
            # target, so we don't go into a loiter mode
            bearing = gps.gps_bearing(own_lla[0], own_lla[1], 
                                      tgt_lla[0], tgt_lla[1])
            lat, lon = gps.gps_newpos(own_lla[0], own_lla[1], bearing, 1000)

        wp = np.array([lat, lon, tgt_lla[2]])                

        # Handle altitude deconfliction
        if alt_deconflict:                   
            wp[2] = self._last_ap_wp[2]
        return wp    
        
    def hitable(self, target_id): 
        '''Returns True if the target_id is hitable

        '''
        own_lla = self.state.lla()
        try:
            tgt_lla = self.local_reds[target_id].lla()
        except KeyError:
            print('ID not in local_reds: %d' % target_id)
            return false
        return gps.hitable(own_lla[0], own_lla[1], own_lla[2], \
                           (self.state.quat_gps.vector[0], \
                            self.state.quat_gps.vector[1], \
                            self.state.quat_gps.vector[2], \
                            self.state.quat_gps.scalar ), \
                           self._max_range, self._fov_width, self._fov_height,\
                           tgt_lla[0], tgt_lla[1], tgt_lla[2])

    def local_conversions(self):
        '''Converts all lat/lon/alt to local cartesian coordinates.
        
        Converts all lat/lon/alt to local cartesian coordinates. If
        alt_deconflict is True, then all vehicles appear to be on the same
        altitude plane in the local frame even though they are actually
        altitude separated.

        '''                                                        
        
        # Convert ros data type to local data type This line of code converts
        # the self_.blues dictionary into a filtered version that only includes
        # non-shot uavs. Also, we filter out our own ID. The new blues
        # dictionary contains the vehicles in the local cartesian frame
        self.local_blues = { id:SASCState(uav.state,id) 
                             for id, uav in self._blues.iteritems() 
                             if id not in self._shot and 
                             self._battlebox.contains(uav.state.pose.pose.position.lat,
                                                      uav.state.pose.pose.position.lon,
                                                      uav.state.pose.pose.position.alt)}

        self.local_reds = { id:SASCState(uav.state,id) 
                            for id, uav in self._reds.iteritems() 
                            if id not in self._shot and
                            self._battlebox.contains(uav.state.pose.pose.position.lat,
                                                     uav.state.pose.pose.position.lon,
                                                     uav.state.pose.pose.position.alt)}

        # convert battlebox to utm
        self._local_corners = [ utm.from_latlon(i[0], i[1]) \
                               for i in self._battlebox._corners ]        
                                        
        return True

    def receive_firing_report(self, msg):
        print '{} received that someone is targeting {}' \
              .format(self._id, msg.report.target_id)
