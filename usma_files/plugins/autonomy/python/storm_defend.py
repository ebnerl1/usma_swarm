'''
Tactic Name: Storm Defender (StormDefend)
Description:  This tactic is a variation of the GotoDefend tactic. It assigns a pre-designated waypoint
                according to the id of the aircraft.  The waypoints are defined in usma_enumerations.py
                and form at 4x4 grid with WP 0 at the NW corner of the battlecube, WP 3 at the SW corner,
                and continuing until WP 15 at the SE corner of the battlecube.   The assigned matrix is 
                cusomizable by specifying a starting location, the number of rows and columns desired 
                (caution, no error checking is yet implemented so do not go "out of bounds").

                |===================|
                |   0   4   8   12  |           ex.  If (start)location = 5, rows = 2, columns = 2,
                |   1   5   9   13  |           the UAVs will be assigned to locations 5, 6, 9, and 10
                |   2   6  10   14  |           according to their ID
                |   3   7  11   15  |
                |===================|           The parameters are set in StormDefend.xml

                It uses the parent class, Goto2 in goto_v2.py as the basis for navigating to the waypoint.
                After that step autonomy completes, it continues on to a "Greedy Shooter" algorithm.
Date:		13 Apr 2017
Changes:	
Authors:	USMA SASC Team
'''
import numpy as np
import math
import autopilot_bridge.msg as apbrg
import ap_msgs.msg as apmsg
from autopilot_bridge.msg import LLA
import ap_lib.gps_utils as gps
import ap_lib.ap_enumerations as enums
import ap_lib.sasc_scrimmage as ss
import usma_enumerations as usma_enums
import goto_v2 as goto

class StormDefend(goto.Goto2):

    def init(self, params):
        super(StormDefend, self).init(params)        
        self._name = 'StormDefend'
        self._rows = int(params['rows'])
        self._columns = int(params['columns'])
        # split up sub-swarm into NxM array with (0,0) at postion self._location (0-15)
        self._loc = self._location + (self._id % self._rows) + (int(self._id / self._columns) % self._columns)*4
        #print 'ID: {}  Loc: {}'.format(self._id, self._loc)   
        self._desired_lat = float(usma_enums.WP_LOC[self._loc][0])
        self._desired_lon = float(usma_enums.WP_LOC[self._loc][1])          
        
    def step_autonomy(self, t, dt):
        status = super(StormDefend, self).step_autonomy(t, dt)            
        self._action = ["None", 0]
        # Fire at any reds that are within firing parameters
        for target_id, red in self._reds.iteritems():
            if (red.game_status == enums.GAME_ACTIVE_DEFENSE or red.game_status == enums.GAME_ACTIVE_OFFENSE):
                if gps.hitable(self._own_pose.pose.pose.position.lat, \
                               self._own_pose.pose.pose.position.lon, \
                               self._own_pose.pose.pose.position.rel_alt, \
                               (self._own_pose.pose.pose.orientation.x, \
                                self._own_pose.pose.pose.orientation.y, \
                                self._own_pose.pose.pose.orientation.z, \
                                self._own_pose.pose.pose.orientation.w ), \
                               self._max_range, self._fov_width, self._fov_height,\
                               red.state.pose.pose.position.lat, 
                               red.state.pose.pose.position.lon, 
                               red.state.pose.pose.position.rel_alt):
                    self._action = ["Fire", target_id]
                    print 'SD============================> ID: {} shot at UAV: {}'.format(self._id, target_id)
                            
        return status
