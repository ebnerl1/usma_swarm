import numpy as np
import math
import autopilot_bridge.msg as apbrg
import ap_msgs.msg as apmsg
from autopilot_bridge.msg import LLA
import ap_lib.gps_utils as gps
import ap_lib.ap_enumerations as enums
import ap_lib.sasc_scrimmage as ss

class SetVOParams(ss.Tactic):

    def init(self, params):
        self._id = int(params['id'])
        self._target_id = -1
        self._wp = np.array([0, 0, 0])
        self._own_pose = apbrg.Geodometry()
        self._blues = dict()
        self._reds = dict()
        self._shot = set()
        self._safe_waypoint = np.array([0, 0, 0])
        self._action = ["None", 0]
        self._name = 'SetVOParams'
        
        self._hysteresis = float(params['Hysteresis'])
        self._object_radius = float(params['Object Radius'])
        self._t_cpa_thresh = float(params['T CPA Thresh'])
        self._mode_2D = bool(params['Mode 2D'])

        topic = 'velocity_obstacles_params'
        try:            
            self.pubs[topic] = \
                               self.createPublisher(topic, apmsg.VelocityObstacles, 1)            
        except AttributeError:            
            pass

        msg = apmsg.VelocityObstacles()
        msg.hysteresis = self._hysteresis
        msg.object_radius = self._object_radius
        msg.t_cpa_thresh = self._t_cpa_thresh
        msg.mode_2D = self._mode_2D
        self.pubs[topic].publish(msg)                

    def step_autonomy(self, t, dt):                                        
        # Reset the action and target ID every loop
        self._action = ["None", 0]                        
        self._target_id = None        
        self._wp = self._safe_waypoint        
        return True
