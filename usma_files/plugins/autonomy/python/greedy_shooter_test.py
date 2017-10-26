import py_scrimmage as sc
import numpy as np
import autopilot_bridge.msg as apbrg
import ap_msgs.msg as apmsg
from autopilot_bridge.msg import LLA
import ap_lib.gps_utils as gps
import ap_lib.ap_enumerations as enums
import ap_lib.sasc_scrimmage as ss

class GreedyShooter(sc.Autonomy):    # sc.Autonomy

    def init(self, params):     # was __init__()
        #self.ID = sc.ID()     # added to try to use like Action example; remove ID part to get back to orig
        self.id = params['id']  # sc.ID.id    # params['id']
        self.swarm_id = params['swarm_id']  # sc.ID.sub_swarm_id   # params['id']
        self.team_id = params['team_id']    # sc.ID.team_id     # params['team_id']
        self._wp = np.array([0, 0, 0])
        self._max_range = enums.MAX_RANGE_DEF
        self._fov_width = enums.HFOV_DEF
        self._fov_height = enums.VFOV_DEF
        self._own_pose = apbrg.Geodometry()
        self._blues = dict()
        self._reds = dict()
        self._reds_shot = set()
        self._safe_waypoint = np.array([0, 0, 0])
        self._action = ["None", 0]
        self.target_id = -1
        pass

    def name(self): 
        return "greedy shooter v1"

    def step_autonomy(self, t, dt):

        self.action = sc.Action()
        self.action.action_type = sc.Action.ActionType.None
        self.action.target_id = 0

        if len(self._reds) == 0:
            self.target_id = -1

        if self.target_id == -1:

            min_dist = np.inf
            for contact in self._reds.values():
                dist = np.linalg.norm(contact.state.pose.pose.position - self._own_pose.pose.pose.position)
                if self.team_id != contact.id.team_id and dist < min_dist:
                    min_dist = dist
                    self.target_id = contact.id.id

        if self.target_id != -1:
            target_state = self._reds[self.target_id].state

            heading = np.arctan2(target_state.pos[1] - self.state.pos[1],
                                 target_state.pos[0] - self.state.pos[0])

            self.desired_state = \
                sc.State(target_state.pos,
                         np.array([30, 0, 0]),
                         sc.Quaternion(0, 0, heading))

            dist = np.linalg.norm(self.state.pos - target_state.pos)
            if dist < 50 and self.state.in_field_of_view(target_state, 10, 10):
                self.action.action_type = sc.Action.ActionType.Fire
                self.action.target_id = self.target_id

        return True 
