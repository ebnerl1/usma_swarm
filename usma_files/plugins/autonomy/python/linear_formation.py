import sys try:
    sys.argv
except AttributeError:
    sys.argv = ['']
import std_msgs.msg as stdmsg
import numpy as np
import math
import autopilot_bridge.msg as apbrg
import ap_msgs.msg as apmsg
from autopilot_bridge.msg import LLA
import ap_lib.gps_utils as gps
import ap_lib.ap_enumerations as enums
import ap_lib.distributed_algorithms as dist
import ap_lib.bitmapped_bytes as bytes
import ap_lib.behavior as behavior
import autonomy_itar.path_planning_utils as pputils
from ap_lib import sasc_scrimmage
import threading
import enum
from ap_lib import quaternion_math as qmath
#import lvdb

class States(enum.Enum):
    setup = 1
    fly = 2

class LeaderStates(enum.Enum):
    straight = 1
    turn = 2

class LinearFormation(sasc_scrimmage.Tactic):

    def init(self, params):
        self._id = int(params['id'])
        self._target_id = -1
        self._max_range = enums.MAX_RANGE_DEF
        self._fov_width = enums.HFOV_DEF
        self._fov_height = enums.VFOV_DEF
        self._own_pose = apbrg.Geodometry()
        self._blues = dict()
        self._reds = dict()
        self._shot = set()
        self.wp_state = 0
        self.wp_change_time = -np.inf
        self.straight_time = 30
        self.bank_time = 60

        self.cube_offset = 100
        self.geobox = gps.GeoBox(
            enums.BATTLE_CUBE_SW_LAT, enums.BATTLE_CUBE_SW_LON,
            enums.BATTLE_CUBE_LENGTH, enums.BATTLE_CUBE_WIDTH,
            enums.BATTLE_CUBE_ORIENT)

        self.curr_corner = 3

        try:
            self._safe_waypoint = self._parent._safe_waypoint
        except AttributeError:
            self._safe_waypoint = np.array([0, 0, 0])

        self._wp = self._safe_waypoint
        pos = self._own_pose.pose.pose.position
        self._safe_waypoint = np.array([pos.lat, pos.lon, pos.alt])

        self._action = ["None", 0]
        self._name = 'LinearFormation'        
        self.behavior_state = States.setup
        try:
            self.wp_calc = \
                pputils.InterceptCalculator(
                    self._parent, self._id, self._parent._swarm
                )
        except AttributeError: 
            self.wp_calc = \
                pputils.InterceptCalculator(self, self._id, dict())

        parser = bytes.LinearFormationOrderParser()

        self.formation_params = {
            'dist': parser.distance,
            'ang': parser.angle,
            'stacked': parser.stack_formation,
            'lead': False,
            'start_time': -np.nan
        }

        topic = 'network/send_swarm_behavior_data'
        try:
            self.lock = self._parent._swarm_lock
            self.pubs[topic] = self._parent._behaviorDataPublisher

        except AttributeError:
            self.lock = self.lock = threading.RLock()
            self.pubs[topic] = \
                self.createPublisher(topic, apmsg.BehaviorParameters, 1)
            self.subs[topic] = \
                self.createSubscriber(
                    topic, apmsg.BehaviorParameters,
                    self._process_swarm_data_msg
                )

        self.sorter = dist.EagerConsensusSort(set(), set(), self.lock, self._id)
        self.sorter.set_msg_publisher(self.pubs[topic])
        self.sorter.reset(self._id)

    def _process_swarm_data_msg(self, msg):
        try:
            self.sorter.process_message(msg)
        except AttributeError:
            self.log_warn("received message but sorter not set. Init has not been called")


    def setup_formation(self, t):
        try:
            hi_to_lo = self.sorter.decide_sort(t)
            hi_to_lo.reverse()
            if not hi_to_lo:
                return None
        except AttributeError: 
            return None

        self.formation_params['start_time'] = t
        args = [self.formation_params['dist'], self.formation_params['ang'],
                self._wp[2], pputils.InterceptCalculator.BASE_ALT_MODE]

        if hi_to_lo[0][0] == self._id:
            print 'setting self as lead'
            self.formation_params['lead'] = True
            return True

        elif self.formation_params['stacked']:
            print 'setting self as follower'
            self.formation_params['lead'] = False
            return self.wp_calc.set_params(hi_to_lo[0][0], *args)

        else:
            print 'setting self as follower2'
            idx = next(data for data in hi_to_lo if data[0] == self._id)
            return self.wp_calc.set_params(hi_to_lo[idx - 1][0], *args)

    def yaw(self):
        q = self._own_pose.pose.pose.orientation
        return qmath.quat_to_euler((q.x, q.y, q.z, q.w))[2]

    def step_autonomy(self, t, dt):                

        # remove any red entries in sorter
        to_del = [key for key in self.sorter._data if key in self._reds]
        for key in to_del:
            del self.sorter._data[key]

        self.sorter.send_requested(t)

        self.wp_calc._swarm = self._blues

        self.sorter._subswarm = set(self.wp_calc._swarm.keys())

        if self.behavior_state == States.setup:
            self._wp = self._safe_waypoint
            if not self.setup_formation(t):
                return True

            self.behavior_state = States.fly

        else:
            
            if self.formation_params['lead']:

                pos = self._own_pose.pose.pose.position
                if gps.gps_distance(pos.lat, pos.lon, self._wp[0], self._wp[1]) < 2 * self.cube_offset:
                    print 'selecting a new waypoint'
                    self.curr_corner = (self.curr_corner + 1) % 4

                wp = self.geobox._corners[self.curr_corner]
                self._wp = np.array([wp[0], wp[1], self._wp[2]])

            elif not self.formation_params['lead']:
                wp = self.wp_calc.compute_intercept_waypoint(None, t)
                if wp is None:
                    self.log_warn("calculated a None waypoint to leader in LinearFormation")
                else: 
                    self._wp = np.array([wp.lat, wp.lon, wp.alt])

        return True
