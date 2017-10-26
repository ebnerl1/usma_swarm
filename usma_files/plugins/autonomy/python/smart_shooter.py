import numpy as np
import math
import autopilot_bridge.msg as apbrg
import ap_msgs.msg as apmsg
import ap_lib.swarm_helper as pursueBytes
from autopilot_bridge.msg import LLA
import ap_lib.gps_utils as gps
import ap_lib.ap_enumerations as enums
from ap_lib import sasc_scrimmage
import autonomy_itar.path_planning_utils as pputils
import enum
import rospy

class States(enum.Enum):
    fly = 0
    no_tgts = 1
    hunt_tgt = 2
    track_tgt = 3
    shoot_tgt = 4
    terminal_standby = 5

class SmartShooter(sasc_scrimmage.Tactic):

    MAX_TIME_LATE = rospy.Duration(10.0)
    BDA_TIME_OUT = 3.0
    RQD_IN_ENVELOPE = 1

    def init(self, params):
        self._id = int(params['id'])
        self._target_id = None
        self._max_range = enums.MAX_RANGE_DEF
        self._fov_width = enums.HFOV_DEF
        self._fov_height = enums.VFOV_DEF
        self._own_pose = apbrg.Geodometry()
        self._blues = dict()
        self._reds = dict()
        self._shot = set()

        try:
            self._safe_waypoint = self._parent._safe_waypoint
        except AttributeError:
            self._safe_waypoint = np.array([0, 0, 0])
        self._wp = self._safe_waypoint

        try:
            self._vehicle_type = int(params['vehicle_type'])
        except KeyError:
            self._vehicle_type = enums.AC_UNKNOWN

        self._action = ["None", 0]
        self._name = 'SmartShooter'
        self.behavior_state = States.hunt_tgt
        self.pursuit_status = False
        self.shot_pending = False
        self.overtime = False
        self.reds_pursued = []
        self.wp_calc = \
            pputils.PNInterceptCalculator(
                self, self._id, self._reds, max_time_late=self.MAX_TIME_LATE
            ) 

        try:
            topic = 'network/send_swarm_behavior_data'
            # we dont need to create a subscriber here because this is taken care of in 
            self.pubs[topic] = self._parent._behavior_data_publisher 
        except AttributeError:
            self.pubs[topic] = \
                self.createPublisher(topic, apmsg.BehaviorParameters, 1)
            self.subs[topic] = \
                self.createSubscriber(
                    topic, apmsg.BehaviorParameters, self._process_swarm_data_msg
                )

    def identify_target(self, t):
        self._target_id = None
        self_pos = self._own_pose.pose.pose.position
        min_range = np.inf

        for uav_id in self._reds:
            red_veh_state = self._reds[uav_id]
            red_pos = red_veh_state.state.pose.pose.position

            shot = uav_id in self._shot
            pursued = uav_id in self.reds_pursued
            active_game = \
                (red_veh_state.game_status == enums.GAME_ACTIVE_DEFENSE or
                 red_veh_state.game_status == enums.GAME_ACTIVE_OFFENSE)
            recent_pose = \
                (t - red_veh_state.state.header.stamp.to_sec() <
                 self.MAX_TIME_LATE.to_sec())

            if not shot and not pursued and recent_pose and active_game:
                d = gps.gps_distance(self_pos.lat, self_pos.lon,
                                     red_pos.lat, red_pos.lon)

                if d < min_range:
                    self._target_id = uav_id
                    self.target_dist = d
                    min_range = d

        if self._target_id:
            self.pursuit_status = True
            self.wp_calc.set_params(
                self._target_id, 0.0, 0.0, self_pos.alt,
                pputils.InterceptCalculator.BASE_ALT_MODE
            )
            self.pursuit_status_update()

    def track_target(self, t):

        #intercept = self.wp_calc.compute_intercept_waypoint(None, t)
        tgt_pose = self.wp_calc.leader_state()

        if self._target_id in self._shot:
            self.behavior_state = States.hunt_tgt
            self.pursuit_status = False
            self.pursuit_status_update()

        elif self._id in self._shot:
            self.behavior_state = States.no_tgts
            self.pursuit_status = False
            self.pursuit_status_update()

        #if intercept:
        #    self._wp = np.array([intercept.lat, intercept.lon, intercept.alt])

        #else:
        #    return False

        self._wp = self.intercept_target(self._target_id)

        return True

    def pursuit_status_update(self):
        if self.overtime:
            return

        parser = pursueBytes.PursuitMessageParser()
        parser.friendly_id = self._id

        if self._target_id != None:
            parser.target_id = self._target_id
            parser.target_distance = self.target_dist
        else:
            parser.target_id = 0
            parser.target_distance = self._max_range

        parser.pursuit_status = self.pursuit_status
        data = parser.pack()
        net_msg = apmsg.BehaviorParameters()
        net_msg.id = pursueBytes.PURSUIT_MESSAGE
        net_msg.params = data
        for _ in range(1): # Hope to get at least one through
            self.pubs['network/send_swarm_behavior_data'].publish(net_msg)

    def intercept_target(self, target_id, alt_deconflict=True):
        '''Returns a waypoint that will intercept the target.

        If the vehicle is a fixed-wing, the waypoint will be extended beyond
        the waypoint loiter distance

        '''
        own_pos = self._own_pose.pose.pose.position
        target_pos = self._reds[target_id].state.pose.pose.position
        own_lla = (own_pos.lat, own_pos.lon, own_pos.alt)
        tgt_lla = (target_pos.lat, target_pos.lon, target_pos.alt)

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

    def target_hitable(self, target_id):
        sp = self._own_pose.pose.pose.position
        so = self._own_pose.pose.pose.orientation
        quat = (so.x, so.y, so.z, so.w)

        if target_id in self._reds:
            target_pos =  self._reds[target_id].state.pose.pose.position
            return gps.hitable(
                sp.lat, sp.lon, sp.alt, quat,
                self._max_range, self._fov_width, self._fov_height,
                target_pos.lat, target_pos.lon, target_pos.alt
            )
        else:
            return False

    def step_autonomy(self, t, dt):                

        self.wp_calc._swarm = self._reds.copy()
        self.wp_calc._swarm.update(self._blues)

        self._wp = self._safe_waypoint
        self._action = ["None", 0]


        for uav_id in self._reds:
                if uav_id not in self._shot and self.target_hitable(uav_id):
                    self._action = ["Fire", uav_id]

                        
        if self._target_id is not None and np.any(self._wp != 0):
            self.wp_calc.set_params(
                self._target_id, 0.0, 0.0, self._own_pose.pose.pose.position.alt,
                pputils.InterceptCalculator.BASE_ALT_MODE
            )

        if self.behavior_state == States.hunt_tgt:
            self.identify_target(t)
            self.shot_pending = False
            self.behavior_state = \
                States.track_tgt if self._target_id is not None else States.no_tgts

        elif self.behavior_state == States.track_tgt:
            intercept = self.track_target(t)

            if intercept:
                if self.target_hitable(self._target_id):
                    self.envelope_counter = 1
                    self.behavior_state = States.shoot_tgt

            else:
                self.behavior_state = States.hunt_tgt

        elif self.behavior_state == States.shoot_tgt:
            intercept = self.track_target(t)
            if intercept:
                if self._target_id in self._shot:
                    self.behavior_state = States.hunt_tgt
                else:
                    self.behavior_state = States.track_tgt

            else:
                self.behavior_state = States.hunt_tgt

        elif self.behavior_state == States.no_tgts:
            if len(self.reds_pursued) > 0:
                self.reds_pursued = []
                self.overtime = True
                self.behavior_state = States.hunt_tgt
            else:
                self._wp = self._safe_waypoint
                self.behavior_state = States.terminal_standby

        elif self.behavior_state == States.terminal_standby:

            if self._id not in self._shot:
                self._target_id = self.identify_target(t)
                if self._target_id != None:
                    self.shot_pending = False
                    self.behavior_state = States.track_tgt

        return True 

    def _process_swarm_data_msg(self, msg):
        if msg.id == pursueBytes.PURSUIT_MESSAGE:
            parser = pursueBytes.PursuitMessageParser()
            parser.unpack(msg.params)

            if parser.friendly_id in self._reds or parser.friendly_id == self._id:
                # dont process items from the other team
                return

            if parser.pursuit_status == True:
                if parser.target_id not in self.reds_pursued:
                    self.reds_pursued.append(parser.target_id)

                if (parser.friendly_id != self._id and
                    parser.target_id == self._target_id):

                    if parser.target_distance < self.target_dist:
                        self.pursuit_status = False
                        self.pursuit_status_update()
                        self._target_id = None
                        self.behavior_state = States.hunt_tgt
                    else: 
                        self.pursuit_status_update()

            else:
                if parser.target_id in self.reds_pursued:
                    self.reds_pursued.remove(parser.target_id)
