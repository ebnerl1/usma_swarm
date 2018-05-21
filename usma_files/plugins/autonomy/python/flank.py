import numpy as np
import autopilot_bridge.msg as apbrg
import ap_lib.gps_utils as gps
import ap_lib.ap_enumerations as enums
import ap_lib.sasc_scrimmage as ss
from ap_lib import quaternion_math as qmath
import enum
import rospy
import math

class States(enum.Enum):
    new_start = 0
    patrol = 1
    kill_supply = 2


class Flank(ss.Tactic):
    def init(self, params):
        self._id = int(params['id'])
        self._target_id = -1
        self._wp = np.array([0, 0, 0])
        self._weapons_range = enums.MAX_RANGE_DEF
        self._tracking_range = self._weapons_range*1.5
        self._fov_width = enums.HFOV_DEF
        self._fov_height = enums.VFOV_DEF
        self._bboxes = enums.get_battleboxes()
        self._own_pose = apbrg.Geodometry()
        self._blues = dict()
        self._reds = dict()
        self._shot = set()
        self._safe_waypoint = np.array([0, 0, 0])
        self._action = ["None", 0]
        self._name = 'Flank'

        self._state = States.new_start

        try:
            self._team = rospy.get_param("swarm_team")
        except:
            self._team = 2

        try:
            self._vehicle_type = int(params['vehicle_type'])
        except KeyError:
            self._vehicle_type = enums.AC_UNKNOWN

        if self._team == 1:
            self._enemy_territory = self._bboxes[4]
        else:
            self._enemy_territory = self._bboxes[2]

        self.box_center = self.interpolate(self._bboxes[1][0][0], self._bboxes[1][0][1], self._bboxes[1][1][0],
                                           self._bboxes[1][1][1], 0.5)
        self._safe_corners = self.set_safe_corners()


    def step_autonomy(self, t, dt):

        # Reset the action every loop
        self._action = ["None", 0]
        self._target_id = None


        own_lat = self._own_pose.pose.pose.position.lat
        own_lon = self._own_pose.pose.pose.position.lon
        own_alt = 400


        if self._state == States.new_start:
            self.ordered_corners = self.closestCorners()
            lat, lon = self.fixed_wing_waypoint(own_lat, own_lon, self.ordered_corners[0][0], self.ordered_corners[0][1])
            if gps.gps_distance(own_lat, own_lon, self.ordered_corners[0][0], self.ordered_corners[0][1]) < 50:
                #print("Starting patrol!")
                self.ordered_corners = self.closestCorners()
                self._state = States.patrol
                self.check_for_supply_route()

        elif self._state == States.patrol:
            lat, lon = self.fixed_wing_waypoint(own_lat, own_lon, self.ordered_corners[2][0],
                                                self.ordered_corners[2][1])#At the time of ordering,
                                                                    #3rd closes waypoint is next in path,
                                                                    #1st closest is waypoint that you're currently at,
                                                                    #2nd closest is waypoint you visited previously
            if gps.gps_distance(own_lat, own_lon, self.ordered_corners[2][0], self.ordered_corners[2][1]) < 25:
                #print("Reached Waypoint at %f,%f!" % (lat, lon))
                self.ordered_corners = self.closestCorners()
                self.check_for_supply_route()



        elif self._state == States.kill_supply:
            lat, lon = self.fixed_wing_waypoint(own_lat, own_lon, self.ordered_corners[1][0],
                                                self.ordered_corners[1][1])
            if gps.gps_distance(own_lat, own_lon, self.ordered_corners[1][0], self.ordered_corners[1][1]) < 50:
                #print("Reached supply point at %f,%f!" % (lat, lon))
                self.ordered_corners = self.closestCorners()

        ######################################################################
        #If a target is in range, pursue and fire
        # Reset the action and target ID every loop
        self._action = ["None", 0]
        self._target_id = None

        # Search for the closest target every frame
        min_dist = self._weapons_range+30
        for uav in self._reds.values():
            if uav.vehicle_id not in self._shot:
                d = gps.gps_distance(self._own_pose.pose.pose.position.lat,\
                                     self._own_pose.pose.pose.position.lon,\
                                     uav.state.pose.pose.position.lat,\
                                     uav.state.pose.pose.position.lon)

                if d < min_dist:
                    #We also want to constrain our pursuits to enemies in front of us
                    if gps.hitable(self._own_pose.pose.pose.position.lat, \
                                   self._own_pose.pose.pose.position.lon, \
                                   self._own_pose.pose.pose.position.rel_alt, \
                                   (self._own_pose.pose.pose.orientation.x, \
                                    self._own_pose.pose.pose.orientation.y, \
                                    self._own_pose.pose.pose.orientation.z, \
                                    self._own_pose.pose.pose.orientation.w), \
                                   min_dist, self._fov_width*2, self._fov_height, \
                                   uav.state.pose.pose.position.lat, \
                                   uav.state.pose.pose.position.lon, \
                                   uav.state.pose.pose.position.alt):
                        min_dist = d
                        self._target_id = uav.vehicle_id

        # If a target has been identified, move towards it
        if self._target_id != None:
            #print("Target Aquired!")
            target_pos = self._reds[self._target_id].state.pose.pose.position

            own_lat = self._own_pose.pose.pose.position.lat
            own_lon = self._own_pose.pose.pose.position.lon
            tgt_lat = target_pos.lat
            tgt_lon = target_pos.lon

            lat = tgt_lat
            lon = tgt_lon

            if self._vehicle_type == enums.AC_FIXED_WING:
                # When using fixed wing, Set the waypoint past the current
                # target, so we don't go into a loiter mode
                bearing = gps.gps_bearing(own_lat, own_lon, tgt_lat, tgt_lon)
                lat, lon = gps.gps_newpos(own_lat, own_lon, bearing, 1000)


            # Determine if the target is within firing parameters
            if gps.hitable(self._own_pose.pose.pose.position.lat, \
                           self._own_pose.pose.pose.position.lon, \
                           self._own_pose.pose.pose.position.rel_alt, \
                           (self._own_pose.pose.pose.orientation.x, \
                            self._own_pose.pose.pose.orientation.y, \
                            self._own_pose.pose.pose.orientation.z, \
                            self._own_pose.pose.pose.orientation.w ), \
                           self._weapons_range, self._fov_width, self._fov_height,\
                           target_pos.lat, target_pos.lon, target_pos.rel_alt):
                self._action = ["Fire", self._target_id]


        #################################################################################
        #Safety to make sure you don't fly out of bounds
        rpy = qmath.quat_to_euler((self._own_pose.pose.pose.orientation.x,\
                                   self._own_pose.pose.pose.orientation.y, \
                                   self._own_pose.pose.pose.orientation.z, \
                                   self._own_pose.pose.pose.orientation.w))
        own_bearing = rpy[2]
        left_lat, left_lon = gps.gps_newpos(own_lat, own_lon, gps.normalize(own_bearing-(math.pi/2)), 20)
        right_lat, right_lon = gps.gps_newpos(own_lat, own_lon, gps.normalize(own_bearing+(math.pi/2)), 20)
        front_lat, front_lon = gps.gps_newpos(own_lat, own_lon, gps.normalize(own_bearing+(math.pi/2)), 50)
        if not self._bboxes[0].contains(left_lat, left_lon, own_alt) or \
                not self._bboxes[0].contains(right_lat, right_lon, own_alt) or \
                not self._bboxes[0].contains(front_lat, front_lon, own_alt):
            lat, lon = self.box_center


        self._wp = np.array([lat, lon, own_alt])

        return True

    def closestCorners(self):
        order = []
        uav_lat = self._own_pose.pose.pose.position.lat
        uav_lon = self._own_pose.pose.pose.position.lon
        for corner in self._safe_corners:
            if order == []:
                order.append(corner)
            else:
                index = 0
                inserted = False
                for otherCorner in order:
                    if not inserted and gps.gps_distance(uav_lat, uav_lon, corner[0],corner[1]) < gps.gps_distance(uav_lat, uav_lon, otherCorner[0],otherCorner[1]):
                       order.insert(index,corner)
                       inserted=True
                    index+=1

                if inserted == False:
                    order.append(corner)
        return order

    def interpolate(self,lat1,lon1,lat2,lon2, bias):
        #Interpolate lat/lon with bias towards point 1 (bias is 0 to 1)
        lat_sign = lat1/abs(lat1)
        lon_sign = lon1/abs(lon1)

        lat = lat2+bias*(lat1-lat2)
        lon = lon2+bias*(lon1-lon2)

        return lat,lon

    def set_safe_corners(self):
        raw_corners = self._bboxes[0].get_corners()
        safe_corners = []
        for corner in raw_corners:
            safe_corners.append(self.interpolate(corner[0], corner[1], self.box_center[0], self.box_center[1], 0.7))
        return safe_corners

    def fixed_wing_waypoint(self, own_lat, own_lon, waypoint_lat, waypoint_lon):
        bearing = gps.gps_bearing(own_lat, own_lon, waypoint_lat, waypoint_lon)
        return gps.gps_newpos(own_lat, own_lon, bearing, 1000)

    def check_for_supply_route(self):
        if self._enemy_territory.geography_contains(self.ordered_corners[0][0],self.ordered_corners[0][1]) and \
                self._enemy_territory.geography_contains(self.ordered_corners[1][0],self.ordered_corners[1][1]):
            print("Securing Supply Route!")
            self._state = States.kill_supply


    def receive_firing_report(self, msg):
        print '{} received that someone is targeting {}' \
            .format(self._id, msg.report.target_id)
