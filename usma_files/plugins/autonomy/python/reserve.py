'''
Tactic Name: Reserve (Reserve)
Description:  This tactic is a defensive tactic that sends the UASs to a home location to loiter.
                If an enemy is detected within friendly airspace, the UAS will pursue the enemy like a
                greedy shooter.  After destroying the enemy, it will return to the home location. It uses
                the waypoints that are defined in usma_enumerations.py that form at 4x4 grid with WP 0 at
                the NW corner of the battlecube, WP 3 at the SW corner, and continuing until WP 15 at the
                SE corner of the battlecube.

                |===================|
                |   0   4   8   12  |
                |   1   5   9   13  |
                |   2   6  10   14  |
                |   3   7  11   15  |
                |===================|

Date:		27 Apr 2017
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

class Reserve(ss.Tactic):

    def init(self, params):
        self._id = int(params['id'])
        self._target_id = -1
        self._last_target = -1
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
        self._vehicle_type = int(params['vehicle_type'])
        self._name = 'Reserve'
        self._home = int(params['home'])
        self._desired_lat = float(usma_enums.WP_LOC[self._home][0])
        self._desired_lon = float(usma_enums.WP_LOC[self._home][1])
        self._pursue = 0

    def dist_calc(self, tlat, tlon):
      R = 6.371 * 1000000
      lat1 = math.radians(self._desired_lat)
      lon1 = math.radians(self._desired_lon)
      lat2 = math.radians(tlat)
      lon2 = math.radians(tlon)
      dlon = lon2 - lon1
      dlat = lat2 - lat1
      a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
      c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
      distance = R * c
      return distance

    def self_dist(self, tlat, tlon):
      R = 6.371 * 1000000
      lat1 = math.radians(self._own_pose.pose.pose.position.lat)
      lon1 = math.radians(self._own_pose.pose.pose.position.lon)
      lat2 = math.radians(tlat)
      lon2 = math.radians(tlon)
      dlon = lon2 - lon1
      dlat = lat2 - lat1
      a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
      c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
      distance = R * c
      return distance
        
    def step_autonomy(self, t, dt):
      self._action = ["None", 0]
      if self._target_id in self._shot:
        self._target_id = -1
      #dont excede max fly dist
      #break and check targets
      sdist = self.dist_calc(self._own_pose.pose.pose.position.lat, self._own_pose.pose.pose.position.lon)
      if sdist > 100:
        print "Exceded fly zone. Returning to holding pattern"
        sdist = self.dist_calc(self._own_pose.pose.pose.position.lat, self._own_pose.pose.pose.position.lon)
        self._target_id = -1
        self._wp = np.array([self._desired_lat, self._desired_lon, self._last_ap_wp[2]])
        return True 

      #hunt current target
      if self._target_id != -1 and self._pursue <= 50:
        print "Continue hunting %d for %d more cycles" % (self._target_id, 50-self._pursue)
        self._pursue += 1
        #check if target outside max dist
        target_pos = self._reds[self._target_id].state.pose.pose.position
        tgt_lat = target_pos.lat
        tgt_lon = target_pos.lon
        tdist = self.dist_calc(tgt_lat, tgt_lon)
        #if target outside airspace, clear and check for other targets
        if tdist > 150:
          print "%d left airspace; breaking contact" % self._target_id
          self._target_id = -1
          self._pursue = 0

        #target still within pursue distance, check if firing solution available        
        else:
          own_lat = self._own_pose.pose.pose.position.lat
          own_lon = self._own_pose.pose.pose.position.lon
          own_alt = self._own_pose.pose.pose.position.rel_alt
          tgt_lat = target_pos.lat
          tgt_lon = target_pos.lon
          tgt_alt = target_pos.rel_alt
          bearing = gps.gps_bearing(own_lat, own_lon, tgt_lat, tgt_lon)
          lat, lon = gps.gps_newpos(own_lat, own_lon, bearing, 1000)
          self._wp = np.array([lat, lon, own_alt])
          #if firing solution, take shot
          if gps.hitable(self._own_pose.pose.pose.position.lat, \
                         self._own_pose.pose.pose.position.lon, \
                         self._own_pose.pose.pose.position.rel_alt, \
                         (self._own_pose.pose.pose.orientation.x, \
                         self._own_pose.pose.pose.orientation.y, \
                         self._own_pose.pose.pose.orientation.z, \
                         self._own_pose.pose.pose.orientation.w ), \
                         self._max_range, self._fov_width, self._fov_height,\
                         target_pos.lat, target_pos.lon, target_pos.rel_alt):
            print "RESERVE===========================> %d Firing on %d" % (self._id, self._target_id)
            self._action = ["Fire", self._target_id]
            self._pursue = 0
          return True

      #pursue exceded, reset and find new target
      if self._pursue > 50:
        self._pursue = 0
        print "Pursue exceeded: reset target"
        self._target_id = -1

      #check if any aircraft in airspace
      if self._target_id == -1:
        print "Checking for hostiles in airspace"
        target = -1
        minDist = 1000
        #find closest hostile within attack range
        for target_id, red in self._reds.iteritems():
            if (red.game_status == enums.GAME_ACTIVE_DEFENSE or red.game_status == enums.GAME_ACTIVE_OFFENSE):
              target_pos = self._reds[target_id].state.pose.pose.position
              if gps.hitable(self._own_pose.pose.pose.position.lat, \
                             self._own_pose.pose.pose.position.lon, \
                             self._own_pose.pose.pose.position.rel_alt, \
                             (self._own_pose.pose.pose.orientation.x, \
                             self._own_pose.pose.pose.orientation.y, \
                             self._own_pose.pose.pose.orientation.z, \
                             self._own_pose.pose.pose.orientation.w ), \
                             self._max_range, self._fov_width, self._fov_height,\
                             target_pos.lat, target_pos.lon, target_pos.rel_alt):
                self._target_id = target_id
                self._action = ["Fire", self._target_id]
                print "RESERVE===========================> %d Firing on %d" % (self._id, self._target_id)
                return True
              dist = self.dist_calc(red.state.pose.pose.position.lat, red.state.pose.pose.position.lon)
              prox = self.self_dist(red.state.pose.pose.position.lat, red.state.pose.pose.position.lon)
              if dist < 150 and target_id not in self._shot and prox < minDist:
                target = target_id
                minDist = dist
            self._target_id = target
            self._pursue = 0

      #if this code is hit, no hostile aircraft in airspace, and still in max range
      #return home and end step
      self._wp = np.array([self._desired_lat, self._desired_lon, self._last_ap_wp[2]])
      return True
