#!/usr/bin/env python

import ap_lib.gps_utils as gps
import math

# Contains enumerations and constants for use throughout the ACS Python architecture

# Fixed (per convention) state-specific waypoints
TAKEOFF_WP = 1       # Airborne
INGRESS_LOITER_WP = 3 # Ingress loiter for time waypoint at constant 75m altitude
INGRESS_CYLINDER_WP = 4 # Ingress loiter to altitude waypoint at calculated alt slot
SWARM_STANDBY_WP = 5 # Available for tasking
SWARM_EGRESS_WP = 6  # Leaving swarm for recovery  NOTE:  Not used right now
RACETRACK_WP = 7     # First racetrack waypoint
LAND_A_WP = 20       # First WP in the east-to-west (RW27) landing sequence
LAND_B_WP = 27       # First WP in the west-to-east (RW10) landing sequence
ATTACK_WP = 34       # First WP in the ground attack landing sequence

# Default Values. These get updated on startup and again on flight ready
MIN_REL_ALT = 70.0     # Minimum relative altitude that a controller can order
MAX_REL_ALT = 1500.0   # Maximum relative altitude that a controller can order
MAX_ABS_LAT = 60.0     # Maximum absolute value that is commandable to lat
MAX_ABS_LON = 180.0    # Maximum absolute value that is commandable to lon
MAX_ROLL = math.radians(60)  # Maximum commandable bank angle (phi)
MAX_PITCH = math.radians(30) # Maximum commandable pitch angle (theta)
MAX_YAW_RATE = 1.0     # Maximum commandable yaw rate
MAX_PITCH_RATE = 1.0   # Maximum commandable pitch rate
MIN_FWD_SPEED = 15.0   # Minimum commandable forward speed
MAX_FWD_SPEED = 23.0   # Maximum commandable forward speed
BASE_REL_ALT = 100.0   # Base rel_alt for "stacked" behaviors
ALT_BLOCK_SIZE = 10.0  # Altitude block size for altitude-separated behaviors

# Enumeration for available swarm behaviors
SWARM_STANDBY = 0          # No swarm behavior (set no payload control)
SWARM_LINEAR_FORMATION = 1 # Straight line high-to-low formation
SWARM_SEARCH = 2           # Conduct a coordinated search of a specified area
NAIVE_SHOOTER = 3          # Shoot closest red UAV & ignore other UAV results
GREEDY_SHOOTER = 4         # Shoot the closest red UAV but use other UAV results
PN_INTERCEPTOR = 5         # Proportional navigation intercept of a target UAV
ALTITUDE_SORT = 6          # Consensus algorithm to sort UAVs by rel_alt
LAZY_ALTITUDE_SORT = 7     # Lazy consensus algorithm to sort UAVs by rel_alt
INDEPENDENT_TRANSIT = 8    # Transit independently to a specified location
SMART_SHOOTER = 9          # Shoot the closest red UAV but use other UAV results and dont pursue enemy already being pursued by a friendly
CONSTRAINED_SHOOTER = 10   # Built on Smart Shooter - Constrained FOV Angle and Range of engaging targets
WINGMAN = 11               # Combination of Linear Formation and Constrained Shooter Behaviors
PATROL_BOX = 12            # Randomly patrol a rectangular box
PATROL_BOX_SHOOTER = 13    # Randomly patrol a rectangular box & engage approaching red UAVs
TACTIC_INTERFACE = 14      # Use generic tactic interface
EVADER = 15                # Move to a waypoint while avoiding enemies
SIMPLE_GRND_ATTACK = 16    # Basic ground attack behavior (just land there)
FIXED_TURN = 17            # Simple test behavior for turn-rate control capability
SWARM_SEQUENCE_LAND = 98   # Land in order (low-to-high UAV)
SWARM_EGRESS = 99          # Egress the swarm for recovery

# Mapping between swarm behaviors and strings for GUI use or debugging
SWARM_BHVRS = {  SWARM_STANDBY:          'Standby', \
                 SWARM_LINEAR_FORMATION: 'Line Formation', \
                 SWARM_SEARCH:           'Swarm Search', \
                 NAIVE_SHOOTER:          'Naive Shooter', \
                 GREEDY_SHOOTER:         'Greedy Shooter', \
                 SMART_SHOOTER:          'Smart Shooter', \
                 CONSTRAINED_SHOOTER:    'Constrained Shooter', \
                 WINGMAN:                'Wingman', \
                 PN_INTERCEPTOR:         'PN Interceptor', \
                 ALTITUDE_SORT:          'Eager Altitude Sort', \
                 LAZY_ALTITUDE_SORT:     'Lazy Altitude Sort', \
                 SWARM_SEQUENCE_LAND:    'Sequence Land', \
                 INDEPENDENT_TRANSIT:    'Independent Transit', \
                 PATROL_BOX:             'Patrol Box', \
                 TACTIC_INTERFACE:       'Tactic Interface', \
                 EVADER:                 'Evader', \
                 PATROL_BOX_SHOOTER:     'Patrol Box Shooter', \
                 SIMPLE_GRND_ATTACK:     'Simple Grnd Attack', \
                 FIXED_TURN:             'Fixed Turn' }

SWARM_BHVR_VALUES = { 'Standby':             SWARM_STANDBY, \
                      'Line Formation':      SWARM_LINEAR_FORMATION, \
                      'Swarm Search':        SWARM_SEARCH, \
                      'Naive Shooter':       NAIVE_SHOOTER, \
                      'Greedy Shooter':      GREEDY_SHOOTER, \
                      'Smart Shooter':       SMART_SHOOTER, \
                      'Constrained Shooter': CONSTRAINED_SHOOTER, \
                      'Wingman':             WINGMAN, \
                      'PN Interceptor':      PN_INTERCEPTOR, \
                      'Eager Altitude Sort': ALTITUDE_SORT, \
                      'Lazy Altitude Sort':  LAZY_ALTITUDE_SORT, \
                      'Sequence Land':       SWARM_SEQUENCE_LAND, \
                      'Independent Transit': INDEPENDENT_TRANSIT, \
                      'Patrol Box':          PATROL_BOX, \
                      'Patrol Box Shooter':  PATROL_BOX_SHOOTER, \
                      'Evader':              EVADER, \
                      'Tactic Interface' :   TACTIC_INTERFACE, \
                      'Simple Grnd Attack':  SIMPLE_GRND_ATTACK, \
                      'Fixed Turn':          FIXED_TURN }

# Enumeration for swarming states
PRE_FLIGHT = 0    # Powered on, going through pre-fllight checks
FLIGHT_READY = 1  # Awaiting launch
INGRESS = 2       # Airborne, waiting for handoff to swarm operator
SWARM_READY = 3   # Available for swarm behavior
OUT_OF_GAME = 4   # Swarming available, but UAV is hit or penalized
LANDING = 5       # Flight crew has control for landing
ON_DECK = 6       # Aircraft has landed
POST_FLIGHT = 7   # Post landing checks (will probably not be seen)
AP_ERROR = 8      # Error state (probably due to wrong autopilot mode)
LAST_STATE = 15   # CANNOT EXCEED 15; network message field is a nibble

# Mapping between swarm states and strings for GUI use or debugging
STATE_STRINGS = { PRE_FLIGHT:   'Preflight', \
                  FLIGHT_READY: 'Flight Ready', \
                  INGRESS:      'Ingress', \
                  SWARM_READY:  'Swarm Ready', \
                  OUT_OF_GAME:  'Out of Game', \
                  LANDING:      'Landing', \
                  ON_DECK:      'On Deck',
                  POST_FLIGHT:  'Post Flight',
                  AP_ERROR:     'State Error' }

STATE_VALUES = { 'Preflight':    PRE_FLIGHT, \
                 'Flight Ready': FLIGHT_READY, \
                 'Ingress':      INGRESS, \
                 'Swarm Ready':  SWARM_READY, \
                 'Out of Game':  OUT_OF_GAME, \
                 'Landing':      LANDING, \
                 'On Deck':      ON_DECK, \
                 'Post Flight':  POST_FLIGHT,
                 'State Error':  AP_ERROR }

# Enumeration for autopilot modes
RTL = 0
MANUAL = 1
FBWA = 2
GUIDED = 3
AUTO = 4
FBWB = 5
CIRCLE = 6
LOITER = 7
INITIALIZING = 8
STABILIZE = 9
UNMAPPED = 15

# Enumeration for aircraft types
AC_UNKNOWN      = 0
AC_COPTER       = 1
AC_FIXED_WING   = 2

# Mapping between autopilot modes and strings for GUI use or debugging
MODE_STRINGS = { RTL:      'RTL', \
                 MANUAL:   'MANUAL', \
                 FBWA:     'FBWA', \
                 GUIDED:   'GUIDED', \
                 AUTO:     'AUTO', \
                 FBWB:     'FBWB', \
                 CIRCLE:   'CIRCLE', \
                 LOITER:   'LOITER', \
                 INITIALIZING: 'INITIALIZING', \
                 STABILIZE: 'STABILIZE', \
                 UNMAPPED: 'UNMAPPED' }

MODE_VALUES = { 'RTL':      RTL, \
                'MANUAL':   MANUAL, \
                'FBWA':     FBWA, \
                'GUIDED':   GUIDED, \
                'AUTO':     AUTO, \
                'FBWB':     FBWB, \
                'CIRCLE':   CIRCLE, \
                'LOITER':   LOITER, \
                'INITIALIZING' : INITIALIZING, \
                'STABILIZE' : STABILIZE, \
                'UNMAPPED': UNMAPPED }
#UNMAPPED = ACRO, LOITER, INITIALIZING, TRAINING, STABILIZE, CRUISE

# Enumeration for types of waypoints that we might need to test for
WP_TYPE_NORMAL = 16
WP_TYPE_LOITER = 17
WP_TYPE_TURNS = 18
WP_TYPE_TIME = 19
WP_TYPE_LAND = 21
WP_TYPE_TAKEOFF = 22
WP_TYPE_LOITER_TO_ALT = 31
WP_TYPE_LAND_SEQUENCE = 189
WP_TYPE_ENABLE_FENCE = 207

# Default IDs for ground stations and other non-flying network participants
ARBITER_ID = 211
SWARM_CDR_ID = 250


#********************************
# Swarm Challenge Game Parameters
#********************************

# Enumeration and mappings for game modes
GAME_INACTIVE = 0        # UAV is alive but outside of the battle cube
GAME_ACTIVE_DEFENSE = 1  # UAV is airborn and "live"
GAME_ACTIVE_OFFENSE = 2  # UAV is airborn and "live"
GAME_HIT = 3             # UAV has been "shot"
GAME_PENALTY = 4         # UAV is serving a penalty
GAME_CRASH = 5           # UAV has crashed
GAME_END = 6             # Game has ended

GAME_MODE_STRINGS = { GAME_INACTIVE:       'Inactive',
                      GAME_ACTIVE_DEFENSE: 'Active (D)',
                      GAME_ACTIVE_OFFENSE: 'Active (O)',
                      GAME_HIT:            'Inactive (hit)',
                      GAME_PENALTY:        'Inactive (penalty)',
                      GAME_CRASH:          'Inactive (crash)',
                      GAME_END:            'Inactive (game end)' }

GAME_MODE_VALUES = { 'Inactive':            GAME_INACTIVE,
                     'Active (D)':          GAME_ACTIVE_DEFENSE,
                     'Active (O)':          GAME_ACTIVE_OFFENSE,
                     'Inactive (hit)':      GAME_HIT,
                     'Inactive (penalty)':  GAME_PENALTY,
                     'Inactive (crash)':    GAME_CRASH,
                     'Inactive (game end)': GAME_END }

# Battle Arena Parameters
BATTLE_CUBE_SW_LAT = 35.720680    # Latitude of the battle cube SW corner
BATTLE_CUBE_SW_LON = -120.771775  # Longitude of the battle cube SW corner
BATTLE_CUBE_LENGTH = 500          # N/S dimension (meters) of the battle cube
BATTLE_CUBE_WIDTH = 500           # E/W dimension (meters) of the battle cube
BATTLE_CUBE_ORIENT = 25.183537917993224   # Battle cube orientation (clockwise degrees)
BATTLE_CUBE_MIN_ALT = 354         # Battle cube floor (meters MSL)
BATTLE_CUBE_MAX_ALT = 854         # Battle cube ceiling (meters MSL)
BATTLE_CUBE_CTR_SW_LAT, BATTLE_CUBE_CTR_SW_LON = \
    gps.gps_newpos(BATTLE_CUBE_SW_LAT, BATTLE_CUBE_SW_LON, \
                   math.radians(BATTLE_CUBE_ORIENT) + math.pi/2.0, \
                   BATTLE_CUBE_WIDTH / 2.0) 
STAGE_CUBE_WIDTH = 200
BLUE_STAGE_SW_LAT = 35.7214444888298
BLUE_STAGE_SW_LON = -120.77377763463352
RED_STAGE_SW_LAT = 35.718768632775195
RED_STAGE_SW_LON = -120.76676858155359

def get_battleboxes():
    ''' Utility function for returning a tuple of relevant battle cube objects
    @return tuple of objects as follows (all GeoBox exc centerline
            which is a tuple of (lat, lon) endpoints): 
            (battleBox, centerline, blueBox, blueStage, redBox, redStage)
    '''
    bBox = gps.GeoBox(BATTLE_CUBE_SW_LAT, BATTLE_CUBE_SW_LON, \
                      BATTLE_CUBE_LENGTH, BATTLE_CUBE_WIDTH, \
                      BATTLE_CUBE_ORIENT, BATTLE_CUBE_MIN_ALT, \
                      BATTLE_CUBE_MAX_ALT)
    ctr1 = gps.gps_newpos(BATTLE_CUBE_SW_LAT, \
                          BATTLE_CUBE_SW_LON, \
                          math.radians(BATTLE_CUBE_ORIENT) + math.pi/2.0, \
                          BATTLE_CUBE_WIDTH / 2.0)
    ctr2 = gps.gps_newpos(ctr1[0], ctr1[1],
                          math.radians(BATTLE_CUBE_ORIENT), \
                          BATTLE_CUBE_LENGTH)
    centerLine = (ctr1, ctr2)
    blueBox = gps.GeoBox(BATTLE_CUBE_SW_LAT, BATTLE_CUBE_SW_LON, \
                         BATTLE_CUBE_LENGTH / 2.0, BATTLE_CUBE_WIDTH, \
                         BATTLE_CUBE_ORIENT, BATTLE_CUBE_MIN_ALT, \
                         BATTLE_CUBE_MAX_ALT)
    blueStage = gps.GeoBox(BLUE_STAGE_SW_LAT, BLUE_STAGE_SW_LON, \
                           BATTLE_CUBE_LENGTH, STAGE_CUBE_WIDTH, \
                           BATTLE_CUBE_ORIENT, 0.0, BATTLE_CUBE_MAX_ALT)
    redBox_SW_Corner = gps.gps_newpos(BATTLE_CUBE_SW_LAT, \
                                      BATTLE_CUBE_SW_LON, \
                                      math.radians(BATTLE_CUBE_ORIENT) + math.pi / 2.0, \
                                      BATTLE_CUBE_WIDTH / 2.0)
    redBox = gps.GeoBox(redBox_SW_Corner[0], redBox_SW_Corner[1], \
                        BATTLE_CUBE_LENGTH, BATTLE_CUBE_WIDTH / 2.0, \
                        BATTLE_CUBE_ORIENT, BATTLE_CUBE_MIN_ALT, \
                        BATTLE_CUBE_MAX_ALT)
    redStage = gps.GeoBox(RED_STAGE_SW_LAT, RED_STAGE_SW_LON, \
                          BATTLE_CUBE_LENGTH, STAGE_CUBE_WIDTH, \
                          BATTLE_CUBE_ORIENT, 0.0, BATTLE_CUBE_MAX_ALT)
    return tuple((bBox, centerLine, blueBox, blueStage, redBox, redStage))


# Air-to-air & air-to-ground targeting envelope parameters (pretty large envelope for now)
MAX_RANGE_DEF = 150.0   # Defending UAV maximum weapons range
HFOV_DEF = 15.0         # Defending UAV horizontal angle-off-the-bow weapons envelope
VFOV_DEF = 180.0        # 180 degree vertical cutout in front of UAV (2D solution)

MAX_RANGE_OFF = 150.0   # Attacking UAV maximum weapons range
HFOV_OFF = 15.0         # Attacking UAV horizontal angle-off-the-bow weapons envelope
VFOV_OFF = 180.0        # Attacking UAV horizontal angle-off-the-bow weapons envelope

A2G_RANGE = 50.0  # Required distance from target for air-to-ground hit

# Scoring parameters
A2A_SCORE = 3.2               # Points per air-to-air hit
A2G_SCORE = 1.0               # Points per live land at adversary goal
ALIVE_SCORE = 3.8             # Points per UAV * live_time / event_duration
EVENT_DURATION = 20.0 * 60.0  # Duration of the event (seconds)
GOAL_POSITS = dict()          # lat/lon of the defended "goals"
GOAL_POSITS['blue'] = ( 35.722783, -120.769350 )
GOAL_POSITS['red'] = ( 35.721602, -120.766319 )

