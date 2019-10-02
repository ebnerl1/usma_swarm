# Savidge Swarm

### List of files for running a Savidge Swarm simulation:
Note: see SavidgeFiles directory for the files listed below. Then copy them to the required location.

1. savidgeswarm_simulations.xml – file that actually gets run in SCRIMMAGE
	- modify: number of UAVs, starting location of UAVs
    - location: /home/user1/scrimmage/usma/missions
2. trial_enumerations.py – creates heatmap
    - modify: grid locations, number of waypoints, distance between waypoints, location of hotspot, score of hotspot, default map score
    - location: /home/user1/scrimmage/usma/plugins/autonomy/python
3. savidgeswarm_v2_simulations.py – contains actual behavior
	- modify: time between re-planning, propagation values
    - location: /home/user1/scrimmage/usma/plugins/autonomy/python
4. Savidgeswarm_simulations2.xml – creates the objects from the class of savidgeswarm_v2_simulations
    - NOTE: to differentiate from 1, added 2 in file name.  If doesn't work, try deleting 2 in name.	
    - modify: nothing
    - location: /home/user1/scrimmage/usma/plugins/autonomy/config
5. swarm_helper_v2.py – used for sending messages
	- modify: nothing
    - location: /home/user1/ACS/acs_ros_ws/src/autonomy-payload/ap_lib/src/ap_lib 
6. Simulations.csv – output file
	-location: /home/user1/scrimmage/scrimmage
7. savidgeswarm_v3_implementation.py – contains actual behavior for flight test
    - Note: not for simulation, this is for live flight test	
    - modify: time between re-planning, propagation values
    - location: /home/user1/scrimmage/usma/plugins/autonomy/python

### Steps to run SCRIMMAGE on Linux:
1. cd scrimmage
2. cd scrimmage
3. scrimmage ../usma/missions/(insert .xml file here)
	Ex. scrimmage ../usma/missions/savidgeswarm_simulations.xml

### SCRIMMAGE Commands:
- ‘b’ - Pause/Resume
- ‘spacebar’ - while paused, increment 0.1 seconds
- ‘]’ - speed up
- ‘[‘ - slow down
- ‘right/left arrow keys’ - switch to other drones’ views
- ‘a’ - rotate through camera views
