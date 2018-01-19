# UPDATE MISSION PARAMETERS ON GCS LAPTOP & VEHICLE ODROIDS (PREFLIGHT)

This page explains how to pull and push information from Github to keep common software configuration, how to create new mission parameters, and how to upload those mission parameters to the GCS laptop and UAS

### Pull & Push Updates to usma_swarm from Westpoint Robotics Github
1. Pull Latest Updates from usma_swarm/usma_files on Github
  * `cd ~/usma_swarm/usma_files`
  * `git pull`
2. Push latest updates to usma_swarm/usma_files on Github (if you make changes)
  * `cd ~/usma_swarm/usma_files`
  * `git add -A`
  * `git commit –m “comments”` (include comments for addition)
  * `git push`

### Create New "Blessed Files" with mission waypoints, geofence, and rally points
1. On laptop, navigate to /home/user1/usma_swam/usma_files/blessed_files
2. Navigate to proper mission folder (ex: Range11)
3. In that folder, properly update the following 6 files (see Waypoint_Key excel for more info):
  * fence.fen
  * rally_all.txt
  * quad_blue.wp
  * zeph_blue.wp
  * quad_red.wp
  * zeph_red.wp
4. Copy, paste, & replace those 6 files into /usma_swam/usma_files/blessed_files
5. Run update_templates_usma.py script in terminator
  * `cd ~/`
  * `./usma_swarm/usma_files/blessed_files/update_templates_usma.py`
  * This will create new files in the /home/user1/blessed directory
6. Check /home/user1/blessed directory to make sure the 4 files were updated

### Update AP_ENUMERATIONS and LOCATIONS files
1. Update ap_enumerations.py file (usually updated prior to a given mission)
  * Navigate to /home/user1/usma_swarm/usma_files/ap_enumerations_file
  * Open ap_enumerations.py
  * Select proper SITL_LOCATION in line 7 for mission location
  * Update other parameters as needed (be cautious in this step)
  * Save and close the file
  * Copy the ap_enumerations.py file, and paste/replace in /home/user1/ACS/acs_ros_ws/src/autonomy-payload/ap_lib/src/ap_lib
2. Update the locations.txt file (usually updated when going to new location
  * Navigate to /home/user1/usma_swarm/usma_files/locations_file 
  * Open locations.txt    
  * Add new location using same convention as others
  * Save and close the file  
  * Copy the locations.txt file, and paste/replace in /home/user1/ACS/ardupilot/Tools/autotest
  
