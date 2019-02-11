# USMA_FILES Swarm_Push Update

### New Important Files
1. swarm_sync.sh
2. blessed_files/dynamic_update_templates_usma.py
3. blessed_files/update_templates_usma.py
4. blessed_files/clean_waypoints.sh
5. blessed_files/launch.csv

# UPDATE MISSION PARAMETERS ON GCS LAPTOP & VEHICLE ODROIDS (PREFLIGHT)

This page explains how to pull and push information from Github to keep common software configuration, how to create new mission parameters, and how to upload those mission parameters to the GCS laptop and UAS

### Pull & Push Updates to usma_swarm from Westpoint Robotics Github
### Change current branch from master -> launchBranch
1. Pull Latest Updates from usma_swarm/usma_files on Github
  * `cd ~/usma_swarm/usma_files`
  * `git pull`
2. Change git branch to track launchBranch
  * `git checkout launchBranch`
3. Push latest updates to usma_swarm/usma_files on Github (if you make changes)
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
5. Open & Edit launch.csv
  * Edit start & end lat/long of launch line
  * Change # of drones being launched
6. Run wpDuplicate.py to create unique wp files
  * `./wpDuplicate.py launch.csv`
  *This will create new & unique wp files for each drone in the current directory
  *To remove these files for clean-up purposes, run:
  * `./clean_waypoints.sh`
7. Navigate back to the usma_files directory
  * `cd ..`
8. Update swarm_sync.sh line 12 with correct array of drone IP addresses
    - Before running, ensure that the wifi adapter is properly set up (see step 1 of update mission parameters onto vehicles)
9. Run swarm_sync.sh script in terminator
  * `./swarm_sync.sh`
  * This will create new files in the /home/user1/blessed directory, then sync those files to each of the drones odroid folders
10. Check each odroid folder to ensure proper syncing of individual files

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

### Update Mission Parameters onto the Vehicles
1. Configure Alfa Wi-Fi dongle on SASC Linux Laptop  
  * Attach an Alfa USB Wi-Fi adapter to the computer  
  * `ifconfig` - Identify the Alfa Wi-Fi adapter (usually the last one) (ex: wlx00c0ca904414, or wlan2)  
  * `wifi_config.sh -T 11 wlx00c0ca904414 201` 
    - Note: 11 is the Team # (11 = Army), 201 is the selected network address for the computer (201-209 are recommended, make sure no other SASC computers are set to that).     
    - "11" is the team # (11 = Army)    
    - "201" is the # w/i the team (201-209 recommended for dongles, must be unique from other active SASC computers)    
    - This will set this computers the Alfa Wifi IP address to 192.168.11.201

  * If using swarm_sync.sh, stop here*!*!
2. Ping the powered up UAS to ensure you can talk to it: `ping 192.168.11.X` (X = tail number, ex: 112)
3. Copy BLESSED folder to ODROID ROOT
  * `scp -r /home/user1/blessed odroid@192.168.11.X:/home/odroid/`(X = tail number, ex: 112)
4. Copy PYTHON behaviors to ODROID behavior directory
  * `scp -r /home/user1/scrimmage/usma/plugins/autonomy/python/ odroid@192.168.11.X:/home/odroid/scrimmage/usma/plugins/autonomy/`
5. Copy AP_ENUMERATIONS to proper ODROID directory
  * `scp /home/user1/ACS/acs_ros_ws/src/autonomy-payload/ap_lib/src/ap_lib/ap_enumerations.py odroid@192.168.11.X:/home/odroid/acs_ros_ws/src/autonomy-payload/ap_lib/src/ap_lib`
