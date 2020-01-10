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

### Update Mission Parameters onto the Vehicles
1. Configure Alfa Wi-Fi dongle on SASC Linux Laptop  
  * Attach an Alfa USB Wi-Fi adapter to the computer  
  * `ifconfig` - Identify the Alfa Wi-Fi adapter (usually the last one) (ex: wlx00c0ca904414, or wlan2)  
  * `wifi_config.sh -T 11 wlx00c0ca904414 201` 
    - Note: 11 is the Team # (11 = Army), 201 is the selected network address for the computer (201-209 are recommended, make sure no other SASC computers are set to that).     
    - "11" is the team # (11 = Army)    
    - "201" is the # w/i the team (201-209 recommended for dongles, must be unique from other active SASC computers)    
    - This will set this computers the Alfa Wifi IP address to 192.168.11.201
2. Ping the powered up UAS to ensure you can talk to it: `ping 192.168.11.X` (X = tail number, ex: 112)
3. Copy BLESSED folder to ODROID ROOT
  * `scp -r /home/user1/blessed odroid@192.168.11.X:/home/odroid/`(X = tail number, ex: 112)
4. Copy PYTHON behaviors to ODROID behavior directory
  * Make sure scrimmage/usma up to date
     - `ssh-add ~/.ssh/swarms_id_rsa` (if DI2E and not NPS Gitlab)
     - `cd ~/scrimmage/usma`
     - `git pull`
  * `scp -r /home/user1/scrimmage/usma/plugins/autonomy/python/ odroid@192.168.11.X:/home/odroid/scrimmage/usma/plugins/autonomy/`
5. Copy AP_ENUMERATIONS to proper ODROID directory
  * `scp /home/user1/ACS/acs_ros_ws/src/autonomy-payload/ap_lib/src/ap_lib/ap_enumerations.py odroid@192.168.11.X:/home/odroid/acs_ros_ws/src/autonomy-payload/ap_lib/src/ap_lib`
