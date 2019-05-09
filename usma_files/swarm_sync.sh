#!/bin/bash
# Script written to update the drones in the swarm
# DIFFERENT FROM INL_PUSH
# Original Author: Conner Russell '19

# Dyanimcally ask for tail numbers and build ip addresses
read -p "Enter tail numbers (sperated by a space): " tails
echo "List of tails: $tails"

declare -a arr=()

for tail in $tails
   do
      echo "Adding 192.168.11.$tail"
      arr+=("192.168.11.$tail")
   done

# Staticly Create an array of drone ip addresses
#declare -a arr=("192.168.11.132" "192.168.11.131" "192.168.11.114" "192.168.11.110") 

# Declare var used for tracking what # drone in the array is being configured
declare -i NDRONE=0

# For each quadcopter update the listed directories
for i in "${arr[@]}"
    do
        if ping -c1 -w3 $i >/dev/null 2>&1; then
            # Update blessed folder with specific drone launch file
            echo "Creating WP file for drone # $NDRONE"
            ./blessed_files/dynamic_update_templates_usma.py quadWP_$NDRONE.wp
            NDRONE=$NDRONE+1

            # If not working, del the directory before using scp to copy over

            # Copying the navigation files over
            echo "Updating $i"
            sshpass -p "odroid" ssh -o StrictHostKeyChecking=no odroid@$i rm -rf blessed
            sshpass -p "odroid" scp -rp -o StrictHostKeyChecking=no $HOME/blessed odroid@$i:
            
            # Copying the behaviors over
            sshpass -p "odroid" ssh -o StrictHostKeyChecking=no odroid@$i rm -rf usma/plugins/autonomy/python
            sshpass -p "odroid" scp -rp -o StrictHostKeyChecking=no $HOME/scrimmage/usma/plugins/autonomy/python/ odroid@$i:scrimmage/usma/plugins/autonomy/
            
            # Copying the extra parameters over
            sshpass -p "odroid" ssh -o StrictHostKeyChecking=no odroid@$i rm acs_ros_ws/src/autonomy-payload/ap_lib/src/ap_lib/ap_enumerations.py
            sshpass -p "odroid" scp -rp -o StrictHostKeyChecking=no $HOME/ACS/acs_ros_ws/src/autonomy-payload/ap_lib/src/ap_lib/ap_enumerations.py odroid@$i:acs_ros_ws/src/autonomy-payload/ap_lib/src/ap_lib
            
            echo "COMPLETED UPDATING OF $i"
         else
            echo "COULD NOT FIND $i"
            echo "Team RAIDERS forever!"
         fi
            
    done
