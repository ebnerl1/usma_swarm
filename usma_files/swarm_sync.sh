#!/bin/bash
# Script written an INL to update the drones in the swarm

# Create an array of drone ip addresses
#declare -a arr=("192.168.11.110" "192.168.11.111" "192.168.11.112" "192.168.11.113" "192.168.11.114" "192.168.11.115" "192.168.11.116" "192.168.11.117" "192.168.11.118" "192.168.11.119" "192.168.11.120" "192.168.11.121" "192.168.11.122" "192.168.11.123" "192.168.11.124" "192.168.11.125" "192.168.11.126" "192.168.11.127" "192.168.11.128" "192.168.11.129" "192.168.11.130" "192.168.11.131" "192.168.11.132")

# Uncomment below line to send to Zephyrs, this overrides the above the declare for the quads.
#declare -a arr=("192.168.11.10" "192.168.11.11" "192.168.11.12" "192.168.11.13" "192.168.11.14" "192.168.11.15" "192.168.11.16" "192.168.11.17" "192.168.11.18" "192.168.11.19" "192.168.11.20" "192.168.11.21" "192.168.11.22" "192.168.11.23" "192.168.11.24" "192.168.11.25" "192.168.11.26" "192.168.11.27" "192.168.11.28" "192.168.11.29" "192.168.11.30" "192.168.11.31" "192.168.11.32")

# A smaller list for testing

declare -a arr=("192.168.11.111" "192.168.11.113" "192.168.11.115" "192.168.11.120" "192.168.11.121" "192.168.11.122" "192.168.11.123" "192.168.11.125") 
#declare -a arr=("192.168.11.111")

# Below is the previous commit of the master branch before the EOY pull request 
#declare -a arr=("192.168.11.125") 
# end 

#Declare var used for tracking what # drone in the array is being configured
NDRONE = 0

# For each quadcopter update the listed directories
for i in "${arr[@]}"
    do
        if ping -c1 -w3 $i >/dev/null 2>&1; then
            #Update blessed folder with specific drone launch file
            ./blessed_files/dynamic_update_templates.usma.py quadWP_$NDRONE.wp

            #Remove the directory before using scp to copy over
            echo "Updating $i"
            sshpass -p "odroid" ssh -o StrictHostKeyChecking=no odroid@$i rm -rf blessed
            sshpass -p "odroid" scp -rp -o StrictHostKeyChecking=no $HOME/blessed odroid@$i:
            
            sshpass -p "odroid" ssh -o StrictHostKeyChecking=no odroid@$i rm -rf usma/plugins/autonomy/python
            sshpass -p "odroid" scp -rp -o StrictHostKeyChecking=no $HOME/scrimmage/usma/plugins/autonomy/python/ odroid@$i:scrimmage/usma/plugins/autonomy/
            
            sshpass -p "odroid" ssh -o StrictHostKeyChecking=no odroid@$i rm acs_ros_ws/src/autonomy-payload/ap_lib/src/ap_lib/ap_enumerations.py
            sshpass -p "odroid" scp -rp -o StrictHostKeyChecking=no $HOME/ACS/acs_ros_ws/src/autonomy-payload/ap_lib/src/ap_lib/ap_enumerations.py odroid@$i:acs_ros_ws/src/autonomy-payload/ap_lib/src/ap_lib
            
            echo "COMPLETED UPDATING OF $i"
         else
            echo "COULD NOT FIND $i"
         fi
            
    done
