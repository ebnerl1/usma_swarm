#!/bin/bash
# Starts altitude sensor

read -p "Enter tail number: " tail
echo "Starting laser range finder on 192.168.11.$tail"

if ping -c1 -w3 192.168.11.$tail >/dev/null 2>&1; then

    echo "Starting laser range finder"
    sh -c "sshpass -p 'odroid' ssh -o StrictHostKeyChecking=no odroid@192.168.11.$tail 'cd ~/acs_ros_ws &&
        source ./devel/setup.bash &&
        rosrun ap_safety_monitor altitude_eye_ROS.py
        '"

fi