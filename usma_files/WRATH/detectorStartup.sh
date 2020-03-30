#!/bin/bash
# Starts detector

read -p "Enter tail number: " tail
echo "Starting detector on 192.168.11.$tail"

if ping -c1 -w3 192.168.11.$tail >/dev/null 2>&1; then

    echo "Starting rad detector"
    sh -c "sshpass -p 'odroid' ssh -o StrictHostKeyChecking=no odroid@192.168.11.$tail 'cd ~/acs_ros_ws &&
        source ./devel/setup.bash &&
        rosrun ap_safety_monitor acquireStdMsg.py
        '"
fi
