#!/bin/bash
# Starts darknet on the nanos

nanoIPs=(
    "192.168.11.50"
)

for ip in "${nanoIPs[@]}"
do
    if ping -c1 -w3 $ip >/dev/null 2>&1; then
        echo "Starting darknet on $ip"

        sh -c "sshpass -p 'RosUsr1!' ssh -o StrictHostKeyChecking=no $ip 'cd catkin_ws &&
            export ROS_MASTER_URI=http://192.168.11.147:11311/ &&
            export ROS_IP=$ip &&
            source ./devel/setup.bash &&
            rosrun swarms webcam.py
            '" &

        sh -c "sshpass -p 'RosUsr1!' ssh -o StrictHostKeyChecking=no $ip 'cd catkin_ws &&
            export ROS_MASTER_URI=http://192.168.11.147:11311/ &&
            export ROS_IP=$ip &&
            source ./devel/setup.bash &&
            rosrun swarms darknetOutput.py
            '" &

        sh -c "sshpass -p 'RosUsr1!' ssh -o StrictHostKeyChecking=no $ip 'cd catkin_ws &&
            export ROS_MASTER_URI=http://192.168.11.147:11311/ &&
            export ROS_IP=$ip &&
            source ./devel/setup.bash &&
            roslaunch darknet_ros darknet_ros.launch
            '"
    fi
done
