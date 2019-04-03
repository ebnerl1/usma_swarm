#!/bin/bash

read -p "Enter tail number: " tail
echo "Configuring 192.168.11.$tail"

sshpass -p "odroid" scp -rp -o StrictHostKeyChecking=no /home/rrc/usma_swarm/usma_files/ROS_Nodes/99-radeye.rules root@192.168.11.$tail:/etc/udev/rules.d/

sshpass -p "odroid" scp -rp -o StrictHostKeyChecking=no /home/rrc/usma_swarm/usma_files/ROS_Nodes/99-rangefinder.rules root@192.168.11.$tail:/etc/udev/rules.d/

sshpass -p "odroid" scp -rp -o StrictHostKeyChecking=no /home/rrc/usma_swarm/usma_files/ROS_Nodes/altitude_eye_ROS.py odroid@192.168.11.$tail:~/acs_ros_ws/src/autonomy-payload/ap_safety_monitor/src/

sshpass -p "odroid" scp -rp -o StrictHostKeyChecking=no /home/rrc/usma_swarm/usma_files/ROS_Nodes/rad_eye2_ROS.py odroid@192.168.11.$tail:~/acs_ros_ws/src/autonomy-payload/ap_safety_monitor/src/

sshpass -p "odroid" scp -rp -o StrictHostKeyChecking=no /home/rrc/usma_swarm/usma_files/ROS_Nodes/master.launch odroid@192.168.11.$tail:~/acs_ros_ws/src/autonomy-payload/ap_master/launch/

sshpass -p "odroid" scp -rp -o StrictHostKeyChecking=no /home/rrc/usma_swarm/usma_files/ROS_Nodes/test.txt odroid@192.168.11.$tail:~/acs_ros_ws/src/autonomy-payload/ap_master/launch/

echo "Finished!"

