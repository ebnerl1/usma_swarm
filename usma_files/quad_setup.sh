#!/bin/bash

read -p "Enter tail number: " tail
echo "Configuring 192.168.11.$tail"

# sending radeye
sshpass -p "odroid" scp -rp -o StrictHostKeyChecking=no $HOME/usma_swarm/usma_files/ROS_Nodes/99-radeye.rules root@192.168.11.$tail:/etc/udev/rules.d/
sshpass -p "odroid" scp -rp -o StrictHostKeyChecking=no $HOME/usma_swarm/usma_files/ROS_Nodes/rad_eye2_ROS.py odroid@192.168.11.$tail:~/acs_ros_ws/src/autonomy-payload/ap_safety_monitor/src/

echo "Configured radeye"

# sending range finder
sshpass -p "odroid" scp -rp -o StrictHostKeyChecking=no $HOME/usma_swarm/usma_files/ROS_Nodes/99-rangefinder.rules root@192.168.11.$tail:/etc/udev/rules.d/
sshpass -p "odroid" scp -rp -o StrictHostKeyChecking=no $HOME/usma_swarm/usma_files/ROS_Nodes/altitude_eye_ROS.py odroid@192.168.11.$tail:~/acs_ros_ws/src/autonomy-payload/ap_safety_monitor/src/

echo "Configured range finder"

# sending hermes
sshpass -p "odroid" scp -rp -o StrictHostKeyChecking=no $HOME/usma_swarm/usma_files/ROS_Nodes/hermes/99-hermes.rules root@192.168.11.$tail:/etc/udev/rules.d/
sshpass -p "odroid" scp -rp -o StrictHostKeyChecking=no $HOME/usma_swarm/usma_files/ROS_Nodes/hermes/scripts/acquireStdMsg.py odroid@192.168.11.$tail:~/acs_ros_ws/src/autonomy-payload/ap_safety_monitor/src/

echo "Configured hermes"

# sending master.launch
sshpass -p "odroid" scp -rp -o StrictHostKeyChecking=no $HOME/usma_swarm/usma_files/ROS_Nodes/master.launch odroid@192.168.11.$tail:~/acs_ros_ws/src/autonomy-payload/ap_master/launch/

echo "Configured master.launch"

echo "Finished!"

