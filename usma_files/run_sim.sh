#!/bin/bash

# Script designed to launch all the necessary windows and programs to run a simulation
# To change num quads, change the number after vArduCopter
# To change location, swap out the location tag after -B (i.e. "-B -LSRNL")
# When you're done with the sim, press enter in the terminal and it will shutdown the drones

cd

echo "Creating drones"

multi-sitl-start.bash -B -LRiverCourt -vArduCopter 2

clear

echo "Starting fti"

gnome-terminal -x bash -c "fti.py -d sitl_bridge_1 -z; exit; exec bash"

echo "Starting Swarm Commander"

echo "Next: "
echo "    1) Run MavProxy"
echo "      - wp list"
echo "      - fence list"
echo "      - rally list"
echo "    2) Toggle flight ready"
echo "    3) Armed"
echo "    4) Auto"

gnome-terminal -x bash -c "swarm_commander.py; exit; exec bash"

# echo network device sitl_bridge_1 | 

#gnome-terminal -e "bash -c '~/usma_swarm/usma_files/serverSide/runServerSide.sh;$SHELL'"
# Line above can be uncommented to launch the wpServer at the same time

# Wait for user to press enter before destroying drones
read varname

echo "Destroying Drones"

multi-sitl-cleanup.bash

rm -rf -d ~/scrimmage/usma/plugins/autonomy/python/*.pyc
rm -rf -d ~/scrimmage/usma/plugins/autonomy/python/Collections/*.pyc

clear

