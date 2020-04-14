#!/bin/bash

# Script designed to launch all the necessary windows and programs to run a simulation
# To change num quads, change the number after vArduCopter
# To change location, swap out the location tag after -B (i.e. "-B -LSRNL")

# If you changed non-behavior files, and need to recompile and restart the drones, press enter
# If you want to cancel the simulation, type 'q' and press enter

# To add files that need to be recompiled, add them to the recompiledFiles Array

scrimmagePath="/home/user1/scrimmage/usma/plugins/autonomy/python/"

recompiledFiles=(
    "Networking/Client.py"
    "Networking/MessageHandler.py"
    "Networking/SwarmCommunications.py"
    "usma_msg/RouteReconMessages.py"
    "usma_msg/RadiationMessages.py"
    "Collections/RadLaneGenerator.py"
    "Collections/ContourLine.py"
    "Model/RadiationModel.py"
)

pythonScript="
import py_compile as p
import sys
p.compile(sys.argv[1])
"

cd

echo "Creating drones"

multi-sitl-start.bash -B -LCentralPark -vArduCopter 2

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

gnome-terminal -x bash -c "echo network device sitl_bridge_1 | swarm_commander.py; exit; exec bash"

# gnome-terminal -e "bash -c '~/usma_swarm/usma_files/serverSide/runServerSide.sh;$SHELL'"
# Line above can be uncommented to launch the wpServer at the same time

# Wait for user to press enter before recompiling drones

echo "Press enter to continue, press q and enter to quit"
read -p "$ " input

while [[ "$input" != "q" ]]
do

    for fileName in "${recompiledFiles[@]}"
    do
        python -c "$pythonScript" "$scrimmagePath$fileName"
    done

    multi-sitl-cleanup.bash

    multi-sitl-start.bash -B -LCentralPark -vArduCopter 2

    clear

    echo "Press enter to continue, press q and enter to quit"
    read -p "$ " input
done

echo "Destroying Drones"

multi-sitl-cleanup.bash


rm -rf -d ~/scrimmage/usma/plugins/autonomy/python/*.pyc
rm -rf -d ~/scrimmage/usma/plugins/autonomy/python/Collections/*.pyc
rm -rf -d ~/scrimmage/usma/plugins/autonomy/python/usma_msg/*.pyc
rm -rf -d ~/scrimmage/usma/plugins/autonomy/python/Model/*.pyc
rm -rf -d ~/scrimmage/usma/plugins/autonomy/python/Networking/*.pyc

clear

