#!/bin/bash

# Script designed to launch all the necessary windows and programs to run a simulation
# To change num quads, change the number after vArduCopter
# To change location, swap out the location tag after -B (i.e. "-B -LSRNL")
cd
multi-sitl-start.bash -B -LSRNL -vArduCopter 2
fti.py -d sitl_bridge_1 -z &
gnome-terminal -e "bash -c 'echo network device sitl_bridge_1 | swarm_commander.py;$SHELL'"
#gnome-terminal -e "bash -c '~/usma_swarm/usma_files/serverSide/runServerSide.sh;$SHELL'"
# Line above can be uncommented to launch the wpServer at the same time