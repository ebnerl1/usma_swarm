#!/bin/bash
cd
multi-sitl-start.bash -B -LSRNL -vArduCopter 2
fti.py -d sitl_bridge_1 -z &
gnome-terminal -e "bash -c 'echo network device sitl_bridge_1 | swarm_commander.py;$SHELL'"
#gnome-terminal -e "bash -c '~/usma_swarm/usma_files/serverSide/runServerSide.sh;$SHELL'"
