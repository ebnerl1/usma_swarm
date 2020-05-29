# How to run a Software-In-The-Loop (SITL) simulation

### This simulation tool recreates a realistic flight environment for the SASC system and uses the same interfaces as live operations.  It is extremely powerful to troubleshoot the system.  If the system works in SITL, it will almost certainly work in real flight.

# Manually Starting up SITL

1. `multi-sitl-start.bash -B -LRiverCourt -vArduCopter 3`
  - starts `3` sim quads: each have payload computer module and ArduCopter (Pixhawk) module
  - `–LRiverCourt` or `–LRange11` changes location to that specified in `locations.txt`(see `updateMissionParameters.md`)
  - `-vArduCopter` changes to quadrotors (excluding it defaults to planes)
2. `fti.py –d sitl_bridge_1 –z`
3. `swarm_commander.py`
  - In swarm_commander terminal (dark grey), type `network device sitl_bridge_1`
4. Launch and operate vehicles according to SASC CHECKLIST
5. `multi-sitl-cleanup.bash`
  - Terminates all the sim (SITL) vehicles

# Quick Start to SITL

### See `usma_files/WRATH/Documentation/UsageGuide.md` for more details on this.

1. `cd ~/usma_swarm/usma_files`
2. `./run_sim.sh`
3. Restart: `<enter>`
4. Quit: `q <enter>`
