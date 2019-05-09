### Initializing a Swarm Behavior
1. Ensure that the drones included in the mission are in swarm state "staged"
    * push selected drones to swarm state "swarm ready"
    * assign selected UAV to subswarm 1
    * Go to "Specify Selected Behavior Parameters" and choose a behavior to begin
    * Make sure waypoint server is running (runServerSide.sh)
2. Terminate behavior
3. Kill waypoint server
    * run killServerSide.sh
4. Make sure to re-run the waypoint server every time you initialize a behavior