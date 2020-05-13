# Usage Guide
Both of the following swarm behaviors rely on a server run from the ground station. There are three types of flights that you can run: simulation, ground test, and full flight. Slight modifications in the server and the swarm behavior will allow you to change between these tests.
* Simulation: This is a full simulation, where both the drones and the data are being simulated
* Ground Test: This is a test, where the actual drones are running the behavior, but they are configured so they will remain where they are and not fly away. This can be done indoors
* Full Flight: This is where the drones are running the behavior and flying between waypoints. Please don't do this indoors.

## **Simulation Script**

### **Quick Start**
* Start Simulation
    ```
    cd ~/usma_swarm/usma_files
    ./run_sim.sh
    ```
* Restart: `<enter>`
* Quit:    `q <enter>`

### **Description**

To make development a little easier, we modified a simulation script [here](../../run_sim.sh). The script has two functions:
* **Start a simulation:**
    * Starts the simulated drones. 
        * To change the number of drones started, change the number on lines 35 and 74 to the number that you want.
        * To change the location, change the -L... argument on lines 35 and 75.
    * Starts FTI on sitl_bridge_1
    * Starts swarm_commander and sets the network device to sitl_bridge_1
* **Restart a simulation:**

    This is important because of how the swarm architecture handles our swarm files. The entire architecture is built in c++, but our scripts are written in python. When we start a drone, it looks at all python files that are required and reads the corresponding compiled .pyc files. If there is not a .pyc file for a .py file, then it generates it. The problem is that, except for the Swarm behavior file, this only happens on startup. So, if we modify a python file that is not the actual swarm behavior and recompile it after the drones have started, then they will not see the modification. This forces us to terminate the drones and restart them if we modify a non-swarm behavior file. 
    
    Because of this, I built in functionality to restart the drones and recompile your python files. If you have a file that you are working on that is a non-swarm file, then add it to the recompiledFiles array in run_sim.sh. When you modify this file and want to restart the drones, then press enter on your simulation terminal, and it will destroy the current drones, recompile the python files, and start new drones. There is no need to close fti and swarm_commander

    When you are done, type q and press enter on the terminal to stop the drones. You will still need to exit out of fti and swarm_commander


## **Radiation Detection**

### **Parameters**
The Wrath_Rad swarm behavior has the following parameters that can be modified to slightly change the functionality of the swarm.
  * Height: This is in meters, and controls the height of every drone in the swarm
  * Time At WP: How long each drone stays at a waypoint for moving on. Useful for testing
  * WP Distance Threshold: How close a drone needs to be to a waypoint before it moves on to the next waypoint. This should always be at least 1
  * Collision Distance Threshold: How far away drones need to be before they enter a collision avoidance state
  * Simulated Drone: Whether or not you're running a simulation
  * Simulated Data: Whether or not we need to simulate radiation data

### **Starting Flights**

1. **Simulation**
    1.  Change the IS_SIMULATION flag in /serverSide/wrath_waypoint_server to True
    1.  Run the server with the following commands:
        ```
        cd ~/usma_swarm/usma_files/serverSide
        python wrath_waypoint_server.py
        ```
    1.  Start the simulation. For this, we will assume you are using the simlulation script explained above:
        ```
        cd ~/usma_swarm/usma_files
        ./run_sim.sh
        ```
    1.  Follow the checklist to arm and launch the drones
    1.  Start the wrath_rad swarm behavior from swarm_commander with following required parameters
        * Simulated Drone = True
        * Simulated Data = True
1. **Ground Test**
    1. Follow steps to start drones, including starting the bridge, fti, and swarm_commander
    1. Change the IS_SIMULATION flag in /serverSide/wrath_waypoint_server to False
    1. Run the server with the following commands:
        ```
        cd ~/usma_swarm/usma_files/serverSide
        python wrath_waypoint_server.py
        ```
    1. Follow the checklist to arm and launch the drones
    1. Start the wrath_rad swarm behavior from swarm_commander with following parameters
        * Simulated Drone = False
        * WP Distance Threshold = 10000 (or some other ridiculously large number)
        * Note: This can be done with simulated data or the hermes

1. **Full Test**
    
    * Follow the exact same steps as **Ground Test** with WP Distance Threshold set to a low value, such as 5

## **Route Recon**

### **Parameters**
The Route Recon swarm behavior has the following parameters that can be modified to slightly change the functionality of the swarm.
  * WP Distance Threshold: How close a drone needs to be to a waypoint before it moves on to the next waypoint. This should always be at least 1
  * Simulated Drone: Whether or not you're running a simulation

### **Starting Flights**

1. **Simulation**
    1.  Change the IS_SIMULATION flag in /serverSide/wrath_route_recon_server.py to True
    1.  Run the server with the following commands:
        ```
        cd ~/usma_swarm/usma_files/serverSide
        python wrath_route_recon_server.py
        ```
    1.  Start the simulation. For this, we will assume you are using the simlulation script explained above:
        ```
        cd ~/usma_swarm/usma_files
        ./run_sim.sh
        ```
    1.  Follow the checklist to arm and launch the drones
    1.  Start the route recon swarm behavior from swarm_commander with following required parameters
        * Simulated Drone = True
1. **Ground Test**
    1. Follow steps to start drones, including starting the bridge, fti, and swarm_commander
    1. Change the IS_SIMULATION flag in /serverSide/wrath_route_recon_server to False
    1. Run the server with the following commands:
        ```
        cd ~/usma_swarm/usma_files/serverSide
        python wrath_route_recon_server.py
        ```
    1. Follow the checklist to arm and launch the drones
    1. Start the route recon swarm behavior from swarm_commander with following parameters
        * Simulated Drone = False
        * WP Distance Threshold = 10000 (or some other ridiculously large number)
        * Note: This can be done with simulated data or the hermes

1. **Full Test**
    
    * Follow the exact same steps as **Ground Test** with WP Distance Threshold set to a low value, such as 5

## **Darknet**
We didn't end up completely integrating the Nano into the swarm, so the network is not in its final configuration. The goal was to have the drones on one network which is 192.168.11.x and the Nanos each on their own network that can only the drone they're connected to. Because of this, every Nano would have the IP 192.168.12.2, and the drone Ethernet port would be 192.168.12.1.

Right now, they Nanos are all on the 192.168.11.x network. To start Darknet, run the script:
```
./usma_swarm/usma_files/WRATH/nanoStartup.sh
```
You will need to go in to the nanoIPs array and set the ip addresses that you want to start. This script will start the webcam, darknet, and darknetOutput ROS nodes
