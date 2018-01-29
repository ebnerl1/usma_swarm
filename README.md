# usma_swarm

### Baseline requirements for single machine
1. Modern CPU (current machine: [Predator 17](https://us-store.acer.com/predator-17-gaming-laptop-g9-791-79y3))
2. Ubuntu 16.04 LTS
3. ROS Kinetic

### Install SASC (Service Academy Swarm Challenge) Software
1. Get imaged Linux/ROS computer from EECS department.
2. Login.
3. Check hosts file is consistent with host name:
    * `echo $HOSTNAME` 
    * Copy the output (ex: ros305)
    * `sudo gedit /etc/hosts.` 
    * Make sure the line 2 of the file is the same as $HOSTNAME.  If not, then correct/past in, and save.
4. Connect to the internet (EECS DS3 Network).  Preferably select a high speed wired line.  EECSNet instructions forthcoming.
5. Install Git:
    * `sudo apt-get install git`
    * NOTE: may have to restart computer if error.
6. Create SSH key:
    * `ssh-keygen -t rsa -C "your.email@example.com" -b 4096`  
    * Hit enter through all questions to keep defaults path, and no password.
7. Install xclip:
    * `sudo apt-get install xclip`
8. Copy SSH key: 
    * `xclip -sel clip < ~/.ssh/id_rsa.pub`
9. Log into: https://gitlab.nps.edu
    * Go to Profile Settings > SSH Keys tab.  Paste in ssh key (starts with ssh-rsa, end with email). Set title to computer name.
10. Clone repo:
    * `cd $HOME`
    * `git clone git@gitlab.nps.edu:sasc/sasc-help.git`
11. Install SASC by typing following commands (will have to enter password along the way): 
    * `sudo apt-get update`
    * `sudo apt-get upgrade`
    * `cd $HOME/sasc-help`
    * `./sasc_installer.py -I –s`
    * Once the installation is complete, restart the computer
12. Initialize 10 planes after installation complete    
    * `init_n_sitls.bash 10`
    * If an error occurs, check the last lines of the bashrc file (`sudo gedit .bashrc`) against a working machine. 

### Update SASC Code to 'livefly' Git Branch
1. Configure Git properly    
    * `git config --global user.email "your_email@example.edu"`    
    * `git config --global user.name "your_name"`
2. Now update code to the "livefly" branch in various repositories (this is what was flown in April 2017 demo and is what is installed on the vehicles)    
  - acs_dashboards    
    * `cd ~/ACS/acs_dashboards`   (Note: should see green “(master)”)  
    * `git checkout livefly`   (Note: now should see green “(livefly)”) 
    * `python3 setup.py build install --user`    (Note: this recompiles)
  - acs-env
    * `cd ~/ACS/acs-env`   (Note: should see green “(master)”)
    * `git checkout livefly`   (Note: now should see green “(livefly)”)    
    * `./install.sh`   (Note: this recompiles)    
  - acs_lib    
    * `cd ~/ACS/acs_lib`   (Note: should see green “(master)”)
    * `git checkout usma`   (Note: now should see green “(usma)” Not Livefly!!!)
    * `python3 setup.py build install --user`   (Note: this recompiles)
  - arbiter    
    * `cd ~/ACS/arbiter`   (Note: should see green “(master)”)
    * `git checkout livefly`   (Note: now should see green “(livefly)”)    
    * `python3 setup.py build install --user`   (Note: this recompiles)
  - swarmcommander
    * `cd ~/ACS/swarmcommander`   (Note: should see green “(master)”)    
    * `git checkout livefly`   (Note: now should see green “(livefly)”)
    * `python3 setup.py build install --user`   (Note: this recompiles)
  - ROS repos    
    * `cd ~/ACS/acs_ros_ws/src/autonomy_itar`   (Note: should see green “(master)”)    
    * `git checkout livefly`   (Note: now should see green “(livefly)”)    
    * `cd ~/ACS/acs_ros_ws/src/autonomy-payload`  (Note: see green “(master)”)    
    * `git checkout livefly`   (Note: now should see green “(livefly)”)    
    * `cd ~/ACS/acs_ros_ws/src/autopilot_bridge`   (Note: see green “(master)”)    
    * `git checkout livefly`   (Note: now should see green “(livefly)”)    
    * `cd ~/ACS/acs_ros_ws/`
    * `catkin_make`    (Note: this recompiles the ROS workspace)

### Bring in USMA Additions to Code
1. Point to the scrimmage folder
    * `cd ~/scrimmage`
2. Clone USMA files from the Gitlab repository and build
    * `git clone git@gitlab.nps.edu:todd.brick_usma.edu/scrimmage-templates.git usma`
    * `cd ~/scrimmage/usma`
    * `mkdir build && cd build`
    * `cmake .. && make`
3. Update .bashrc file to point to that folder  
    * `cd ~/`  
    * `gedit .bashrc`  
    * Scroll to bottom, and add the following line:  
      `export SCRIMMAGE_TACTIC_INTERFACE_FILE=/home/user1/scrimmage/usma/plugins/autonomy/python/behaviors.xml`
    * Save and close bashrc file  
    * `source .bashrc` (To source the updated .bashrc file)
4. Install USMA SWARM files from westpoint-robotics Github
    * `cd ~/`  
    * `git clone https://github.com/westpoint-robotics/usma_swarm.git` 
    
## Swarm Simulation using SITL Environment
1. Start basic simulation, should verify the install was successful  
    * `begin_sim.py 6`
2. See SASC documentation for more information on how to control


## Setting up the laptop to communicate with the Zephyr or DJI450

1. The Army Zephyrs are set up with the following IP schema: 
 - `192.168.11.X` where X is the “tail” number of the UAS.

2. With the Alfa USB Wi-Fi adapter connected, check for correct installation:
 - `user1@ros01:~$ lsusb`
 - Look for an entry in the list that has “RealTek” in the entry.

3. Check the network name associated with the adapter.
 - `user1@ros01:~$ ifconfig`
 - Look for wlan2 or a new entry

4. Set up the Wi-Fi adapter for the Army network.
 - `user1@ros01:~$ wifi_config.sh -T 11 wlan2 201`
 - where the wlan2 is the name of the network and 201 is the last octet of the IP address.
 - this will setup wlan2 with IP address of 192.168.11.201/24.

5. If the UAS is powered up, try ping the IP address
 - `user1@ros01:~$ ping 192.168.11.X` where X is the tail number

6. SSH into the odroid payload computer
	user1@ros01:~$ ssh odroid@192.168.11.X
  accept the key warning and use odroid as the password


Setting up the telemetry radio to communicate a Zephyr

1.  There may be a bug in the telem_config.sh in the ~/.local/bin folder
     Open up telem_config.sh in a text editor.
     Edit line 7 so that it reads like below:
     #ATC=$ACS_ROOT/SiK/Firmware/tools/atcommander.py # original line
  ATC=/home/user1/.local/bin/atcommander.py	# new line
2.  With the Sik USB telemetry radio connected, check for correct installation
	user1@ros01:~$ ls /dev | grep USB
  it should return /dev/ttyUSB0
3.  Set the net ID using the command
	user1@ros01:~$ telem_config.sh /dev/ttyUSB0 X
4.  Connect to the Pixhawk using the telemetry link
	user1@ros01:~$ mavproxy.py --master=/dev/ttyUSB0
  the mavproxy.py should connect and return info on the Arduplane autopilot


To do:
1.  Determine where the python behavior files need to be placed so that the tactic interface can find them, along with the behaviors.xml file, if needed
2.  Copy customized fence, wp.template, and rally files into ~/blessed/
3.  Verify that the plane configures properly with fti.py tool with correct fence and mission based on stack number
