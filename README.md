# usma_swarm
Repo for the Service Academy Swarm Challenge

## Setting up the laptop to communicate with the Zephyr or DJI450

1. The Army Zephyrs are set up with the following IP schema: 
`192.168.11.X` where X is the “tail” number of the UAS.

2. With the Alfa USB Wi-Fi adapter connected, check for correct installation:
`user1@ros01:~$ lsusb`
- Look for an entry in the list that has “RealTek” in the entry.

3. Check the network name associated with the adapter.
`user1@ros01:~$ ifconfig`
-Look for wlan2 or a new entry

4. Set up the Wi-Fi adapter for the Army network.
`user1@ros01:~$ wifi_config.sh -T 11 wlan2 201`
- where the wlan2 is the name of the network and 201 is the last octet of the IP address.
- this will setup wlan2 with IP address of 192.168.11.201/24.

5. If the UAS is powered up, try ping the IP address
	user1@ros01:~$ ping 192.168.11.X
  where X is the tail number
6.  SSH into the odroid payload computer
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
