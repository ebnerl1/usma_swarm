# Updating Pixhawk Firmware on SASC Vehicles
The first section describes how to flash Pixhawk 2 firmware for the TAROT 650 implementation.  The 2nd section is older and pertains to the original SASC implementation.

### Build New Pixhawk firmware (can be used for Tarot 650 Quad Implementation) (TODO Test this)
Note: this is not needed if you have the Pixhawk Firmwares already built as we do in `usma_swarm/usma_files/PX4_params`

1. Detailed instructions from NPS available at `/ACS/ardupilot/build.md`

2. Make sure you have `px4-v2` firmware build

  * `cd ~/ACS/ardupilot/build` and ensure `px-v2` is there.  
  * If not get it from NPS Gitlab (TODO: update to DI2E): `git clone --recursive git@gitlab.nps.edu:sasc/ardupilot.git` 

3. Ensure `test.py` file will not cause problems

  * `cd ~/scrimmage/usma/plugins/autonomy/python`   
  * `gedit test.py`, ensure all lines are commented, save (TODO: fix this)
  
4. Build the firmware (Note: unsure what v3 does, use v2)

  * `./waf configure --board px4-v2`   
  * `./waf copter`


### Install Necessary Tools to Load Firmware onto Pixhawk
1. Install mono

  * Instructions available at `www.mono-project.com/download/stable`
  * Installation commands:
     - `sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 3FA7E0328081BFF6A14DA29AA6A19B38D3D831EF`
     - `sudo apt install apt-transport-https ca-certificates`
     - `echo "deb https://download.mono-project.com/repo/ubuntu stable-xenial main" | sudo tee /etc/apt/sources.list.d/mono-official-stable.list`
     - `sudo apt update`
     - `sudo apt install mono-devel`

2. Download MissionPlanner Windows Executable

  * Available at `http://firmware.ardupilot.org/Tools/MissionPlanner/`


### Flash Firmware onto Pixhawk
1. Connect computer to internet (needed for basic firmware install).
2. Run MissionPlanner.  If on linux, commands:

  * `cd MissionPlanner-1.3.53`
  * `sudo mono MissionPlanner.exe`

3. Install Basic Firmware First:
  * In MP, select Initial Setup > Install Firmware > ArduCopter Quad
     - APM 2+? > NO
     - PX4/PIXHAWK/PIXRACER? > YES
     - PIXRACER? > NO
     - CUBE? > YES (if working with Pixhawk 2 Cube)
  * Follow MP Prompts - should result in Pixhawks making "happy sounds"
  
4. Install SASC Firmware:
  * In MP, select Initial Setup > Install Firmware 
  * Load custom firmware
  * Select: `usma_swarm\usma_files\PX4_Params\Pixhawk_Firmwares\arducopter_px4_v2_livefly.px4`
  * Follow MP Prompts - should result in Pixhawks making "happy sounds"
  
### Configure Pixhawk for Quad Operations after new Flash
1. Connect fully built vehicle to QGC (may not connect if Pixhawk alone)

2. Calibrate the following:

  * Accelerometers: Vehicle Setup > Sensors > Accelerometers - Follow prompt  
  * Compass: Vehicle Setup > Sensors > Compass - Follow prompt
  * Radio: Vehicle Setup > Radio > Calibrate - Follow prompt

3. Update the Flight Modes at Vehicle Setup > Flight Modes to:

  * Flight Mode 1: Stabilize
  * Flight Mode 4: Loiter  
  * Flight Mode 5: Auto

4. Update the Power Monitor (to read voltage) at Vehicle Setup > Power to:

  * Battery monitor: Analog Voltage Only
  * Battery capacity: 6000 mAh 
  * Minimum arming voltage: 0
  * Power sensor: Power Module 90A

5. In Parameters set "serial2_baud" to "1,500,000"

6. Make sure ODROID master.launch parameters are up to date for USB FTDI
  * See instructions in: https://github.com/westpoint-robotics/usma_swarm/blob/master/odroidReimageAndConfigure.md

### Updating the Pixhawk on the SASC Zephyr II UASs

1.	Note the “tail” number of the UAS: ____(A)____ (e.g. 10)
2.	Open up the “hatch” of the Zephyr and gently lift the Pixhawk out of the 3D printed carriage so that the micro B USB port is accessible.
3.	Start up and log onto the Linux computer.  Ensure the computer is connected to the Internet via an Ethernet connection.  Ensure you have a USB A to USB micro B cable that can reach between the laptop and the Pixhawk.
4.	On the Linux computer, open a terminal (`Ctrl-Alt-T`).
5.	In the terminal, enter `setupPixhawkPlane.sh` (only for fixed wing Zephyrs!)
6.	Build the firmware image: At the application menu, enter `B` (can skip if it has already been done for this release of ArduPlane).
7.	Update the Pixhawk:  At the application menu, enter `U`.  If the application asks for an aircraft ID, enter the “tail” number identified in line 1 (e.g. 10).
8.	Follow all the instructions for connecting and disconnecting the Pixhawk when prompted.
9.	If an ‘Update Successful!” is displayed, enter Q to quit the application.
10. From the terminal window, make note of the “hash” code whcih appears after the version string (e.g. `ArduPlane V3.8.0beta1 (cf848147)`.
11. Gently seat the Pixhawk back in the carriage.  Make sure no cables are pinched.
