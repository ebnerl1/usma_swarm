## Updating the Pixhawk on the SASC Zephyr II UASs

1.	Note the “tail” number of the UAS: ____(A)____ (e.g. 10)
2.	Power up the UAS with a DC power supply.
3.	Open up the “hatch” of the Zephyr and gently lift the Pixhawk out of the 3D printed carriage so that the micro B USB port is accessible.
4.	Start up and log onto the Linux computer.  Ensure the computer is connected to the Internet via an Ethernet connection.  Ensure you have a USB A to USB micro B cable that can reach between the laptop and the Pixhawk.
5.	On the Linux computer, open a terminal (`Ctrl-Alt-T`).
6.	In the terminal, enter `setupPixhawkPlane.sh` (only for fixed wing Zephyrs!)
7.	Build the firmware image.  At the application menu, enter `B` (can skip if it has already been done for this release of ArduPlane).
8.	Update the Pixhawk.  At the application menu, enter `U`.  If the application asks for an aircraft ID, enter the “tail” number identified in line 1 (e.g. 10).
9.	Follow all the instructions for connecting and disconnecting the Pixhawk when prompted.
10.	If an ‘Update Successful!” is displayed, enter Q to quit the application.  In the terminal window, look for the “hash” code after the version string (e.g. `ArduPlane V3.8.0beta1 (cf848147)`.  Make a note of the hash and record it.
Power down the plane and disconnect the DC power supply
Gently seat the Pixhawk back in the carriage.  Make sure no cables are pinched.
