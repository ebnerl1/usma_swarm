## Updating the Pixhawk on the SASC Zephyr II UASs

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
