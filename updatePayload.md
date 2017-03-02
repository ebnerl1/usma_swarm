## Updating the payload on UAS
1.	Enter the “tail” number of the UAS: _______ (e.g. 10) (A)
2.	Power up the UAS with a DC power supply.
3.	Start up and log onto the Linux computer.  Ensure the computer is connected to the Internet via an Ethernet connection.
4.	On the Linux computer, open a terminal (Ctrl-Alt-T).
5.	Attach an Alfa Wi-Fi adapter to the computer
6.	In the terminal, enter ifconfig
  a.	Look for an adapter such as eth0 (or eth1) for the Ethernet connection (on the Predators, the Ethernet adapter has a much more complex name).  Enter that adapter name: ________________ (B).
  b.	Look for an adapter similar to wlan2, where the last number could be anything larger than 2 (on the Predators, the adapter has a much more complex name).  Note that wlan0 or wlan1 is usually the built-in adapter for the laptop.  Enter that adapter name: ________________(C).  
7.	Set up the computer as a router between the Ethernet and the Wi-Fi.  In the terminal, enter wifi_config.sh –T 11 –R (B) (C) 1, where (B) and (C) are the names entered in lines 6a and 6b above.
8.	In the terminal, re-enter ifconfig.  Look at the IP address for the Wi-Fi adapter you entered for (C).  It should have an address of 192.168.11.1.
9.	Copy over a valid RSA ID to the UAV.  In the terminal, enter scp $HOME/.ssh/id_rsa odroid@192.168.11.(A):~/.ssh/id_rsa, where (A) is the “tail” number you entered in line 1.  When prompted for a password, enter odroid.
10.	Connect to the UAS.  In the terminal, enter ssh odroid@192.168.11.(A), where (A) is the “tail” number you entered in line 1.  When prompted for a password, enter odroid.
11.	At the UAS command prompt (e.g. odroid@192.168.11.10 :~$), enter ./odroid-installer-14.04.sh
  a.	When prompted for a unique numeric ID, enter the tail number from line 1. (e.g. 10)
  b.	When prompted for the team, enter 11 (11=Army).
  c.	When prompted for the name, enter plane(A), where (A) is the “tail” number you entered in line 1.  (e.g. plane10)
  d.	At prompt asking if the Internet gateway is installed, hit the Enter key.
12.	Enter the time of installation start: _____________
13.	(Enter the time of installation end: _____________ (The installation can take up to an hour)
14.	If the script reports that the installation was successful, enter your initials and date verifying the payload has been updated successfully:
  a.	Technician: __________________
  b.	Date: _______________________ 
