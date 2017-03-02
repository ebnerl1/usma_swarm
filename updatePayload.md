## Updating the payload on UAS
1.	Note the “tail” number of the UAS: ________(A)________(e.g. 10)
2.	Power up the UAS with a DC power supply.
3.	Start up and log onto the Linux computer.  Ensure the computer is connected to the Internet via an Ethernet connection.
4.	On the Linux computer, open a terminal (Ctrl-Alt-T).
5.	Attach an Alfa Wi-Fi adapter to the computer
6.	In the terminal, type `ifconfig` and hit enter.
 - Identify the Internet-facing adapter. For Ethernet connections, look for an adapter name such as `eth0` or `eth1`; for the Wi-Fi connections, look for the first wireless adapter such as `wlan0` or `wlan1` (on the Predators, the Ethernet adapter has a much more complex name).  Note that adapter name as ________(B)________.
 - Identify the Alfa Wi-Fi adapter.  Look for a wireless adapter such as wlan2, where the last number could be anything larger than 2 and it does not have an IP address assigned to it (on the Predators, the adapter has a much more complex name).  Note that `wlan0` or `wlan1` is usually the built-in adapter for the laptop, which may or may not have an IP address.  Note that adapter name as _______(C)_________.  
7.	Set up the computer as a router between the Ethernet and the Wi-Fi.
  - `wifi_config.sh –T 11 –R (B) (C) 1`, where (B) and (C) are the names identified in lines 6a and 6b above.
8.	In the terminal, re-enter `ifconfig` and check the IP address for the Wi-Fi adapter you entered for (C). It should have an address of 192.168.11.1
9.	Copy over a valid RSA ID to the UAV. 
  - `scp $HOME/.ssh/id_rsa odroid@192.168.11.(A):~/.ssh/id_rsa`, where (A) is the “tail” number you entered in line 1. 
  - When prompted for a password, enter `odroid`.
10.	Connect to the UAS via secure shell.  
  - `ssh odroid@192.168.11.(A)`, where (A) is the “tail” number you entered in line 1.  
  - When prompted for a password, enter `odroid`.
11.	At the UAS command prompt (e.g. odroid@192.168.11.10 :~$), enter `./odroid-installer-14.04.sh`
  - When prompted for a unique numeric ID, enter the tail number from line 1. (e.g. 10)
  - When prompted for the team, enter 11 (11=Army).
  - When prompted for the name, enter plane(A), where (A) is the “tail” number you entered in line 1.  (e.g. plane10)
  - At prompt asking if the Internet gateway is installed, hit the Enter key.
  - Download and Install will begin
12.	Note the time of installation start. The installation can take up to an hour. 
