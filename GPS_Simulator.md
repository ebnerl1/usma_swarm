# Run the GPS Simulator in TH208 for Preflight

### Turn Off the System when not in use. It will affect cell phone GPS receivers.

1. Power on the unit
2. Log in as "Cadet".  Password is on the Computer.
3. Run `AY181_Range11.vi` on the desktop
4. In GPS Settings set the 3 files as:
  - Almanac File Path: `C:\Documents and Settings\Cadet\My Documents\Range11\dec2017_almanac.al3`
  - Ephemeris File Path: `C:\Documents and Settings\Cadet\My Documents\Range11\brdc0020.17n`
  - Trajectory File Path: `C:\Documents and Settings\Cadet\My Documents\Range11\haltRange11.txt`
5. In LLA set (for River Court):
  - Lat: 41 deg 23 min 26.581 sec N 
  - Lon: 73 deg 57 min 11.801 sec W 
  - Alt: 2.59 m
6. Hit `Run` Button (-||->), 2 buttons left of the `Stop Button`
7. Hit `Run Continuously`, 3 buttons left of the `Stop Button`
8. Vehicles may take up to 15min to get GPS if they have to download new almanac
