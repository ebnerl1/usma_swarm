# How to run a simulation in SCRIMMAGE

### This feature is particularly helpful to make rapid developement/test cycles for a behavior without needing to run the full SITL simulation.  By running Scrimmage, vehicles are automatically launched and start doing the behavior specified. In addition, metrics can be gathered in a log file to assess behavior performance. 

# How to run SCRIMMAGE

1. `cd scrimmage/scrimmage`
2. `scrimmage ../usma/missions/greedy_goto.xml` (or replace `greedy_goto.xml` with the mission you want to run)


# SCRIMMAGE Commands

1. `b` - Pause/Resume
2. `]` - Speed up time
3. `[` - Slow down time
4. `up/down` or `left/right` arrow keys - switch to other drone views
5. `a` - Rotate through camera views


# Files needed to run SCRIMMAGE (using greedy_goto as example)

1. `greedy_goto.xml`
  - File location:`scrimmage/usma/missions`
  - File that scrimmage actually runs in command line. 
  - Contains swarm composition, location, and simulation parameters. 
  - Swarm behavior defined in `<autonomy>` parameter, which calls the .xml file in bullet 2 

2. `Greedy_goto.xml`
  - File location:`scrimmage/usma/plugins/autonomy/config`
  - This file is called by the .xml in bullet 1
  - Only purpose is to point to the actual behavior script in bullet 3. 
  - Set `<module>` to the name of the python file you want to run (bullet 3) 
  - Set `<class>` to the name of the class in that behavior
 
3. `greedy_goto.py`
  - File location:`scrimmage/usma/plugins/autonomy/python`
  - This is the actual behavior code to be developed and tested
  - This is the same file that gets run on SITL and live vehicles

4. `Simulations.csv`
  - File location:`scrimmage/scrimmage`
  - This is an output file with metrics collected during simulation
