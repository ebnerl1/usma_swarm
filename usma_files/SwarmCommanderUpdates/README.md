# Useful Updates for SWARM COMMANDER Module

### To Update Where Swarm Commander Initializes:
1. Open the `__init__.py` file
2. Update the SITL_LOCATION parameter (line 18)
3. Make sure the `#zoom to default location` (line ~43) has the appropriate lat-lon
4. Save and close the file
5. Copy this file into `/home/user1/ACS/swarmcommander/Swarmcommander/modules/sc_qt_gui/`
6. Then in a Terminal:
    * `cd ~/ACS/swarmcommander`
    * `python3 setup.py build install --user` (Note: this recompiles)
7. Alternatively, can open up the `__init__.py` file and paste in your lat lon in line ~43, then repeat step 6.
    * For SRNL: `33.274562, -81.577593`

### To enable vehicles on the ground to be seen by Swarm Commander:
1. Copy the `dashboardDialogWrapper.py` here into: `/home/user1/ACS/swarmcommander/modules/sc_qt_gui/`
2. Alternatively can:
    * Navigate to `/home/user1/ACS/swarmcommander/modules/sc_qt_gui/`
    * Edit `dashboardDialogWrapper.py`
    * Comment out lines 122-125 (with `self.__do_not_display_states.add`)
3. Then in a Terminal:
    * `cd ~/ACS/swarmcommander`
    * `python3 setup.py build install --user` (Note: this recompiles)
