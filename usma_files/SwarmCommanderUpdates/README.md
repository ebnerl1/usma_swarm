# Useful Updates for SWARM COMMANDER Module

### To Update Where Swarm Commander Initializes:
1. Open the `__init__.py` file
2. Update the SITL_LOCATION parameter (line 18)
3. Make sure the `#zoom to default location` (line ~43) has the appropriate lat-lon
4. Save and close the file
5. Copy this file into `/home/user1/ACS/swarmcommander/modules/sc_qt_gui/`
6. Then in a Terminal:
    * `cd ~/ACS/swarmcommander`
    * `python3 setup.py build install --user` (Note: this recompiles)


