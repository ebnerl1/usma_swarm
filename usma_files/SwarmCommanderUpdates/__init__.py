#!/usr/bin/env python3
"""
    Swarm Commander Qt 5 GUI Module
    Cotainer module that holds all things GUI (for Qt interface to SC).
    Michael Day
    Oct 2014
"""
from SwarmCommander.modules.lib import sc_module

from PyQt5.QtWidgets import QApplication, QDialog
from PyQt5.QtCore import QTimer
from acs_dashboards.lib.acs_general_gui.acs_map.mapWidgetWrapper import MapWidget
from SwarmCommander.modules.sc_qt_gui.dashboardDialogWrapper import DashboardDialog

import ap_lib.ap_enumerations as enums
import sys

SITL_LOCATION = 1   # 0 = McMillan, 1 = USMA/Range11, 2 = USMA/RiverCts, 3 = INL

class SC_QtGUIModule(sc_module.SCModule):
    def __init__(self, sc_state):
        super(SC_QtGUIModule, self).__init__(sc_state, "qt_gui", "qt_gui module")
        self.__app = QApplication([])
        
        self.mapWidget = MapWidget()
        self.mapWidget.show()

        self.__dashboardDialog = DashboardDialog(self.sc_state, self.mapWidget)
        self.__dashboardDialog.show()

        #periodic updates...
        self.__updater = QTimer()
        self.__updater.setInterval(500)
        self.__updater.timeout.connect(self.time_to_update)
        self.__updater.start();

        #more frequent updates...
        self.__updaterFrequent = QTimer()
        self.__updaterFrequent.setInterval(40)
        self.__updaterFrequent.timeout.connect(self.time_to_update_frequent)
        self.__updaterFrequent.start();

        #zoom to default location:
        if SITL_LOCATION == 3: 
            self.mapWidget.zoomTo(16, 43.872280, -112.726646)
        elif SITL_LOCATION == 2: 
            self.mapWidget.zoomTo(16, 41.390717, -73.953278)
        elif SITL_LOCATION == 1: 
            self.mapWidget.zoomTo(16, 41.360461, -74.032838)
        else:
            self.mapWidget.zoomTo(16, 35.716888, -120.7646408)

        #add game-related icons and boxes
        (bbox, cline, _, bluestage, _, redstage) = \
            enums.get_battleboxes()
        self.mapWidget.addGoalIcons(enums.GOAL_POSITS["blue"], \
                                    enums.GOAL_POSITS["red"]) 
        self.mapWidget.drawBattleBox(bluestage.get_corners(), color=(125, 125, 255))
        self.mapWidget.drawBattleBox(redstage.get_corners(), color=(255, 125, 125))
        self.mapWidget.drawBattleBox(bbox.get_corners(), cline)

        #slots
        self.mapWidget.getView().just_selected_uav.connect(self.on_uav_select)

    def start_app(self):
        sys.exit(self.__app.exec_())

    def time_to_update_frequent(self):
        #update dashboard and map:
        self.__dashboardDialog.update_uav_states()

        #update icons on map
        for id, uav_state in self.sc_state.swarm_state.uav_states.items():
            self.mapWidget.updateIcon(id, uav_state)
        for id, red_state in self.sc_state.swarm_state.red_states.items():
            if red_state.get_game_state() in ( enums.GAME_ACTIVE_DEFENSE, \
                                               enums.GAME_ACTIVE_OFFENSE ):
                self.mapWidget.updateIcon(id, red_state, True)
            else:
                self.mapWidget.hideUAVIcon(id)

    def time_to_update(self):
        #check for new textures for the map:
        self.mapWidget.checkForNewTextures()

    def unload(self):
        #THIS NEEDS TO WORK (IT DOES CURRENTLY)
        #The closeEvent handler of the dialog is what saves config:
        self.__dashboardDialog.close()
        
        #Doesn't work -- dunno why:
        #self.__mapWidget.close()

        #do any cleanup here
        self.mapWidget.done(0)
        self.__dashboardDialog.done(0)
        
        QApplication.quit()

    def on_uav_select(self, id):
        self.__dashboardDialog.selectUAV(id)

def init(sc_state):
    '''facilitates dynamic initialization of the module'''
    return SC_QtGUIModule(sc_state)
