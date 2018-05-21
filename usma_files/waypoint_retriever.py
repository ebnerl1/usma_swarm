import spud_unified as usma_enums
import tkMessageBox
from tkinter import *
gui = Tk()
gui.title("Waypoint Retriever")

spud = Label(gui, text="Spud Number: ")
spud.grid(row=0)
spudentry = Entry(gui, bd = 5)
spudentry.grid(row=0, column=1)
#spudnum = spudentry.get()

waypoint = Label(gui, text="Waypoint Number: ")
waypoint.grid(row=1)
waypointentry = Entry(gui, bd = 5)
waypointentry.grid(row=1, column=1)
#waypointnum = waypointentry.get()

def getWP():
  spudnum = spudentry.get()

  if (int(spudnum) == 1):
      enumList = usma_enums.WP_LOC_S1
  elif (int(spudnum) == 0):
      enumList = usma_enums.WP_LOC_SX
  elif (int(spudnum) == 4):
      enumList = usma_enums.WP_LOC_S4
  elif (int(spudnum) == 5):
      enumList = usma_enums.WP_LOC_S5
  else: tkMessageBox.showerror("Error", "Please Choose 0,1,4,5")
  waypointnum = waypointentry.get()
  window = Toplevel(gui)
  window.title("LatLon")
  lat = Label(window, text = "Latitude: ")
  lat.grid(row=0)
  latlab = Label(window, text=enumList[int(waypointnum)][0]).grid(row=0,column=1)
  

  lon = Label(window, text = "Longitude: ")
  lon.grid(row=1)
  lonlab = Label(window, text=enumList[int(waypointnum)][1]).grid(row=1,column=1)
  print (enumList[int(waypointnum)])
  

getbutton = Button(gui, text="Get Waypoint", command=getWP)
getbutton.grid(row=2)



gui.mainloop()
