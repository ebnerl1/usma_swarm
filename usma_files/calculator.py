import tkMessageBox
from tkinter import *
root = Tk()
root.title("Calculator")


#spud = Label(gui, text="Spud Number: ")
#spud.grid(row=0)
#spudentry = Entry(gui, bd = 5)
#spudentry.grid(row=0, column=1)
#spudnum = spudentry.get()

#waypoint = Label(gui, text="Waypoint Number: ")
#waypoint.grid(row=1)
#waypointentry = Entry(gui, bd = 5)
#waypointentry.grid(row=1, column=1)
#waypointnum = waypointentry.get()

evalstring = ""
def getOne():
  global evalstring
  evalstring+="1"
  print(evalstring)
def getTwo():
  global evalstring
  evalstring+="2"
  print(evalstring)
def getThree():
  global evalstring
  evalstring+="3"
  print(evalstring)
def getFour():
  global evalstring
  evalstring+="4"
  print(evalstring)
def getFive():
  global evalstring
  evalstring+="5"
  print(evalstring)
def getSix():
  global evalstring
  evalstring+="6"
  print(evalstring)
def getSeven():
  global evalstring
  evalstring+="7"
  print(evalstring)
def getEight():
  global evalstring
  evalstring+="8"
  print(evalstring)
def getNine():
  global evalstring
  evalstring+="9"
  print(evalstring)
def getZero():
  global evalstring
  evalstring+="0"
  print(evalstring)

def getAdd():
  global evalstring
  evalstring+="+"
  print(evalstring)

def getSubtract():
  global evalstring
  evalstring+="-"
  print(evalstring)

def getMultiply():
  global evalstring
  evalstring+="*"
  print(evalstring)

def getDivide():
  global evalstring
  evalstring+="/"
  print(evalstring)

def getPower():
  global evalstring
  evalstring+="**"
  print(evalstring)

def getEquals():
  global evalstring
  equal = eval(evalstring)
  print(equal)
  evalstring=""


  
  

#getbutton = Button(gui, text="Get Waypoint", command=getWP)
#getbutton.grid(row=2

onebutton = Button(root, text="1", command = getOne, width=2)
onebutton.grid(row=0,column=0)
twobutton = Button(root, text="2", command = getTwo, width=2)
twobutton.grid(row=0,column=1)
threebutton = Button(root, text="3", command = getThree, width=2)
threebutton.grid(row=0,column=2)
fourbutton = Button(root, text="4", command = getFour, width=2)
fourbutton.grid(row=1,column=0)
fivebutton = Button(root, text="5", command = getFive, width=2)
fivebutton.grid(row=1,column=1)
sixbutton = Button(root, text="6", command = getSix, width=2)
sixbutton.grid(row=1,column=2)
sevenbutton = Button(root, text="7", command = getSeven, width=2)
sevenbutton.grid(row=2,column=0)
eightbutton = Button(root, text="8", command = getEight, width=2)
eightbutton.grid(row=2,column=1)
ninebutton = Button(root, text="9", command = getNine, width=2)
ninebutton.grid(row=2,column=2)
zerobutton = Button(root, text="0", command = getZero, width=2)
zerobutton.grid(row=3,column=0)

addbutton = Button(root, text="+", command = getAdd, width=2)
addbutton.grid(row=0,column=3)
subtractbutton = Button(root, text="-", command = getSubtract, width=2)
subtractbutton.grid(row=1,column=3)
multiplybutton = Button(root, text="*", command = getMultiply, width=2)
multiplybutton.grid(row=2,column=3)
divisionbutton = Button(root, text="/", command = getDivide, width=2)
divisionbutton.grid(row=3,column=1)
powerbutton = Button(root, text="^", command = getPower, width=2)
powerbutton.grid(row=3, column=2)


equalsbutton = Button(root, text="=", command= getEquals, width=2)
equalsbutton.grid(row=3,column=3)



root.mainloop()
