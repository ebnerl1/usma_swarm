#!/usr/bin/env python

import getopt, sys, time
from subprocess import Popen

import ap_lib.acs_messages as messages
import ap_lib.ap_enumerations as enums
from ap_lib.acs_socket import Socket

SITL_LOCATION = 3   # 0 = McMillan, 1 = USMA/Range11, 2 = USMA/RiverCts, 3 = INL

def usage():
    print "Usage: begin_sim.py [options] num_UAVs"
    print "Options:"
    print "-d --disable-battlebox       Disable battlebox"
    print "-h --help                    Print help screen:"
    print "-v --vehicle                 Vehicle type (default ArduPlane)"
    sys.exit(2)

vehicle="ArduPlane"

try:
    opts, args = getopt.getopt(sys.argv[1:], "v:h:d", ["vehicle", "help", "disable-battlebox"])
except getopt.GetoptError as err:
    # print help information and exit:
    print str(err)  # will print something like "option -a not recognized"
    usage()

disable_battlebox = False
for o, a in opts:
    if o in ("-v", "--vehicle"):
        if a not in ('ArduPlane','ArduCopter'):
            print("Invalid vehicle. Valid values are ArduPlane or ArduCopter.")
            sys.exit(3)

        vehicle = a
    elif o in ("-d", "--disable-battlebox"):
        disable_battlebox = True
    elif o in ("-h", "--help"):
        usage()
        sys.exit()
    else:
        assert False, "unhandled option"

if len(args) < 1:
    usage()

num_uavs = int(args[0])

num_blues = num_uavs
num_reds = 0
blue_start_id = 1
red_start_id = blue_start_id + num_blues

if num_uavs > 10:
    response = raw_input(str(num_uavs) + " is a lot of simulations! this could bog down your computer (or freeze it completely).  are you sure? (y/N)")
    if response != 'y' and response != 'Y':
        sys.exit(0)

print "Starting sim with", num_blues, "blues and", num_reds, "reds"

team_sock = []

#blue (sitl_bridge_1) is team_id 0, red is 1
def sendMsgToTeam(msg, team_id):
    try:
        #sometimes the message doesn't get through on VMs: send 5 times (messages are supposed to be idempotent anyway)
        team_sock[team_id].send(msg)
        time.sleep(0.1)
        team_sock[team_id].send(msg)
        time.sleep(0.1)
        team_sock[team_id].send(msg)
        time.sleep(0.1)
        team_sock[team_id].send(msg)
        time.sleep(0.1)
        team_sock[team_id].send(msg)
    except Exception as ex:
        pass

def sendMsgMultipleTimes(msg, team_id, repeat=5):
    for j in xrange(1, repeat):
        sendMsgToTeam(msg, team_id)
        time.sleep(1)

Popen(["multi-sitl-cleanup.bash"]).wait()

print("Starting %d blues" % (num_blues))
if SITL_LOCATION == 1:
    Popen(["multi-sitl-start.bash", "-LRange11", "-v", vehicle, "-I", str(blue_start_id), 
        "-B", str(num_blues)]).wait()
elif SITL_LOCATION == 2:
    Popen(["multi-sitl-start.bash", "-LRiverCourt", "-v", vehicle, "-I", str(blue_start_id), 
        "-B", str(num_blues)]).wait()
elif SITL_LOCATION == 3:
    Popen(["multi-sitl-start.bash", "-LINL", "-v", vehicle, "-I", str(blue_start_id), 
        "-B", str(num_blues)]).wait()
else:
    Popen(["multi-sitl-start.bash", "-v", vehicle, "-I", str(blue_start_id), 
        "-B", str(num_blues)]).wait()

#print("Starting %d reds" % (num_reds))
#Popen(["multi-sitl-start.bash", "-v", vehicle, "-B", "-P", "-T", "2", "-I",
	 #str(red_start_id), str(num_reds)]).wait()

print("Preparing to launch simulated aircraft...")

#wait n seconds for sitls to start
#TODO: actively detect when aircraft are active rather than wait
time.sleep(10)

#Setup Sockets
try:
    team_sock.append(Socket(252, 5554, 'sitl_bridge_1', None, None, bcast_bind=True))
    #team_sock.append(Socket(251, 5554, 'sitl_bridge_2', None, None, bcast_bind=True))
except Exception as ex:
    pass    

#print("Socket IP:", team_sock[0]._ip)
#print("Socket bcast_IP:", team1_sock[0]._bcast)

#Arm message
ad = messages.Arm()
ad.msg_secs = 0
ad.msg_nsecs = 0
ad.msg_fl_rel = True
ad.enable = True

#Cal Pressure message:
cl = messages.Calibrate()
cl.msg_secs = 0
cl.msg_nsecs = 0
cl.msg_fl_rel = True
cl.index = 1

#Flight Ready message:
fr = messages.FlightReady()
fr.msg_secs = 0
fr.msg_nsecs = 0
fr.msg_fl_rel = True
fr.ready = True

#AUTO mode message
au = messages.Mode()
au.msg_secs = 0
au.msg_nsecs = 0
au.msg_fl_rel = True
au.mode = 4           # AUTO mode number

#Mission Config message
mc = messages.MissionConfig()
mc.msg_secs = 0
mc.msg_nsecs = 0
mc.msg_fl_rel = True
mc.std_alt = 100
mc.stack_num = 1

if vehicle == "ArduCopter":
    #wait a bit longer because ArduCopter takes for-freaking-ever to init.
    time.sleep(5)

#Simulated planes now start with ID at 1 (not 101)
current_team = 0
copter_offset = 50
if vehicle == "ArduCopter":
    blue_start_id = blue_start_id + copter_offset
    red_start_id = red_start_id + copter_offset

for i in xrange (blue_start_id,blue_start_id+num_uavs):
    if i < red_start_id:
        mc.stack_num = 1
    else:
        #put red team in stack 2
        print ("Putting %d in stack 2" % i)
        mc.stack_num = 2
        current_team = 1        

    print ("telling", i, "to prep")
    fr.msg_dst = i
    sendMsgToTeam(fr, current_team)
    mc.msg_dst = i    
    sendMsgToTeam(mc, current_team)    
    if vehicle == "ArduPlane":
        cl.msg_dst = i
        sendMsgToTeam(cl, current_team)
        #calibration takes a sec
        time.sleep(1)

#update to mission from the Mission Config message needs a few seconds
# to arrive and update the autopilot
#DON'T do this wait in a loop -- slowdown is cummulative:
time.sleep(4)

current_team = 0
for i in xrange (blue_start_id,blue_start_id+num_uavs):
    ad.msg_dst = i

    if i >= red_start_id:
        current_team = 1
    
    ad.msg_dst = i

    #don't arm copter yet -- if it hits the timeout and disarms it will
    #kick us back into STABILIZE mode (need GUIDED mode to takeoff)
    if vehicle == "ArduPlane":
        print ("Arming %d" %i)
        sendMsgToTeam(ad, current_team)
        #Switch to AUTO mode (for either Plane or Copter)
        au.msg_dst = i
        sendMsgToTeam(au, current_team)

    #Note a lot more sleeps for ArduCopter -- it's finiky
    if vehicle == "ArduCopter":
        #don't launch all at once: sleep between launches
        #ALSO: update to mission from the Mission Config message needs a few seconds
        # to arrive and update the autopilot
        time.sleep(1)
        print ("Launching", i)
        au.msg_dst = i
        au.mode = 3 #Copter launches in GUIDED mode
        #try a few times for finiky ArduCopter
        sendMsgMultipleTimes(au, current_team)

        #make sure to arm RIGHT BEFORE takeoff or arming might time out:
        print ("Arming %d" %i)
        sendMsgToTeam(ad, current_team)
        time.sleep(2) #wait for arming to take

        #Send takeoff command
        tk = messages.Takeoff()
        tk.msg_dst = i
        tk.msg_secs = 0
        tk.msg_nsecs = 0
        tk.takeoff_alt = 80
        print ("Sending takeoff msg to %d (team %d)" % (i, current_team))
        sendMsgToTeam(tk, current_team)

        #send to auto after a few second wait for throttle up
        time.sleep(3)
        print ("Sending %d to AUTO" % i)
        au.mode = 4 #AUTO
        #try a few times for finiky ArduCopter
        sendMsgMultipleTimes(au, current_team)

#swarm_commander_start_command = ["swarm_commander.py"]
#Popen(swarm_commander_start_command)
#arbiter_start_command = ["arbiter_start.py"]
#if disable_battlebox:
    #arbiter_start_command.extend(["--score", "bb", "0"])
#Popen(arbiter_start_command)

