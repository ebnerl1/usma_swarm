#!/usr/bin/env python3

#This script is for creating templates in the $HOME/blessed folder from
#the standard mission / fence / parameter / rally files in acs-env/data/missions

import os, sys, shutil

#standard variables
LAUNCH_WP=1
INGRESS_WP=3
LAST_RACETRACK_WP=10
LAND_TURNS=[13,21,29]
LAND_LOITERS=[15,23,31,37]
LAND_LINEUP=[25,33,41]
ATTACK_NAV_WPS = [37,39,41]
LAND_WPS = [18,26,34,42]
LAST_WP=45

#standard strings
#Oth member of this list is STACK 1, 1th is STACK 2, and so on.
wp_files=[sys.argv[1]]
wp_template='wp.template'

rally_file='rally_all.txt'
rally_template='rally.template'

fence_file='fence.fen'
fence_template='fence'

params_file='ZephyrII_std_annotated.parm'
params_template='param'

template_dir = os.environ['HOME'] + '/blessed/'

mission_dir = ''
params_dir = ''

#doesn't ALWAYS set STDALT for template file, there are some exceptions
def set_std_alt(wp_tokens):
    wp_num = int(wp_tokens[0])

    #Exceptions:
    if wp_num == INGRESS_WP or wp_num in LAND_LOITERS or wp_num in LAND_LINEUP:
        return

    wp_type = int(wp_tokens[3])
    nav_wp_types = [16,17,18,19,31]

    if wp_type in nav_wp_types:
        wp_tokens[10]="STDALT.000000"

#see if there is an ACS_ROOT environment variable
#if 'ACS_ROOT' in os.environ.keys():
#    mission_dir = os.environ['ACS_ROOT'] 
#else:
    #try the default location
#mission_dir = os.environ['HOME'] + '/ACS' 
mission_dir = os.environ['HOME']

#if not os.path.isdir(mission_dir):
#    print("ERROR (" + sys.argv[0] + "): can't find ACS scripts directory. You may #need to run the ACS installer.\nIf you've already run the installer you may need #to source the ~/.bashrc or the ~/.profile file.")
#    sys.exit(-1)

#mission_dir = mission_dir + "/acs-env/data"
params_dir = mission_dir + "/usma_swarm/usma_files/blessed_files/"
mission_dir = mission_dir + "/usma_swarm/usma_files/blessed_files/"


#params dir has same prefix as mission_dir ONLY AT THIS POINT IN THE SCRIPT
#params_dir = mission_dir + "/params/"
#mission_dir = mission_dir + "/missions/swarming/"

#make sure the blessed folder is present
if not os.path.isdir(template_dir):
    try: 
        os.makedirs(template_dir)
    except Exception as e:
        print("ERROR (" + sys.argv[0] + "): can't create template directory.")
        sys.exit(-2)

#fence and params are just a straight copy at this time:
shutil.copy(mission_dir + fence_file, template_dir + fence_template)
shutil.copy(params_dir + params_file, template_dir + params_template)

#process rally --------------------------
f = open(mission_dir + rally_file, 'r')
template_str = ''
i = 1
for line in f:
    line_tokens = line.split()

    #4th token needs to be changed
    line_tokens[3] = 'STDALT.000000'

    template_str += 'STACK_' + str(i)
    for tok in line_tokens:
        template_str = template_str + ' ' + tok

    template_str += "\n"
    i += 1

f.close()

f = open(template_dir + rally_template,'w')
f.write(template_str)
f.close()
#done with rally -----------------------

#process wp ------------------------------
#Unfortunately this is VERY specific to how the mission is setup.  
#Some day we need to have the autopilot support multiple missions on board
#and then this confusing mess wouldn't have to be here.

#First pass: pull all waypoints out of files and put into a list of lists
NUM_STACKS = len(wp_files)
missions = []
for i in range(0, NUM_STACKS):
    missions.append([])

    current_wp = 0
    f = open(mission_dir + wp_files[i], 'r')
    for line in f:
        line_tokens = line.split()

        try:
            read_wp = int(line_tokens[0])
        except Exception as e:
            #Assume first token is not an integer and therefore part of header
            continue

        if read_wp != current_wp:
            print("ERROR (" + sys.argv[0] + "): unexpected waypoint.")
            continue

        missions[i].append(line_tokens)
        current_wp += 1
    f.close()

#Second pass: build template
num_mission_wps = (LAST_RACETRACK_WP - INGRESS_WP) + 1
f = open(template_dir + wp_template, 'w')
f.write("QGC WPL 110\n")
for i in range(0, len(missions[0])):
    wp_num = int(missions[0][i][0])

    if wp_num == LAUNCH_WP:
        #launch waypoint might be different (lower) for quadrotor missions
        for j in range(0, len(missions)):
            f.write("STACK_" + str(j+1) + " " + " ".join(missions[j][i]) + "\n")
    elif wp_num < INGRESS_WP:
        #other early waypoints should all be the same and need no alt adjustment.
        f.write(" ".join(missions[0][i]) + "\n")
    elif (wp_num >= INGRESS_WP and wp_num <= LAST_RACETRACK_WP) or \
          wp_num in LAND_TURNS:
        #mission points based on stack and use mission-parameter altitude
        for j in range(0, len(missions)):
            set_std_alt(missions[j][i])
            f.write("STACK_" + str(j+1) + " " + " ".join(missions[j][i]) + "\n")
    elif wp_num in ATTACK_NAV_WPS:
        for j in range(0, len(missions)):
            f.write("STACK_" + str(j+1) + " " + " ".join(missions[j][i]) + "\n")
    elif wp_num in LAND_LINEUP or wp_num in LAND_LOITERS or \
         wp_num in LAND_WPS:
        #landing sequence waypoints use stack ID but not the mission-parameter
        for j in range(0, len(missions)):
            f.write("STACK_" + str(j+1) + " " + " ".join(missions[j][i]) + "\n")
    elif wp_num > LAST_RACETRACK_WP and wp_num < LAST_WP:
        #other mission waypoints are passed through but use mission-parameter alt
        set_std_alt(missions[0][i])
        f.write(" ".join(missions[0][i]) + "\n")
    elif wp_num == LAST_WP:
        #last waypoint uses stack ID and mission-parameter alt
        for j in range(0, len(missions)):
            set_std_alt(missions[j][i]) #Sam G comment: comment this out for setting alt levels
            f.write("STACK_" + str(j+1) + " " + " ".join(missions[j][i]) + "\n")

f.close()
#done with wp ----------------------------
