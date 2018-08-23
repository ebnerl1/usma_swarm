#!/bin/bash
#########################################################################
# Ross Arnold, U.S. Army ARDEC
# Aug 21, 2018
#
# Purpose:
#   This is a script to check all the quadrotors on the network to see
#   if they have complete and correct data on their odroids.  This should
#   be used after copying files to the odroids (possibly using inl_push.sh)
#   to ensure that files copied correctly.
#
# Usage:
#   ./check.sh         (checks all quads in the swarm)
#   ./check.sh 115     (checks only quad 115)
#   ./check.sh 115 123 (checks both quads 115 and 123)
#
# Summary:
#   1: Go through each UAV and check if they have particular folders.
#   2: Check if they have correct files in those folders. 
#   3: Check md5 hash of certain files vs. what is on the local PC.
#
##########################################################################

# Define our constants
# Colors first
RED='\033[1;31m'
GREEN='\033[0;32m'
LGREEN='\033[1;32m'
YELLOW='\033[1;33m'
GRAY='\033[1;30m'
WHITE='\033[1;37m'
NC='\033[0m'

# Now folders
TARGET_USER_FOLDER="/home/odroid/"
TARGET_BLESSED_FOLDER="blessed/"
TARGET_PYTHON_FOLDER="scrimmage/usma/plugins/autonomy/python/"
TARGET_AP_ENUM_FOLDER="acs_ros_ws/src/autonomy-payload/ap_lib/src/ap_lib/"
GCS_BLESSED_FOLDER="$HOME/blessed/"
GCS_PYTHON_FOLDER="$HOME/scrimmage/usma/plugins/autonomy/python/"
GCS_AP_ENUM_FOLDER="$HOME/ACS/acs_ros_ws/src/autonomy-payload/ap_lib/src/ap_lib/"

# Now files
AP_ENUM_FILE="ap_enumerations.py"
FENCE_FILE="fence"
PARAM_FILE="param"
RALLY_FILE="rally"
RALLY_TEMPLATE_FILE="rally.template"
WP_FILE="wp"
WP_TEMPLATE_FILE="wp.template"

# Check to see if folder exists
check_folder()
{
  folder_name="$1"
  command="ls $folder_name"
  
  # Send the ls command with our folder name to the odroid to see if it
  # has any contents
  exists=$(sshpass -p 'odroid' ssh -q odroid@192.168.11.$tail_num $command)
  
  # If it shows the file list, i.e. the string is present, then we are OK.
  # Otherwise the test has failed, we add to the error string and return 0
  if [ -z "$exists" ]; then
    failures="$failures \rERROR: $TARGET_USER_FOLDER$folder_name folder not found on vehicle $tail_num"
    return 0
  fi
  return 1
}

# Check to see if file exists and if so, check MD5 hash
check_file()
{
  # $1: folder name
  # $2: file name
  # $3: md5 hash
  file_name_path="$1$2"
  command="stat $file_name_path"
  
  # If there is no md5 hash there was no file on the host system so we couldn't
  # check it, this is a problem
  if [ -z "$3" ]; then
    failures="$failures \r\n ERROR: $2 file not found on local machine"
    return 0
  fi
  
  # Send the stat command with our file name to the odroid to see if it exists
  exists=$(sshpass -p 'odroid' ssh -q odroid@192.168.11.$tail_num $command)
  # If it shows the file list, i.e. the string is present, then we are OK.
  # Otherwise the test has failed, we add to the error string and return 0
  if [ -z "$exists" ]; then
    failures="$failures \r\n ERROR: $TARGET_USER_FOLDER$file_name_path file not found on vehicle $tail_num"
    return 0
  fi
  
  # Now check the md5 hash.
  command="md5sum $file_name_path"
  hash=$(sshpass -p 'odroid' ssh -q odroid@192.168.11.$tail_num $command)
  # For some reason hash also contains the file path concatenated.  So we
  # need to remove that. The following line removes everything after the space
  # in the string.  
  hash=${hash% *}
  if [ $3 != $hash ]; then
    failures="$failures \r\n ERROR: $TARGET_USER_FOLDER$file_name_path MD5 hash failed on vehicle $tail_num"
    return 0
  fi
  return 1
}

check_one_vehicle()
{
  tail_num=$1
  
  #echo Checking vehicle $tail_num...
  
  # Define our failure string
  failures=""
  
  # First check to see that this vehicle is actually available on the net
  nc -z 192.168.11.$tail_num 22
  retv=$?
  if [ $retv -eq 1 ]; then
    echo -e "${GRAY} Vehicle ${WHITE}$tail_num${GRAY} (IP: 192.168.11.$tail_num) not found on network.${NC}"
    return 0
  fi
  
  ########################
  #### BLESSED files #####
  ########################
  #####
  # QUESTION do we really need these on the odroid?  We load them from
  # MAVProxy from the GCS right??? -rda
  #####
  # Check folder on target
  check_folder $TARGET_BLESSED_FOLDER
  return_code=$?

  # Only check the files if the folder exists
  if [ $return_code -eq 1 ]; then
    # Check file on target
    # Get md5 hash from the correct file
    md5=($(md5sum $GCS_BLESSED_FOLDER$FENCE_FILE))
    check_file $TARGET_BLESSED_FOLDER $FENCE_FILE $md5 
    md5=($(md5sum $GCS_BLESSED_FOLDER$PARAM_FILE))
    check_file $TARGET_BLESSED_FOLDER $PARAM_FILE $md5 
    md5=($(md5sum $GCS_BLESSED_FOLDER$RALLY_FILE))
    check_file $TARGET_BLESSED_FOLDER $RALLY_FILE $md5 
    md5=($(md5sum $GCS_BLESSED_FOLDER$RALLY_TEMPLATE_FILE))
    check_file $TARGET_BLESSED_FOLDER $RALLY_TEMPLATE_FILE $md5 
    md5=($(md5sum $GCS_BLESSED_FOLDER$WP_FILE))
    check_file $TARGET_BLESSED_FOLDER $WP_FILE $md5 
    md5=($(md5sum $GCS_BLESSED_FOLDER$WP_TEMPLATE_FILE))
    check_file $TARGET_BLESSED_FOLDER $WP_TEMPLATE_FILE $md5 
  fi

  ########################
  ##### PYTHON files #####
  ########################
  # Check folder on target
  check_folder $TARGET_PYTHON_FOLDER
  return_code=$?

  # Only check the files if the folder exists
  #if [ $return_code -eq 1 ]; then
  # Check file and md5 hash on target
  # TODO - Which file do we want to check??
  #fi

  ########################
  # AP_ENUMERATION files #
  ########################
  # Check folder on target
  check_folder $TARGET_AP_ENUM_FOLDER
  return_code=$?

  # Only check the files if the folder exists
  if [ $return_code -eq 1 ]; then
    # Check file and hash on target
    md5=($(md5sum $GCS_AP_ENUM_FOLDER$AP_ENUM_FILE))
    check_file $TARGET_AP_ENUM_FOLDER $AP_ENUM_FILE $md5
  fi
  
  # Now we do the report.  It only prints failures, which it adds
  # to a failure string.

  if [ ! -z "$failures" ]; then
    echo "******************************************************"
    echo -e " Vehicle ${YELLOW}$tail_num${NC}: Failed with the following errors:${RED}$failures${NC}"
    echo "******************************************************"
  else
    echo -e ${GREEN} Vehicle ${LGREEN}$tail_num${GREEN}: Pass.  All files OK.${NC}
  fi
}

# First make sure our own network is configured properly so we can actually
# reach out to the swarm.
# Get the adapters
#adapters=$(ip link | awk -F: '$0 !~ "lo|vir|^[^0-9]"{print $2;getline}')
# The one we want is the last one
#alfa=$(echo $adapters | awk '{print $NF}')

# Get the IP address of our last adapter which is the ALFA if we have it 
ips=$(hostname -I)
last_ip=$(echo $ips | awk '{print $NF}')

# If our IP isn't right then we need to configure before doing anything else.
if [ $last_ip != "192.168.11.201" ]; then
  echo "WiFi is not configured correctly.  Run wifi_config.sh -T 11 <ADAPTER NAME> 201"
  exit 1
fi

# If we received an input variable, then check a specific vehicle.  Otherwise
# look through all vehicles in the swarm.
if [ ! -z "$1" ]  # Do we have any inputs?
  then
  echo "Beginning file check for vehicle(s) $@..."
  for vnum in "$@" # We have at least one input, so iterate through them all
  do
    if [ $vnum -gt 109 -a $vnum -lt 133 ]
      then
      check_one_vehicle $vnum
    else
      echo "$vnum: Incorrect input.  Tail number must be blank, or between 110 and 132."
      #exit 1
    fi
  done
else  
  echo "Beginning file check for entire swarm..."
  for i in {110..132}
  do
    check_one_vehicle $i
  done
fi

echo Check complete.

