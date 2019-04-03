#!/bin/bash

#------------------------------------------------------------------------------
# Purpose: Set up companion computer, once Ubuntu is installed
#          Written for ODroid U3 running "standard" Ubuntu install
# Author: Mike Clement
#------------------------------------------------------------------------------

ROS_DISTRO="indigo"
APT_OPTS="-o Dpkg::Options::=--force-confdef -o Dpkg::Options::=--force-confask"

SSL_PREFIX="https://gitlab.nps.edu/"
SSH_PREFIX="git@gitlab.nps.edu:"

###############################################################################
# Helper functions

# Function to check for command failure and print useful error messages
function check_fail
{
if [ $? != 0 ]; then
  echo -e "\nERROR: $1\n"
  kill -INT $$
fi
}

###############################################################################
# Parition various pieces of the install
# Note: we try to make each function idempotent, so it can safely be
# run over and over again, and used as an upgrade script.

############################################################
# Perform initial configuration (generally only needed once)
############################################################
function install_initial
{

# Replacing resolv.conf later, don't hold up sudo resolution
sudo sh -c 'echo -n > /etc/resolv.conf'

# Make it so this user can do passwordless sudo :)
sudo grep -x "$USER ALL=(ALL) NOPASSWD: ALL" /etc/sudoers > /dev/null
if [ $? != 0 ]; then
  sudo chmod u+w /etc/sudoers
  check_fail "chmod (sudo permissions)"
  echo "$USER ALL=(ALL) NOPASSWD: ALL" | sudo tee -a /etc/sudoers > /dev/null
  check_fail "echo (sudo permissions)"
  sudo chmod u-w /etc/sudoers
  check_fail "chmod (sudo permissions)"
fi

# Set the hostname
sudo hostname $AIRCRAFT_NAME
check_fail "set temporary hostname"
sudo sh -c "echo $AIRCRAFT_NAME > /etc/hostname"
check_fail "set persistent hostname"
sudo sed -ri "s/\s+localhost.*/\tlocalhost\t$AIRCRAFT_NAME/" /etc/hosts
check_fail "set loopback hostname"

# Fix things that make SSH logins slow
grep UseDNS /etc/ssh/sshd_config > /dev/null
if [ $? == 0 ]; then
  sudo sed -i s/UseDNS\ yes/UseDNS\ no/ /etc/ssh/sshd_config
else
  sudo sh -c 'echo "UseDNS no" >> /etc/ssh/sshd_config'
fi
grep UsePAM /etc/ssh/sshd_config > /dev/null
if [ $? == 0 ]; then
  sudo sed -i s/UsePAM\ yes/UsePAM\ no/ /etc/ssh/sshd_config
else
  sudo sh -c 'echo "UsePAM no" >> /etc/ssh/sshd_config'
fi

# Disable services we don't plan to use
for s in ntp pppd-dns rsync saned x11-common; do
  sudo service $s stop
  sudo update-rc.d -f $s remove
done
for s in bluetooth cups cups-browsed lightdm modemmanager network-manager whoopsie; do
  sudo sh -c "echo manual > /etc/init/${s}.override"
done

# Update locale settings
sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX

# Disable automatic apt tasks
sudo rm /etc/cron.daily/apt

# Remove any old wireless device entries from udev, and add a
# generic rule to make *any* wifi device 'wlan0'
cat > udev.tmp <<EOF
# Catch-all for wlan devices
SUBSYSTEM=="net", ACTION=="add", DRIVERS=="?*", ATTR{dev_id}=="0x0", ATTR{type}=="1", KERNEL=="wlan*", NAME="wlan0"
EOF
check_fail "cat (udev rules)"
sudo mv udev.tmp /etc/udev/rules.d/70-persistent-net.rules
check_fail "mv (udev rules)"
sudo chown root:root /etc/udev/rules.d/70-persistent-net.rules
check_fail "chown (udev rules)"

# Create network configuration
SUBNET=$TEAM_ID
if [ $TEAM_ID == 3 ]; then
    ESSID="redzephyr"
    CHANNEL=11
    AP="00:55:44:33:22:11"
elif [ $TEAM_ID == 11 ]; then
    ESSID="army"
    CHANNEL=1
    AP="00:99:88:77:66:55"
elif [ $TEAM_ID == 12 ]; then
    ESSID="navy"
    CHANNEL=6
    AP="00:88:77:66:55:44"
elif [ $TEAM_ID == 13 ]; then
    ESSID="airforce"
    CHANNEL=11
    AP="00:77:66:55:44:33"
else  # Assumes team is blue (2)
    ESSID="zephyr"
    CHANNEL=6
    AP="00:11:22:33:44:55"
fi

cat > interfaces.tmp <<EOF
auto lo
iface lo inet loopback

auto eth0
iface eth0 inet static
    address 192.168.0.1
    netmask 255.255.255.0
    dns-nameservers 4.4.4.4 8.8.8.8

auto wlan0
iface wlan0 inet static
    address 192.168.${SUBNET}.${AIRCRAFT_ID}
    netmask 255.255.255.0
    gateway 192.168.${SUBNET}.1
    dns-nameservers 10.179.10.9 8.8.8.8 4.4.4.4
    wireless-mode ad-hoc
    wireless-essid ${ESSID}
    wireless-channel ${CHANNEL}
    wireless-ap ${AP}
    wireless-txpower 10
EOF
check_fail "cat (interfaces config)"
sudo mv interfaces.tmp /etc/network/interfaces
check_fail "mv (interfaces config)"
sudo chown root:root /etc/network/interfaces
check_fail "chown (interfaces config)"

# Make sure nsswitch is allowing DNS lookups
sudo sed -i 's/\[NOTFOUND\=return\] dns/dns \[NOTFOUND\=return\]/' /etc/nsswitch.conf

# Replace /etc/resolv.conf
#sudo sh -c "rm /etc/resolv.conf && echo \"nameserver 172.20.20.11\nnameserver 8.8.8.8\" > /etc/resolv.conf"
#sudo sh -c "rm /etc/resolv.conf && echo \"nameserver 8.8.8.8\nnameserver 4.4.4.4\" > /etc/resolv.conf"
sudo sh -c "rm /etc/resolv.conf && echo \"nameserver 10.179.10.9\nnameserver 8.8.8.8\nnameserver 4.4.4.4\" > /etc/resolv.conf"
check_fail "resolv conf"

}

#############################################
# Set up network device for immediate install
#############################################
function install_setup_networking
{

# Stand up interface as-is for this boot
if [ -z $NET_DEVICE ]; then
  # If not specified, look for first wireless interface
  NET_DEVICE=`sudo ifconfig -a | grep -m 1 wlan | awk '{print $1}'`
fi
sudo ifconfig $NET_DEVICE down
check_fail "ifconfig down"
sudo iwconfig $NET_DEVICE mode ad-hoc essid $ESSID channel $CHANNEL ap $AP txpower 20
# iface might not be wireless, let this fail
#check_fail "iwconfig"
sudo ifconfig $NET_DEVICE inet 192.168.$TEAM_ID.$AIRCRAFT_ID/24 up
check_fail "ifconfig up"
sudo route add default gw 192.168.$TEAM_ID.1
check_fail "route add"

}

###############################
# Do base software installation
###############################
function install_base_software
{

NO_CLONE=$1

# Add ROS repo
ls /etc/apt/sources.list.d/ros-latest.list &> /dev/null
if [ $? != 0 ]; then
  sudo sh -c 'wget http://packages.namniart.com/repos/namniart.key -O - | apt-key add -'
  check_fail "ROS apt-key"
  sudo sh -c 'echo "deb http://packages.namniart.com/repos/ros trusty main" > /etc/apt/sources.list.d/ros-latest.list'
  check_fail "ROS apt source"
fi

# Update package lists
sudo apt-get update
check_fail "apt-get update"

# Remove unneeded packages
sudo apt-get --assume-yes remove flash-kernel

if [ $UBUNTU_UPDATE = true ]; then
    # Upgrade existing packages
    sudo apt-get --assume-yes $APT_OPTS upgrade
fi

# There's a known bug with some base images
sudo apt-get --reinstall $APT_OPTS install python-debian
sudo apt-get --reinstall $APT_OPTS install python-chardet
sudo apt-get --reinstall $APT_OPTS install python-pip 

#install dependencies
sudo apt-get install python-lxml

if [ $UBUNTU_UPDATE = true ]; then
    # Upgrade existing packages
    sudo apt-get --assume-yes $APT_OPTS upgrade
fi

check_fail "apt-get upgrade"

# Install ROS and other useful packages
sudo apt-get --assume-yes $APT_OPTS install \
git \
ros-${ROS_DISTRO}-ros-base \
ros-${ROS_DISTRO}-sensor-msgs \
ros-${ROS_DISTRO}-tf \
ros-${ROS_DISTRO}-yocs-cmd-vel-mux \
iperf \
python-netifaces \
python-serial \
python-setuptools \
libeigen3-dev \
cmake \
build-essential \
screen
check_fail "apt-get install"

sudo pip install future
sudo pip install angles

# Regrettably, we need to disable Git's SSL cert check
git config --global http.sslVerify false
check_fail "git_config"

# Set up ROS
# (some of these might fail after the first successful run, don't worry about error checking)
sudo rosdep init
rosdep update
grep "/opt/ros/${ROS_DISTRO}/setup.bash" ~/.bashrc
if [ $? != 0 ]; then
  echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
fi
source /opt/ros/${ROS_DISTRO}/setup.bash

# Clean up deb files to free up some space
sudo apt-get clean

#cd ~
#DIR_NAME="geographiclib"
#if [ $NO_CLONE = false ]; then
#    mkdir -p ${HOME}/builds
#    pushd ${HOME}/builds >& /dev/null
#       URL="git://git.code.sourceforge.net/p/geographiclib/code"
#       REMOTE_NAME="origin"
#       REF="release"
#       if [ ! -d "${DIR_NAME}" ]; then
#           git clone "${URL}" "${DIR_NAME}"
#       fi
#       pushd ${DIR_NAME} >& /dev/null
#          git checkout ${REF}         
#          git fetch ${REMOTE_NAME}
#          git reset --hard ${REMOTE_NAME}/${REF}
#       popd >& /dev/null
#    popd >& /dev/null
#fi
#
## Build geographiclib
#pushd ${HOME}/builds/$DIR_NAME/build >& /dev/null
#   cmake ..
#   make
#   sudo make install
#popd >& /dev/null

}

#####################################
# Install payload software from repos
#####################################
function install_payload_software
{

USE_SSH=$1
NO_CLONE=$2

if [ $USE_SSH = true ]; then
    PREFIX="${SSH_PREFIX}"
else
    PREFIX="${SSL_PREFIX}"
fi

## Clone / Update scrimmage (only need headers, no build)
#cd ~
#if [ $NO_CLONE = false ]; then
#    mkdir -p ${HOME}/scrimmage
#    pushd ${HOME}/scrimmage >& /dev/null
#
#       # clone scrimmage
#       DIR_NAME="scrimmage"
#       URL="${PREFIX}sasc/${DIR_NAME}.git"       
#       REMOTE_NAME="origin"
#       REF="master"
#       if [ ! -d "${DIR_NAME}" ]; then
#           git clone "${URL}" "${DIR_NAME}"
#       fi
#       pushd ${DIR_NAME} >& /dev/null
#          git checkout ${REF}   
#          git remote set-url "${REMOTE_NAME}" "${URL}" > /dev/null 2>&1
#          git fetch ${REMOTE_NAME}
#          git reset --hard ${REMOTE_NAME}/${REF}
#       popd >& /dev/null
#       
#       # Clone scrimmage-templates
#       DIR_NAME="scrimmage-templates"
#       URL="${PREFIX}sasc-itar/${DIR_NAME}.git"       
#       REMOTE_NAME="origin"
#       REF="master"
#       if [ ! -d "${DIR_NAME}" ]; then
#           git clone "${URL}" "${DIR_NAME}"
#       fi
#       pushd ${DIR_NAME} >& /dev/null
#          git checkout ${REF}   
#          git remote set-url "${REMOTE_NAME}" "${URL}" > /dev/null 2>&1
#          git fetch ${REMOTE_NAME}
#          git reset --hard ${REMOTE_NAME}/${REF}                 
#       popd >& /dev/null
#    popd >& /dev/null
#fi
#
## Run cmake/make in scrimmage-templates.
#pushd ${HOME}/scrimmage/scrimmage-templates >& /dev/null
#   mkdir -p build
#   pushd ./build >& /dev/null
#      cmake ..
#      make
#   popd >& /dev/null
#popd >& /dev/null
#
## Put SCRIMMAGE_ROOT in .bashrc and export it for catkin_make   
#export SCRIMMAGE_ROOT=${HOME}/scrimmage/scrimmage
#grep "SCRIMMAGE_ROOT" ~/.bashrc &> /dev/null
#if [ $? != 0 ]; then
#    echo "export SCRIMMAGE_ROOT=${SCRIMMAGE_ROOT}" >> ~/.bashrc
#fi
#
## Create the tactic interface select file, if it doesn't exist.
#TACTIC_SELECT_FILE=$(readlink -f "${HOME}/tactic-interface-env.sh")
#if [ ! -e ${TACTIC_SELECT_FILE} ]; then
#    echo "#!/bin/bash" > ${TACTIC_SELECT_FILE}
#fi
#chmod +x ${TACTIC_SELECT_FILE}
#
## Append the source line if setenv.sh doesn't exist
#grep "setenv.sh" $TACTIC_SELECT_FILE &> /dev/null
#if [ $? != 0 ]; then
#    echo "source ${HOME}/scrimmage/scrimmage-templates/build/setenv.sh" >> ${TACTIC_SELECT_FILE}
#fi   

# Install (py)mavlink library
if [ $NO_CLONE = false ]; then
    cd ~
    ls mavlink/ &> /dev/null
    if [ $? != 0 ]; then    
        git clone ${PREFIX}sasc/mavlink.git
        check_fail "mavlink git clone"
        cd mavlink/
        git checkout dev  # The branch we use for production
        check_fail "mavlink git checkout dev"
        git submodule sync
        git submodule init
        git submodule update        
    else
        cd mavlink/
        git fetch $GIT_REMOTE # update local copy of remote repo
        check_fail "mavlink git fetch"
        git checkout .  # discard any local changes
        check_fail "mavlink git checkout ."
        git clean -df  # discard any untracked files
        check_fail "mavlink git clean"
        git checkout dev  # have the right branch checked out
        check_fail "mavlink git checkout dev"
        git reset --hard $GIT_REMOTE/dev  # bring in updates
        check_fail "mavlink git reset"

        git submodule sync
        git submodule init
        git submodule update
    fi
fi

# Set dialect so we don't compile the universe
env | grep "MAVLINK_DIALECT" &> /dev/null
if [ $? != 0 ]; then
  export MAVLINK_DIALECT=ardupilotmega
fi
grep "MAVLINK_DIALECT" ~/.bashrc &> /dev/null
if [ $? != 0 ]; then
  echo "export MAVLINK_DIALECT=ardupilotmega" >> ~/.bashrc
fi


rm -rf ~/.local/  # Clear out old build
check_fail "mavlink remove .local/"

cd ~/mavlink/pymavlink
python setup.py build install --user
check_fail "mavlink setup.py"

# Initialize the ROS workspace
# (highly unlikely we'll see failures here; ignoring check_fail())
if [ $NO_CLONE = false ]; then
    cd ~
    ls acs_ros_ws/src/ &> /dev/null
    if [ $? != 0 ]; then
        mkdir -p acs_ros_ws/src/                
    fi
fi

cd ~/acs_ros_ws/src
catkin_init_workspace
cd ..
catkin_make
echo "source ~/acs_ros_ws/devel/setup.bash" >> ~/.bashrc
source ~/acs_ros_ws/devel/setup.bash

if [ $NO_CLONE = false ]; then
    # Clone or update the autonomy-payload repo
    cd ~/acs_ros_ws/src/
    ls autonomy-payload/ &> /dev/null
    if [ $? != 0 ]; then
        git clone ${PREFIX}sasc/autonomy-payload.git
        check_fail "payload git clone"
    else
        cd autonomy-payload
        git fetch $GIT_REMOTE # update local copy of remote repo
        check_fail "payload git fetch"
        git checkout .  # discard any local changes
        check_fail "payload git checkout ."
        git clean -df  # discard any untracked files
        check_fail "payload git clean"
        git checkout master  # have the right branch checked out
        check_fail "payload git checkout master"
        git reset --hard $GIT_REMOTE/master  # bring in updates
        check_fail "payload git reset"
    fi
fi

if [ $NO_CLONE = false ]; then
    # Clone or update the autopilot_bridge repo
    cd ~/acs_ros_ws/src/
    ls autopilot_bridge/ &> /dev/null
    if [ $? != 0 ]; then    
        git clone ${PREFIX}sasc/autopilot_bridge.git
        check_fail "mavbridge git clone"
    else
        cd autopilot_bridge
        git fetch $GIT_REMOTE # update local copy of remote repo
        check_fail "mavbridge git fetch"
        git checkout .  # discard any local changes
        check_fail "mavbridge git checkout ."
        git clean -df  # discard any untracked files
        check_fail "mavbridge git clean"
        git checkout master  # have the right branch checked out
        check_fail "mavbridge git checkout master"
        git reset --hard $GIT_REMOTE/master  # bring in updates
        check_fail "mavbridge git reset"
    fi
fi

if [ $NO_CLONE = false ]; then
    # Clone or update the autonomy_itar repo
    cd ~/acs_ros_ws/src/
    ls autonomy_itar/ &> /dev/null
    if [ $? != 0 ]; then
        git clone ${PREFIX}sasc-itar/autonomy_itar.git
        check_fail "itar git clone"
    else
        cd autonomy_itar
        git fetch $GIT_REMOTE # update local copy of remote repo
        check_fail "itar git fetch"
        git checkout .  # discard any local changes
        check_fail "itar git checkout ."
        git clean -df  # discard any untracked files
        check_fail "itar git clean"
        git checkout master  # have the right branch checked out
        check_fail "itar git checkout master"
        git reset --hard $GIT_REMOTE/master  # bring in updates
        check_fail "itar git reset"
    fi
fi

# Build all workspace packages
# NOTE: This now also builds all shared libs in autonomy-payload
cd ~/acs_ros_ws/
catkin_make clean
check_fail "catkin_make clean"
catkin_make
check_fail "catkin_make"

## Make sure ~/odroid-installer.sh is softlinked to repo version
rm ~/odroid-installer.sh &> /dev/null
ln -s ~/acs_ros_ws/src/autonomy-payload/deploy/odroid-installer-14.04.sh ~/odroid-installer.sh
if [ $? != 0 ]; then
  echo "WARNING: could not softlink install script into home!"
fi

}

###################################################
# Function to set up auto-start of payload software
###################################################
function install_autostart
{

# Set up automatic start-on-boot
cat > init.tmp <<EOF
#!/bin/sh

# Startup script for autonomy-payload

PIDFILE=~odroid/autonomy-payload.pid

case \$1 in
  start)
    echo "*** STARTING AUTONOMY PAYLOAD ***"
    su -l -c "source /opt/ros/${ROS_DISTRO}/setup.bash; source ~/acs_ros_ws/devel/setup.bash; source ~/tactic-interface-env.sh; roslaunch --pid=\$PIDFILE ap_master master.launch id:=${AIRCRAFT_ID} name:=${AIRCRAFT_NAME} &" odroid
    ;;
  stop)
    start-stop-daemon --stop --pidfile \$PIDFILE
    rm \$PIDFILE
    ;;
  *)
    echo "Please use start or stop."
    ;;
esac
EOF
check_fail "init.d cat"
sudo mv init.tmp /etc/init.d/autonomy-payload
check_fail "init.d mv"
sudo chown root:root /etc/init.d/autonomy-payload
check_fail "init.d chown"
sudo chmod 755 /etc/init.d/autonomy-payload
check_fail "init.d chmod"
sudo update-rc.d autonomy-payload defaults
check_fail "init.d update"

}

###############################################################################
# Main program flow

INSTALL_CONF=true   # Do initial configuration (IP, Name, etc)
INSTALL_NETW=true   # Do immediate network device config
INSTALL_BASE=true   # Install base software (ROS, etc)
INSTALL_PAYL=true   # Install autonomy payload software
INSTALL_INIT=true   # Set up init.d script
NET_DEVICE=         # Network device for installation (if not wlan*)
GIT_REMOTE="origin" # Git repository for payload install
USE_SSH=false
NO_CLONE=false
UBUNTU_UPDATE=true

# Parse any arguments
while getopts ":d:g:uqcsknx" opt; do
  case $opt in
    d)
      if [ -z $OPTARG ]; then
        echo "Please specify a device"
        exit 1
      fi
      NET_DEVICE=$OPTARG
      ;;
    g)
      if [ -z $OPTARG ]; then
        echo "Please specify a Git remote"
        exit 1
      fi
      GIT_REMOTE=$OPTARG
      ;;
    u)
      INSTALL_CONF=false
      INSTALL_NETW=false
      INSTALL_INIT=false
      ;;
    q)
      INSTALL_CONF=false
      INSTALL_NETW=false
      INSTALL_BASE=false
      INSTALL_INIT=false
      ;;
    c)
      INSTALL_NETW=false
      INSTALL_BASE=false
      INSTALL_PAYL=false
      ;;
    s)
      INSTALL_INIT=false
      ;;
    k)
      USE_SSH=true
      ;;      
    n) 
      NO_CLONE=true
      ;;
    x)
      UBUNTU_UPDATE=false
      ;;
    \?)
      echo ""
      echo "usage: $0 [options]"
      echo "  -d DEVICE   Network device to configure for installation"
      echo "  -r REMOTE   Git remote to use for fetching"
      echo "  -u          Update only (skip initial config and init.d script)"
      echo "  -q          Quick update only (ALSO skip base software install)"
      echo "  -c          Configuration and script only (no software install)"
      echo "  -s          DON'T install startup (init.d) script"
      echo "  -k          Use ssh key"
      echo "  -n          Don't clone some repos"
      echo "  -x          Skip ubuntu upgrade step (for faster intall)"
      echo ""
      exit 0
      ;;
  esac
done

# We want to run as the actual user, not as root
if [ `whoami` == "root" ]; then
  echo "Please run as the user that will run the payload software."
  exit 1
fi

# Stop any running payload
echo "Stopping payload software ..."
sudo service autonomy-payload stop

if [ $INSTALL_CONF == true ]; then
  # Collect any needed user information
  echo ""
  read -p "Please enter a unique numeric ID for this aircraft: " AIRCRAFT_ID
  if [ -z $AIRCRAFT_ID ]; then
    echo "No ID specified; aborting"
    exit 1
  fi
  read -p "Please enter the team (2 for blue, 3 for red, 11 for army, 12 for navy, 13 for air force) for this aircraft: " TEAM_ID
  if [ -z $TEAM_ID ]; then
    echo "No team specified; aborting"
    exit 1
  fi
  read -p "Please enter a hostname for this aircraft (max 16 chars): " AIRCRAFT_NAME
  if [ -z $AIRCRAFT_NAME ]; then AIRCRAFT_NAME='odroid'; fi

  install_initial
fi

if [ $INSTALL_NETW == true ]; then
  install_setup_networking
fi

# Software install steps require both Internet and correct time
if [ $INSTALL_BASE == true ] || [ $INSTALL_PAYL == true ]; then
  # Make sure user has Internet connection set up
  echo ""
  read -p "Please make sure you have an appropriate Internet gateway set up, then press Enter. "

  # Shut down NTP service and get a proper time hack
  # (important for make/catkin_make to work correctly)
  sudo service ntp stop
  #sudo ntpdate time.nps.edu  # This should be customized later
  sudo ntpdate us.pool.ntp.org
  if [ $? != 0 ]; then
    echo ""
    echo "Could not sync time using NTP (perhaps check your network connection?)"
    #don't faill if we can't hit the time server
    #echo ""
    #read -p "Enter to abort, or enter time (MMDDhhmmYYYY): " CURRENT_TIME
    #if [ -z $CURRENT_TIME ]; then exit 1; fi
    #sudo date $CURRENT_TIME
    #check_fail "set date/time"
  fi
fi

# Conditionally perform software installs
if [ $INSTALL_BASE == true ]; then
  #echo "TODO => NOT INSTALLING BASE"
  install_base_software $NO_CLONE
fi
if [ $INSTALL_PAYL == true ]; then
  install_payload_software $USE_SSH $NO_CLONE

  # Perform any remediation steps (fixes discovered later on)
  # In a separate script so we always run the latest version
  . ~/acs_ros_ws/src/autonomy-payload/deploy/remediate-14.04.sh
fi

# Conditionally install init.d script
if [ $INSTALL_INIT == true ]; then
  install_autostart
fi

# Final message(s) to user
echo ""
echo "Congratulations, your system has been updated. Please check"
echo "that the correct branches of mavlink, autonomy-payload, and"
echo "and autopilot_bridge have been checked out. Then, reboot to"
echo "automatically start the payload software."
echo ""
ls ~/bags/*bag* &> /dev/null
if [ $? == 0 ]; then
  echo "WARNING: There are ROS bags in ~/bags/;"
  echo "you may wish to clear these off before flight!"
  echo ""
fi
ls ~/.ros/log/ &> /dev/null
if [ $? == 0 ]; then
  echo "WARNING: There are ROS logs in ~/.ros/log/;"
  echo "you may wish to clear these off before flight!"
  echo ""
fi
