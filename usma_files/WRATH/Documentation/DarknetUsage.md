# Darknet Install

I'm sorry, but I have no idea where my how to install file went, and I don't remember how to install darknet. Here are some points though:

* You must install OpenCV
* You must install Darknet_ROS. This is just a git repository to pull from

# Darknet Usage

These instructions need to be run on the Nano with a valid Darknet Install

1. Open new terminal for the ROS master
    1. `cd ~/catkin_ws`
    1. `catkin_make`
    1. `roscore`
1. Open new terminal for Darknet
    1. `cd ~/catkin_ws`
    1. `source ./devel/setup.bash`
    1. `roslaunch darknet_ros darknet_ros.launch`
1. Open new terminal for Webcam
    1. `cd ~/catkin_ws`
    1. `source ./devel/setup.bash`
    1. `rosrun swarms webcam.py`
