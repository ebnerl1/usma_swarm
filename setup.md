# usma_swarm Video Stream Setup

1. Connect HDMI, keyboard, Ethernet cable to ODROID and power on.

2. Ensure SASC is installed

3. Configure an Ethernet interface in the /etc/network/interfaces file

* If an ethernet cable is used, ensure that the /etc/network/interfaces file’s referent to the wired adapter, usually eth0, is set to DHCP:

    `auto eth0`
    `iface eth0 inet`

  * If using basestation (router with no security set), use the following command and replace wlan0 and basestation with whatever corresponds to the current setup:

  `sudo iwconfig wlan0 essid basestation`

4. Download packages. All packages start with:

    `sudo apt-get install ros-<version>-`

  Followed by the package name, with all underscores replaced by hyphens. The packages needed are:

  * To be able to stream video through image_view:

  * usb_stream
	* compressed_image_transport
	* image_view

  * For the GUI:

  * rosbridge_suite
  * web_video_server

5. Additionally, GUI install requires:

    `$ rosdep update'
    `$ rosdep install rosbridge_server`
    `$ rosdep install web_video_server`
    `$ roslaunch rosbridge_server rosbridge_websocket.launch`
    `$ rosrun web_video_server web_video_server`

6. Edit ~/.bashrc to include:

    `source devel/setup.bash`
    `export ROS_IP=192.168.11.X (or whatever the machine’s own IP is, can be found with hostname –I or just ifconfig`
    `export ROS_MASTER_URI=http://192.168.11.201:11311`

  * In some cases when a new package isn’t recognized by ros, you may also need:

    `. ~/catkin_ws/devel/setup.bash`

  * Remember to save and then enter the command:

    `source ~/.bashrc`

7. Change the /etc/hosts file on the drone to include the drone’s name in the 127.0.0.1 localhost address, the Master’s name and IP

  * Do the same with the master, just the opposite

8. Add our little piece from drone_stream to the master.launch file in acs_ros_ws in the ap_master package

9. Ensure that the basestation is on the same subnet and run the command:

    `wifi_config.sh -T 11 <wlan name> 201`

10. On the quadrotor, unconfigure the Ethernet interface and reconfigure the static ip in the 192.168.11.0/24 subnet 10. Ensure you can ssh into the quadrotor from the basestation (ROS computer)

11. Connect the web cam and run the following command:

    `ls /dev/video*`

12. Ensure "video0" is present, or whichever one is the camera.

13. Run roscore on the basestation and then run the master.launch on the quad.

14. If the master.launch launches error-free (a few warnings are okay), execute the following command on the basestation:

    `rostopic list`

  * You should see the new video topic. To view, run:

    `rosrun image_view image_view image:=<topic> compressed`

  * To view the republished compressed video, just change the desired image topic
  * To use the GUI launch:

    `$ roslaunch rosbridge_server rosbridge_websocket.launch`
    `$ rosrun web_video_server web_video_server`

###Troubleshooting:

Here are some of the issues I’ve run into, sometimes a reboot will fix the issue

Cannot contact MASTER:
* Ping the drone or master to ensure connection
* Inspect the hosts file on each
* Inspect the .bashrc, source the .bashrc
* Make sure roscore is running

 Cannot find AF_INET:
* Export the ROS_IP

usb_cam process has died:
* Check to make sure the master.launch video settings make sense. The camera may not be able to shoot in too high of resolution (e.g. 1080p on a 720p maximum camera), or the height and width may be switched
* Make sure the video format is yuyv, not mjpeg
*	Unplug and replug the camera, then relaunch

Namespace issues:
* Make sure the topics are published and republished with different names. This can be changed in the launch file.

GUI is unable to subscribe to topics:
* Refresh the web page
* Currently there must be an internet connection with the master
* Currently, image_view must be opened and the GUI can only subscribe to those topics

A lot of latency:
* Relaunch the stream on the drone side
* Make sure the subscription is to a compressed stream
* Sometimes, the lag is on the application side or from the camera itself as it adjusts its exposure, etc.
