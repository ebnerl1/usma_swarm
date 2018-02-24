#usma_swarm Port Stream Setup

1. Get the ODROID connected to the internet so that it is capable of downloading packages. Execute the following command:
`sudo nano /etc/network/interfaces`
Then comment out the SASC wlan0 interface by entering "#" in front of each line. Below this commented out interface, enter:
`auto wlan0`
`iface wlan0 inet dhcp`
`   wpa-ssid "YourSSID"`
`   wpa-psk "YourPassword"`
Restart the computer and it should connect to the new wireless interface.


2. The following commands work through the tutorial at https://forum.odroid.com/viewtopic.php?f=52&t=23503
`sudo apt-get install v4l-utils`
`v4l2-ctl --list-formats-ext`
Downloads video for linux and shows you the capabilities of the connected camera. Ensure that "mjpeg" is one of the camera's capabilities.


3. To view the parameters that you can change on your camera, execute the following command:
`v4l2-ctl --list-ctrls`


4. How to enable video streaming & image capture via the ODROID camera:
`sudo apt-get update`
`git clone https://github.com/jacksonliam/mjpg-streamer.git`
`cd mjpg-streamer/mjpg-streamer-experimental`
`sudo apt-get install cmake libjpeg62-dev`
`make`
`sudo make install`


5. Starts mjpg-streamer as a web server with authentication and read from the camera (RUN THIS ON ODROID):
`sudo /usr/local/bin/mjpg_streamer -i 'input_uvc.so -r 1280x720 -m 50000 -n -f 25 -d /dev/video0' -o 'output_http.so -p 8090 -w /usr/local/share/mjpg-streamer/www/ -c odroid:odroid'`


6. Then, you should be able to browse to the odroid's IP and view the page with video (RUN ON BASESTATION):
`http://192.168.11.X:8090`


7. To grab a single image from the camera and store it in a directory (RUN THIS ON BASESTATION):
`sudo apt-get install curl`
`curl -s -f -m  http://odroid:odroid@192.168.11.X:8090/?action=snapshot > /your/directory/here`


8. On your basestation, install vlc media player:
`sudo apt-get install vlc`
`sudo adduser username video`
`vlc http://odroid:odroid@192.168.11.X:8090/?action=stream`
You must run these commands while the stream is open in the web server.


9. On the basestation, download the "Snap-odroid-image" script from /usma_swarm/usma_files/Video which lets you run a script to take a picture from the odroid and label that picture with some basic information.
