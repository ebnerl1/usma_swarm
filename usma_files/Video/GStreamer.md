#usma_swarm Port Stream Setup

1. Get the ODROID connected to the internet so that it is capable of downloading packages. Execute the following command:
`sudo nano /etc/network/interfaces`
Then comment out the SASC wlan0 interface by entering "#" in front of each line. Below this commented out interface, enter:
`auto wlan0
iface wlan0 inet dhcp
   wpa-ssid "YourSSID"
   wpa-psk "YourPassword"`
Restart the computer and it should connect to the new wireless interface.


2. Install gstreamer-tools:
`sudo apt-get install gstreamer-tools`
You will know the installation is successful if at the end of running the command you see `X newly updated X newly installed X newly upgraded` You can further verify this by running `man gst-launch` A listing of options should appear.


3. Edit the network interfaces file again and comment out the new interface and restore the vehicles default configuration for the Army IP address. Restart the computer and ensure you can ping the vehicle at its IP address 192.168.11.XXX where XXX is the vehicle ID.


4. In this folder, there are two bash scripts: playvideo.sh and receive_video.sh. Receive_video.sh should be installed on the basestation that you want to play the video on. Playvideo.sh should be installed on the vehicle you wish to stream from.


5. Secure copy the playvideo.sh script to the vehicle you wish to stream video from with the command:
`scp /your/directory/location/playvideo.sh odroid@192.168.11.XXX:/home/odroid` where XXX is the vehicle ID. Once the file is on the vehicle, execute the command `nano playvideo.sh` and change the port number to 5000+XXX (do the math). I.E.: If the vehcile is 25, then change the port number to 5025. Save and exit the script.


6. Now, you should be able to play the video. Ensure that a usb camera is connected. `ls /dev | grep "video0"` If no device is listed, ensure the usb camera is properly connected and restart the computer. It is likely that the camera is listed as `video0` so restarting will switch it back to `video0`. 


7. To start a video stream, simply enter the command on the vehicle (or through SSH) `./playvideo.sh`


8. On your basestation, run the command `./receive_video.sh XXX` where XXX is the vehicle ID. The video stream should appear on the base station. The base station MUST BE 192.168.11.203. If you want it to be another computer, you have to SSH into the vehicle that you wish to receive the video stream from and `nano playvideo.sh` and change the IP address to the basestation that you want to receive video on.


9. On the basestation, download the "Snap-odroid-image" script from /usma_swarm/usma_files/Video which lets you run a script to take a picture from the odroid and label that picture with some basic information.
