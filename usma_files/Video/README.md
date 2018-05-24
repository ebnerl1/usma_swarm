#usma_swarm Port Stream Setup
1. The next three steps get everything installed on the aerial vehicle to enable you to stream quality video. This is just one way to accomplish this task, feel free to customize the next three steps to your comfort. The key task is to just install gstreamer-tools and v4l2-utils. 

2. Get the ODROID connected to the internet so that it is capable of downloading packages. Execute the following command:
`sudo nano /etc/network/interfaces`
Then comment out the SASC wlan0 interface by entering "#" in front of each line. Below this commented out interface, enter:
`auto wlan0
iface wlan0 inet dhcp
   wpa-ssid "YourSSID"
   wpa-psk "YourPassword"`
Restart the computer and it should connect to the new wireless interface.


3. Install gstreamer-tools:
`sudo apt-get install gstreamer-tools`
You will know the installation is successful if at the end of running the command you see `X newly updated X newly installed X newly upgraded` You can further verify this by running `man gst-launch` A listing of options should appear.

4. Install video for linux (v4l2)
`sudo apt-get install v4l-utils`
`v4l2-ctl --list-formats-ext`
Downloads video for linux and shows you the capabilities of the connected camera. Ensure that "mjpeg" is one of the camera's capabilities.


5. Edit the network interfaces file again and comment out the new interface and restore the vehicles default configuration for the Army IP address. Restart the computer and ensure you can ping the vehicle at its IP address 192.168.11.XXX where XXX is the vehicle ID.


5. In this folder, there are two bash scripts: playvideo.sh and receive_video.sh. Receive_video.sh should be installed on the basestation that you want to play the video on. Playvideo.sh should be installed on the vehicle you wish to stream from.


6. Secure copy the playvideo.sh script to the vehicle you wish to stream video from with the command:
`scp /your/directory/location/playvideo.sh odroid@192.168.11.XXX:/home/odroid` where XXX is the vehicle ID. Once the file is on the vehicle, execute the command `nano playvideo.sh` and change the port number to 5000+XXX (do the math). I.E.: If the vehcile is 25, then change the port number to 5025. Save and exit the script.


7. Now, you should be able to play the video from the vehicle. SSH into the vehicle and ensure that a usb camera is connected. `ls /dev | grep "video"` If no "video0" device is listed, ensure the usb camera is properly connected and restart the computer. It is likely that the camera is listed as `video1` so restarting the computer will switch it back to `video0`. 


8. To start a video stream, simply enter the command on the vehicle through SSH: `./playvideo.sh`


9. On your basestation, ensure you are on the same network as your aerial vehicles. Run the command `./receive_video.sh XXX` where XXX is the vehicle ID. The video stream should appear on the base station. If you wish to open multiple video streams, just open another terminal and run the same command (for a different vehicle). IMPORTANT!!! The base station MUST BE 192.168.11.203. If you want it to be another computer, you have to SSH into the vehicle that you wish to receive the video stream from and `nano playvideo.sh` and change the IP address to the basestation that you want to receive video on. For now, it is just configured to stream to .203 


10. Once the video appears on your screen, you might notice some white balance issues. To fix this, please reference the file "camera adjustments." There are multiple camera settings that you can adjust with v4l2, but the one that usually always worked for me was adjusting the power_line_frequency (the instructions for how to do this are in the camera_adjustments file. 

11. If you wish to save the video that you are capturing, you need to run "Simple Screen Recorder." The instructions to do this are located in this folder and is named "Install Screen Recorder." With Simple Screen Recorder, you can capture a video stream of your screen to include the video you are capturing as well as the commands you are running (good for presentations!)

12. Lastly, on some vehicles I noticed that there is an error that appears after trying to execute the command `./playvideo.sh`. The error that appears is "device or resoruce busy". To fix this error, please consult the "fix_device_busy" file that is in this folder.

If you have any further questions concerning video streaming from the odroid camera, the ODROID forums are a great place to start. If you are extremely desperate, you can contact me: Sam Miller (samuelmiller0714@gmail.com) to see if I remember exactly how to do it. Best of luck in accomplishing your project goals!
