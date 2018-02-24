#!/bin/bash

#this script requires that you create a directory called 'odroid_images' in the home folder
#Change 'rrc' to the name of the user 

PICFOLDER='/home/rrc/odroid_images'
DATE=`date '+%Y-%m-%d-%H-%M-%S'`;

curl -s -f -m 5 http://odroid:odroid@192.168.11.125:8090/?action=snapshot > "$PICFOLDER/camera-$DATE.jpeg"

if [ -s "$PICFOLDER/camera-$DATE.jpeg" ]; then
    cp -f "$PICFOLDER/camera-$DATE.jpeg" /tmp/.camera.jpeg
    convert "/tmp/.camera.jpeg" -gravity northwest -pointsize 16 -undercolor 'rgba(255,155,0,0.6)' -fill black -annotate +5+5 "ODROID Snapshot $DATE" "$PICFOLDER/camera-$DATE.jpeg"
else
    logger -t $0 "unable to save $PICFOLDER/camera-$DATE.jpeg"
    sudo systemctl restart mjpeg_streamer.service
fi
