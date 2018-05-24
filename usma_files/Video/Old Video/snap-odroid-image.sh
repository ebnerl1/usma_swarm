#!/bin/bash

#this script requires that you create a directory called 'odroid_images' in the home folder
#Change 'rrc' to the name of the user 

PICFOLDER='/home/rrc/odroid_images'
DATE=`date '+%Y-%m-%d-%H-%M-%S'`;

curl -s -f -m 5 http://odroid:odroid@192.168.11.$1:8090/?action=snapshot > "$PICFOLDER/waypoint$2-$DATE.jpeg"

if [ -s "$PICFOLDER/waypoint$2-$DATE.jpeg" ]; then
    cp -f "$PICFOLDER/waypoint$2-$DATE.jpeg" /tmp/.camera.jpeg
    convert "/tmp/.camera.jpeg" -gravity northwest -pointsize 16 -undercolor 'rgba(255,155,0,0.6)' -fill black -annotate +5+5 "UAS#$1 Snapshot $DATE Waypoint $2 " "$PICFOLDER/waypoint$2-$DATE.jpeg"
else
    logger -t $0 "unable to save $PICFOLDER/waypoint$2-$DATE.jpeg"
    sudo systemctl restart mjpeg_streamer.service
fi
