#!/bin/bash
if [[ $1 -eq 0 ]] ; then
    echo ''
    echo 'USAGE:'
    echo './receive_video.sh <<quad number here>'
    echo ''
    echo 'Where the quad number is simply the two or three digit vehicle ID'
    echo 'Also, you need to make sure that your IP address is 192.168.11.203'
    echo ''
    exit 0
fi
vehicle=$((5000+$1))
gst-launch-1.0 udpsrc port=$vehicle ! application/x-rtp,encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegdec ! autovideosink
