#!/bin/bash
gst-launch-1.0 v4l2src ! video/x-raw,width=640,height=480 ! jpegenc ! rtpjpegpay ! udpsink host=192.168.11.203 port=5000
