Server (on the zephyrs):
sudo apt-get install vlc (only do this once)
cvlc v4l2:///dev/video0:chroma=mjpg:width=800:height=600 --sout '#rtp{sdp=rtsp://192.168.11.XXX:4444/live.sdp}'

Client (the basestation):
In a browser like Firefox...
rtsp://192.168.11.XXX:4444/live.sdp
