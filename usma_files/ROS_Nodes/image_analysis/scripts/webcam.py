#!/usr/bin/env python
import cv2
import rospy
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# OpenCV webcam object
camera = cv2.VideoCapture(0)

# Object that converts from OpenCV images to ros images
bridge = CvBridge()


def sendVideoFeed():
    pub = rospy.Publisher('camera/rgb/image_raw', Image, queue_size = 10)   
    rospy.init_node('webcam', anonymous=True)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        receivedImage, frame = camera.read()
   		
        if (not receivedImage):
            continue

        # get image dimensions for logging
        height, width, channels = frame.shape
        rospy.loginfo("Sending image: " + str(height) + " " + str(width))

        # convert OpenCV image to ROS image and publish message
        img = bridge.cv2_to_imgmsg(frame, "bgr8")
        pub.publish(img)

        rate.sleep()

if __name__ == '__main__':
    try:
        sendVideoFeed()
    except rospy.ROSInterruptException:
        pass

camera.release()
cv2.destroyAllWindows()
