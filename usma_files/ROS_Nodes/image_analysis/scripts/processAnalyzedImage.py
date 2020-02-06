#!/usr/bin/env python

import cv2
import rospy

from cv_bridge import CvBridge

from swarms.msg import ObjectCount
from swarms.msg import BoundingBoxes
from sensor_msgs.msg import Image

bridge = CvBridge()

def imageCallback(data):
    frame = bridge.imgmsg_to_cv2(data, "bgr8")
    cv2.circle(frame, (50, 50), 10, 255)
    cv2.imshow('frame', frame)  
    cv2.waitKey(1)

def boundingBoxesCallback(data):
    for boundingBox in data.bounding_boxes:
        print boundingBox.Class, boundingBox.probability


def listener():
    rospy.init_node('processAnalyzedImage', anonymous = True)
    rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, boundingBoxesCallback)
    rospy.Subscriber('/darknet_ros/detection_image', Image, imageCallback)
	
    rospy.spin()

if __name__ == '__main__':
    listener()

