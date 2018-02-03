#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# Instantiate CvBridge
bridge = CvBridge()

# simple class to contain the node's variables and code
class UsbcamReceiverNode:
    # Class constructor; advertise intent to publish
    def __init__(self):
        # advertise we'll publish cam_frame topic
        #self.cam_frame_pub = rospy.Publisher('usb_rx/cam_frame', Image, queue_size=10)
        
        # subscribe to the following topics
        rospy.Subscriber("cam_frame", Image, self.image_callback)


    def image_callback(self, msg):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        #self.cam_frame_pub.publish(data)
        print("Received image: %s"%(msg.header.frame_id))
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg 
            #cv2.imwrite('camera_image.jpeg', cv2_img)
            cv2.imwrite('IMG__%s.jpeg'%(msg.header.frame_id), cv2_img)
   
if __name__ == '__main__':
    # initialize the ROS client API, giving the default node name
    rospy.init_node("usbcam_receiver_node")
    
    node = UsbcamReceiverNode()
    
    # enter the ROS main loop
    rospy.spin()
