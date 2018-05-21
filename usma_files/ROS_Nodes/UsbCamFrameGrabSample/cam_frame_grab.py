#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Int32

# simple class to contain the node's variables and code
class CamFrameGrabNode:
    # Class constructor; advertise intent to publish
    def __init__(self):
        self.header_data = "Empty"
        self.uav_id = 0
        self.wp_id = 0
        self.uav_alt = 0.0
        self.trigger = False
        
        # advertise we'll publish cam_frame topic
        self.cam_frame_pub = rospy.Publisher('cam_frame', Image, queue_size=10)
        
        # subscribe to the following topics
        rospy.Subscriber("at_wp/uav_id", Int32, self.rx_uav_id_callback)
        rospy.Subscriber("at_wp/wp_id", Int32, self.rx_wp_id_callback)
        rospy.Subscriber("at_wp/uav_alt", Float32, self.rx_uav_alt_callback)
        rospy.Subscriber("at_wp/trigger", Bool, self.rx_trigger_callback)
        rospy.Subscriber("usb_cam/image_raw", Image, self.image_raw_callback)

    def rx_uav_id_callback(self, msg):
        self.uav_id = msg.data

    def rx_wp_id_callback(self, msg):
        self.wp_id = msg.data
        
    def rx_uav_alt_callback(self, msg):
        self.uav_alt = msg.data
                
    def rx_trigger_callback(self, msg):
        self.trigger = msg.data

    def image_raw_callback(self, data):
        if self.trigger == True:
            data.header.frame_id = "UAV_{}__WP_{}__ALT_{}".format(self.uav_id, self.wp_id, self.uav_alt)
            self.cam_frame_pub.publish(data)
            self.trigger = False

if __name__ == '__main__':
    # initialize the ROS client API, giving the default node name
    rospy.init_node("cam_frame_grab_node")
    
    node = CamFrameGrabNode()
    
    # enter the ROS main loop
    rospy.spin()
