#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Int32

# simple class to contain the node's variables and code
class AtWpTriggerNode:
    # Class constructor; advertise intent to publish
    def __init__(self):
        self.time_period = 5
        self.uav_id = 115
        self.wp_id = 64
        self.uav_alt = 22.3
        
        # advertise that we'll publish on at_wp topic
        self.trigger_pub = rospy.Publisher("at_wp/trigger", Bool, queue_size=10)
        self.uav_id_pub = rospy.Publisher("at_wp/uav_id", Int32, queue_size=10)
        self.wp_id_pub = rospy.Publisher("at_wp/wp_id", Int32, queue_size=10)
        self.uav_alt_pub = rospy.Publisher("at_wp/uav_alt", Float32, queue_size=10)
        
        # create the timer with period time_period
        rospy.Timer(rospy.Duration(self.time_period), self.timer_callback)
    
    # Callback function for the timer event
    def timer_callback(self, event):
        at_wp_trigger = True
        self.trigger_pub.publish(at_wp_trigger) 
        self.uav_id_pub.publish(self.uav_id) 
        
        wp_id = Int32()
        wp_id.data = self.wp_id
        self.wp_id_pub.publish(wp_id) 
        
        uav_alt = Float32()
        uav_alt.data = self.uav_alt
        self.uav_alt_pub.publish(uav_alt) 

if __name__ == '__main__':
    # initialize the ROS client API, giving the default node name
    rospy.init_node("at_wp_trigger_node")
    
    node = AtWpTriggerNode()
    
    # enter the ROS main loop
    rospy.spin()


