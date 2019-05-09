#!/usr/bin/python
# This code takes in serial information from the SF11 Laser Rangefinder.
# It publishes this information to the ROS topic /tactic_interface/altitude.
# The ROS node polls the attitude sensors on the quadcopter to get roll, pitch, and 
# yaw angles in Quaternion.  The quat_converter function then converts this to
# standard angles.  Trigonometric identities are in the algorithm to get the 
# adjusted height.  The adjusted height is published to the topic with a time stamp.


import serial
import rospy
import numpy

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Imu

port = serial.Serial("/dev/rangefinder", baudrate=115200)

reading = 0.00
roll = 0.00
pitch = 0.00
adjusted_height = 0.00

def quat_converter(x,y,z,w):
    
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = numpy.arctan(sinr_cosp/cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = numpy.arcsin(sinp)

    return (roll, pitch)


def callback(data):
    global roll
    global pitch
    x = data.orientation.x
    y = data.orientation.y
    z = data.orientation.z
    w = data.orientation.w
    result = quat_converter(x,y,z,w)
    roll = result[0]
    pitch = result[1]
    #print("roll:"+str(roll)+" pitch:"+str(pitch))


def talker():
    rospy.init_node('altitude', anonymous = True)
    pub = rospy.Publisher('/tactic_interface/altitude', PointStamped , queue_size = 1)
    rospy.Subscriber('/autopilot/imu', Imu, callback)
    msg = PointStamped()
    #rate = rospy.Rate(20) # 20hz
    while not rospy.is_shutdown():
        global adjusted_height
        data_raw = repr(port.readline())
        data_raw = data_raw.split()
        if len(data_raw) > 1:
            data_raw = data_raw[1]
            if len(data_raw) <= 6:
                global reading
                reading = data_raw
                reading = float(reading)
                adjusted_height = reading * numpy.cos(roll) * numpy.cos(pitch)
                


        msg.header.stamp = rospy.Time.now()
        msg.point.x = adjusted_height

        pub.publish(msg)
        #rate.sleep()
        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        port.close()
        pass
