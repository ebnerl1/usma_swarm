#!/usr/bin/python
import serial
import rospy

from geometry_msgs.msg import PointStamped

port = serial.Serial("/dev/rangefinder", baudrate=115200)

reading = 0.00

def talker():
    rospy.init_node('altitude', anonymous = True)
    pub = rospy.Publisher('/tactic_interface/altitude', PointStamped , queue_size = 1)
    msg = PointStamped()
    #rate = rospy.Rate(20) # 20hz
    while not rospy.is_shutdown():
        data_raw = repr(port.readline())
        data_raw = data_raw.split()
        if len(data_raw) > 1:
            data_raw = data_raw[1]
            if len(data_raw) <= 5:
                global reading
                reading = data_raw
                reading = float(reading)
                


        msg.header.stamp = rospy.Time.now()
        msg.point.x = reading
        pub.publish(msg)
        #rate.sleep()
        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        port.close()
        pass
       
