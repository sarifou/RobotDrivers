#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray


def motors():
    pub = rospy.Publisher('tquad/serial_subscriber', Int16MultiArray, queue_size=1)
    rospy.init_node('tquad_driver', anonymous=True)
    rate = rospy.Rate(1) # 1hz

    vel = Int16MultiArray()
    vel.data = []

    while not rospy.is_shutdown():
        vel.data = [250,250,250,250]
        rospy.loginfo(vel)
        pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        motors()
    except rospy.ROSInterruptException:
        pass