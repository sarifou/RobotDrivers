#!/usr/bin/env python

#======= Import ================
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Range
from sensor_msgs.msg import BatteryState

lines = []
ultrasound = Range()
battery = BatteryState

def range(value):
    ultrasound.header.stamp = rospy.Time.now()
    ultrasound.header.frame_id = "/ultrasound"
    ultrasound.radiation_type = 0
    ultrasound.field_of_view = 0.1
    ultrasound.min_range = 0
    ultrasound.max_range = 2
    ultrasound.range = value
    ultrasound_pub.publish(ultrasound)

# --- Subscriber 
def callback(data):
    rospy.loginfo("I heard %s",data.data)
    range(data.data[0])
    lines.append(data.data[1])
    lines.append(data.data[2])
    lines.append(data.data[3])
    #battery = data.data[4]
    

    
def listener():
    rospy.init_node('serial_bridge')
    rospy.Subscriber("tquad/serial_publisher", Float64MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        ultrasound_pub = rospy.Publisher('tquad/ultrasound', Range, queue_size=2)
        battery_pub = rospy.Publisher('tquad/battery', BatteryState, queue_size=2)
        listener()
    except rospy.ROSInterruptException:
        pass