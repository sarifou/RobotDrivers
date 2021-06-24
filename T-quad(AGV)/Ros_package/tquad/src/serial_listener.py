#!/usr/bin/env python
# coding: utf-8

#======= Import ================
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Range
from sensor_msgs.msg import BatteryState
from msg import LineSensor

lineSensor = LineSensor()
ultrasound = Range()
battery = BatteryState()

def publishRange(value):
    """
        Fonction pour publier la valeur renvoyée par le capteur ultrason
    """
    ultrasound.header.stamp = rospy.Time.now()
    ultrasound.header.frame_id = "/ultrasound"
    ultrasound.radiation_type = 0
    ultrasound.field_of_view = 0.1
    ultrasound.min_range = 0
    ultrasound.max_range = 2
    ultrasound.range = value
    ultrasound_pub.publish(ultrasound)

def publishBatteryVoltage(value):
    """
        Fonction pour publier le voltage et le pourcentage de la batterie
    """
    battery.header.stamp = rospy.Time.now()
    battery.voltage = value / 1000
    battery.percentage = 1
    battery_pub.publish(battery)

def publishLineSensors(left, middle, right):
    """
        Fonction pour publier les valeurs renvoyées par les capteurs de ligne
    """
    lineSensor.left = left
    lineSensor.middle = middle
    lineSensor.right = right
    lines_pub.publish(lineSensor)

def callback(data):
    """
        Fonction callback du subscriber
    """
    rospy.loginfo("I heard %s",data.data)
    publishRange(data.data[0])
    publishBatteryVoltage(data.data[4])
    publishLineSensors(data.data[1], data.data[2], data.data[3])

if __name__ == '__main__':
    try:
        rospy.init_node('serial_split')
        rospy.Subscriber("tquad/serial_publisher", Float64MultiArray, callback)
        ultrasound_pub = rospy.Publisher('tquad/ultrasound', Range, queue_size=2)
        lines_pub = rospy.Publisher('tquad/lines_sensors', LineSensor, queue_size=2)
        battery_pub = rospy.Publisher('tquad/battery_state', BatteryState, queue_size=2)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass