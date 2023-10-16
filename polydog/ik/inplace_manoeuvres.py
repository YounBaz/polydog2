#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
import time

rospy.init_node('manoeuvres')
pub = rospy.Publisher('coordinates', Float32MultiArray, queue_size=10)
rate = rospy.Rate(1)  # Adjust the rate as needed
ym_values = list(range(0, -95, -5))

for ym in ym_values:
    coordinates = Float32MultiArray(data=[0, ym, 0])
    pub.publish(coordinates)
    time.sleep(1) 
