#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2

def callback(data):
	print(data.data[20])
	#print(type(data.data))
	#print(data.data[0])

if __name__ == "__main__":
	rospy.init_node("chad_test3", anonymous=True)

	subs = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, callback, queue_size = 1)

	rospy.spin()