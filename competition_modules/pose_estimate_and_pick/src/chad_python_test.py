#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image



def callback(data):
	
	print(type(data))
	data.encoding = "1"
	print(data.encoding) 

if __name__ == "__main__":

	rospy.init_node('chad_python_test',anonymous=False)
	subs = rospy.Subscriber("/Object_detection/mask", Image, callback)
	rospy.spin()