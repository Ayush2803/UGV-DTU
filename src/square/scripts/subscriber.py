#!/usr/bin/env python
#license removed for brevity

import rospy
from std_msgs.msg import Float32

def callback(data):
	x=float(data.data)
	x=x*x
	s=str(x)
	rospy.loginfo(s)

def subscriber():
	rospy.init_node('subscriber', anonymous=True)
	rospy.Subscriber('chatter', Float32, callback)
	rospy.spin()

if __name__=='__main__':
	subscriber()

