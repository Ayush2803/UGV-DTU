#!/usr/bin/env python


import rospy
from std_msgs.msg import Int32
def publisher():
	pub=rospy.Publisher('chatter',Int32, queue_size=10)
	rospy.init_node('publisher', anonymous=True)
	rate= rospy.Rate(10)
	x=4
	while not rospy.is_shutdown():
		pub.publish(x)
		s=str(x)
		rospy.loginfo(s)
		rate.sleep()

if __name__ == '__main__' :
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass	
	
