#!/usr/bin/env python
import serial
import time
ser=serial.Serial("/dev/ttyUSB0",115200)
import rospy
from std_msgs.msg import Float32

def callback(data):
    	#rospy.loginfo(rospy.get_caller_id() , data.data)
    	
    	
	if ser.isOpen():
		print(ser.name+'is open...')

	command_theta = data.data
	
	#turn_time = 50 * 0.016465
	turn_time = command_theta * 0.9585

	if command_theta > 0:
		ser.write(b'@1sB20\r')#regenerative braking at 15%
		ser.write(b'@0sB0\r')
		ser.write(b'@1sv5\r')
		ser.write(b'@0sv0\r')
		#print("RIGHT " + str(command_theta))
		
		time.sleep(abs(turn_time))

		print("RIGHT" + str(command_theta))		
		ser.write(b'@1sB20\r')#regenerative braking at 15%
		ser.write(b'@0sB20\r')
		
		ser.write(b'@1sv0\r')
		ser.write(b'@0sv0\r')

		#time.sleep(10)
		
	elif command_theta < 0:
		ser.write(b'@0sB20\r')#regenerative braking at 15%
		ser.write(b'@1sB0\r')
		ser.write(b'@0sv5\r')
		ser.write(b'@1sv0\r')
		
		#print("LEFT" + str(command_theta))
		time.sleep(abs(turn_time))

		print("LEFT " + str(command_theta))		
		ser.write(b'@0sB20\r')#regenerative braking at 15%
		ser.write(b'@1sB20\r')
		
		ser.write(b'@0sv0\r')
		ser.write(b'@1sv0\r')
		#time.sleep(10)		
		
	elif command_theta == 0:
		ser.write(b'@0sB0\r')#regenerative braking at 0%
		ser.write(b'@1sB0\r')
		ser.write(b'@0sv5\r')
		ser.write(b'@1sv5\r')
		#print("STRAIGHT"+ str(command_theta))

		time.sleep(0.5)
		ser.write(b'@0sB20\r')#regenerative braking at 0%
		ser.write(b'@1sB20\r')
		ser.write(b'@0sv0\r')
		ser.write(b'@1sv0\r')
		print("STRAIGHT"+ str(command_theta))
	else:
		pass	

	command_theta = 0

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("Direction", Float32, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
