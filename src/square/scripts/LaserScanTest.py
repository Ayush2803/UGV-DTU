#!/usr/bin/env python
import numpy as np
import cv2
import math
import rospy
from sensor_msgs.msg import LaserScan

def generate(scan_rate, angle_min, angle_max, ranges, intensities):

    scan=LaserScan()
    start_time = rospy.Time.now()
    angle_increment=(angle_max-angle_min)/400
    time_increment=1/scan_rate

    scan.header.stamp = start_time;
    scan.header.frame_id = 'scanner'
    scan.angle_min=angle_min
    scan.angle_max=angle_max
    scan.angle_increment=angle_increment
    scan.time_increment=time_increment
    scan.range_min=0
    scan.range_max=10
    scan.ranges=ranges
    scan.intensities=intensities

    return scan

    

def publisher():

    rospy.init_node('LaserScan', anonymous='True')
    pub = rospy.Publisher('/scan', LaserScan, queue_size=10)

    img=np.zeros((400,400), np.uint8)

    for i in range(1,400):
        img[i][100]=255

    c=0

    while not rospy.is_shutdown():
        cv2.imshow("IMG", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        angle_min=180
        angle_max=-180
        ranges=[]
        intensities=[]

        scan_rate = 50
        rate=rospy.Rate(scan_rate)

        for i in range(1,400):
            x=100
            y=i
            if img[y][x]==255:
                d=(y+c)**2 + (x-200)**2
                d=math.sqrt(d)
                ranges.append(d)
		d=d/200
		intensities.append(0.5)
                sin=(x-200)/(d*200)
                theta=math.asin(sin)
                if theta>angle_max:
                    angle_max=theta
                if theta<angle_min:
                    angle_min=theta

        scan=generate(scan_rate, angle_min, angle_max, ranges, intensities)

        pub.publish(scan)
	rospy.loginfo("Publishing")
	rate.sleep()



if __name__ == '__main__' :
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
