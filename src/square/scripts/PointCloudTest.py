#!/usr/bin/env python

import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import rospy

def create_fields(msg):

    msg.fields = [PointField(), PointField(), PointField(), PointField()]

    msg.fields[0].name="x"
    msg.fields[0].offset=0
    msg.fields[0].datatype=7
    msg.fields[0].count=1

    msg.fields[1].name = "y"
    msg.fields[1].offset = 0
    msg.fields[1].datatype = 7
    msg.fields[1].count = 1

    msg.fields[2].name = "z"
    msg.fields[2].offset =0
    msg.fields[2].datatype = 7
    msg.fields[2].count = 1

    msg.fields[3].name = "intensity"
    msg.fields[3].offset = 16
    msg.fields[3].datatype = 7
    msg.fields[3].count = 1

def generate(arr1, time_stamp):
    arr=np.array(arr1)
    arr=np.reshape(arr, (-1,4))
    cloud_msg=PointCloud2()
    cloud_msg.header.stamp=time_stamp
    cloud_msg.header.frame_id="frame"
    cloud_msg.height=arr.shape[0]
    cloud_msg.width=arr.shape[1]
    create_fields(cloud_msg)
    cloud_msg.is_bigendian=False
    cloud_msg.point_step = arr.dtype.itemsize
    cloud_msg.row_step = cloud_msg.point_step*arr.shape[1]
    cloud_msg.is_dense = True
    cloud_msg.data = arr1
    print arr
    return cloud_msg

def publisher() :
    '''
    arr=np.zeros(shape=(32,4))
    for i in range (0,31):
        arr[i][0]=1
        arr[i][1]=2*i
        arr[i][2]=0
        arr[i][3]=0.5
    '''
    arr=[1,1,0,1]
   
    rospy.init_node("PC2Publisher", anonymous=True)
    pub=rospy.Publisher('points', PointCloud2, queue_size=10)

    time_stamp=rospy.Time.now()

    cloud=generate(arr, time_stamp)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(cloud)
        rospy.loginfo("Publishing")
        rate.sleep()

if __name__ == '__main__' :
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass




