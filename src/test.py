#!/usr/bin/env python3

import rospy
import rosbag
from sensor_msgs import point_cloud2
import pcl_ros
rospy.init_node('test', anonymous=True)

bag = rosbag.Bag('Scene1.bag')

for topic, msg, t in bag.read_messages(topics=['/r200/camera/depth_registered/points']):
    height = msg.height
    print("height", height)

    width = msg.width
    print("weight", width)

    fileds = msg.fields
    print("fields")
    print(fileds)

    data = msg.data
    print("data")
    print(type(data))

bag.close()


rospy.spin()

