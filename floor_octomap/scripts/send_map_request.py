#!/usr/bin/env python

import rospy
from floor_octomap.msg import StampedString

pub = rospy.Publisher('/map_request', StampedString, queue_size=10)
rospy.init_node('send_map_request', anonymous=True)
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    if (raw_input("Send? ") == "q"):
        break
    req_msg = StampedString()
    req_msg.header.stamp = rospy.Time.now()
    pub.publish(req_msg)
    rate.sleep()
