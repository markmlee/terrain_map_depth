#!/usr/bin/env python

import roslib
import rospy
import sys
import argparse
import tf
import tf2_ros
import numpy
import geometry_msgs.msg

from tf.transformations import *

'''
Credits:
https://github.com/andreasBihlmaier/ahbros/blob/master/scripts/tf_alias_node.py
'''

def main(args):
  parser = argparse.ArgumentParser()
  parser.add_argument('-f', '--freq', type=float, default=10, help='Frequency TFs are republished (default: 10 Hz)')
  parser.add_argument('target_from_tf', help='Published Base Frame')
  parser.add_argument('target_to_tf', help='Published Target Frame')
  parser.add_argument('source_from_tf', help='Read Base Frame')
  parser.add_argument('source_to_tf', help='Read Target Frame')
  args = parser.parse_args(rospy.myargv()[1:])
  target_from_tf, target_to_tf, source_from_tf, source_to_tf = args.target_from_tf, args.target_to_tf, args.source_from_tf, args.source_to_tf

  rospy.init_node('tf_alias_node', anonymous=True)
  tf_listener = tf.TransformListener()
  tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
  rospy.sleep(rospy.Duration(.5))

  position, quaternion = tf_listener.lookupTransform(source_from_tf, source_to_tf, rospy.Time(0))

  static_transformStamped = geometry_msgs.msg.TransformStamped()

  static_transformStamped.header.stamp = rospy.Time.now()
  static_transformStamped.header.frame_id = target_from_tf
  static_transformStamped.child_frame_id = target_to_tf

  static_transformStamped.transform.translation.x = position[0]
  static_transformStamped.transform.translation.y = position[1]
  static_transformStamped.transform.translation.z = position[2]
  static_transformStamped.transform.rotation.x = quaternion[0]
  static_transformStamped.transform.rotation.y = quaternion[1]
  static_transformStamped.transform.rotation.z = quaternion[2]
  static_transformStamped.transform.rotation.w = quaternion[3]

  tf_broadcaster.sendTransform(static_transformStamped)

  rospy.spin()


if __name__ == '__main__':
  main(sys.argv)