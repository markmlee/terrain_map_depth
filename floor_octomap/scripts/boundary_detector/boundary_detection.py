#! /usr/bin/env python

import rospy
from floor_octomap.msg import BoundingBox

import rospy

class BoundaryDetection:
    def __init__(self):
        self.current_boundaries = []

    def process(image, camera_info):
        pass