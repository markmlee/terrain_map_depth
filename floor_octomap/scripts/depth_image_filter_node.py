#!/usr/bin/env python
"""
Purpose of the file: subscribe to a topic called /image_raw of type sensor_msgs/Image
Apply filter to the resulting image
"""
from __future__ import print_function
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

class SubThenFilter:
    def __init__(self, sub_topic, pub_topic, frame_id):
        self.sub = rospy.Subscriber(sub_topic, Image, self.image_callback, queue_size=1)
        self.pub = rospy.Publisher(pub_topic, Image, queue_size=1)
        self.bridge = CvBridge()

        self.frame_id = frame_id
        
        self.use_median_blur = True
        self.median_blur_size = 5

        self.use_inpaiting = False
        self.inpaiting_iterations = 5

        self.manual_threshold = False
        self.filtering_iterations = 3

        self.max_threshold_uint16 = 2000 # 3m ?

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"passthrough")
        except CvBridgeError as e:
            print(e)

        cv_image = np.nan_to_num(cv_image)

        min_val, max_val, _, _ = cv2.minMaxLoc(cv_image)
        # print("===== In\n{}\n=====".format((min_val, max_val)))

        uint16_to_uint8_scaling = 255.0/max_val
        cv_image = cv2.convertScaleAbs(cv_image, alpha=uint16_to_uint8_scaling)
        max_threshold_uint8 = self.max_threshold_uint16 * uint16_to_uint8_scaling

        initial_mask = cv_image > max_threshold_uint8

        if self.use_median_blur:
            # cv_image = cv2.medianBlur(cv_image, self.median_blur_size)
            for filter_size in [5, 33, 81]:#[3, 5, 13, 25, 51, 101]: # Gradually incresae filter size
                mask = cv_image > max_threshold_uint8
                if not mask.any(): # Break early if all hole were covered
                    break
                indices = np.where(mask)
                cv_image_2 = cv2.medianBlur(cv_image, filter_size) # We don't want the average as this would capture values that are too far away
                # cv_image_2 = cv2.blur(cv_image, (filter_size, filter_size))
                # cv_image_2 = cv2.bilateralFilter(cv_image, filter_size, 75, 75)
                cv_image[indices] = cv_image_2[indices]
            cv_image = cv2.medianBlur(cv_image, 5)
            # cv_image = cv2.bilateralFilter(cv_image, 2, 75, 75)
            # cv_image = cv2.blur(cv_image, (5, 5))


        if self.use_inpaiting:
            # Look at this
            # https://answers.opencv.org/question/86569/inpainting-normal-behavior-or-a-bug/
            # or this
            # http://www.morethantechnical.com/2011/03/05/neat-opencv-smoothing-trick-when-kineacking-kinect-hacking-w-code/
            # iterations = 0
            # while True and iterations < self.inpaiting_iterations:
            #     mask = cv_image > max_threshold_uint8
            #     if not mask.any():
            #         break
            #     mask = mask.astype(np.uint8)
            #     cv_image = cv2.inpaint(cv_image , mask, 5, cv2.INPAINT_NS)
            #     iterations += 1
            # print("Inpaiting iterations: {}".format(iterations))
            cv_image_2 = cv2.inpaint(cv_image , initial_mask.astype(np.uint8), 5, cv2.INPAINT_TELEA)
            # cv_image = cv2.medianBlur(cv_image, 5)
            indices = np.where(initial_mask)

        if self.manual_threshold:
            # Slow
            mask = cv_image > max_threshold_uint8
            indices = np.where(mask)

            for x, y in zip(indices[0], indices[1]):
                min_ = max_threshold_uint8 + 1
                filter_size = 2
                iterations = 0
                while min_ > max_threshold_uint8 and iterations < self.filtering_iterations:
                    for x_a in range(x - filter_size, x + filter_size):
                        for y_a in range(y - filter_size, y + filter_size):
                            min_ = np.min(cv_image[x_a][y_a])
                    filter_size *= 2
                    iterations += 1
                # print(min_)
                cv_image[x][y] = min_


        # if self.use_adaptive_threshold:
        #     cv_image = cv2.adaptiveThreshold(cv_image, max_threshold_uint8, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2) # 3 = Meters

        # cv2.imshow('image', cv_image)
        # cv2.waitKey(1)

        # min_val, max_val, _, _ = cv2.minMaxLoc(cv_image)
        # print("===== uint8\n{}\n=====".format((min_val, max_val)))

        cv_image = cv_image.astype(np.float64)
        cv_image /= uint16_to_uint8_scaling
        cv_image = cv_image.astype(np.uint16)

        # min_val, max_val, _, _ = cv2.minMaxLoc(cv_image)
        # print("===== Out\n{}\n=====".format((min_val, max_val)))

        try:
            msg = self.bridge.cv2_to_imgmsg(cv_image, "passthrough")
            data.data = msg.data
            data.header.frame_id = self.frame_id
            self.pub.publish(data)
        except CvBridgeError as e:
            print(e)

if __name__ == "__main__":
    if len(sys.argv) == 6:
        sub_topic = str(sys.argv[1])
        pub_topic = str(sys.argv[2])
        frame_id = str(sys.argv[3])

        rospy.init_node("filter_depth_server")
        rospy.loginfo("Starting filter_depth_server. Subscribed from {}, Publish to {}".format(sub_topic, pub_topic))
        sf = SubThenFilter(sub_topic, pub_topic, frame_id)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("shutting down")
    else:
        rospy.loginfo("Need two inputs: pointcloud subscribe topic and publish topic")
        sys.exit(1)

cv2.destroyAllWindows()