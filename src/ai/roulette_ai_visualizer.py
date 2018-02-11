#!/usr/bin/python

import cv2
import RouletteWheel
import rospy
import cv_bridge
from sensor_msgs.msg import Image


class Node:
    def __init__(self, topic):
        self.sub = rospy.Subscriber('camera/{}/undistorted'.format(topic),
                                    Image, self.callback)
        self.bridge = cv_bridge.CvBridge()


    def callback(self, image_msg):
        image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        image = cv2.resize(image, (0, 0), fx=0.5, fy=0.5)
        wheel = RouletteWheel.RouletteWheel(image)

        small_img = cv2.resize(image, (0, 0), fx=0.5, fy=0.5)
        cv2.imshow('Image', small_img)
        if len(wheel.slices):
            detections = cv2.resize(wheel.image, (0, 0), fx=0.5, fy=0.5)
            cv2.imshow('Slices', wheel.visual_slices)
            cv2.imshow('Detections', detections)
        cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('roulette_visualizer')
    topic = rospy.get_param('~camera')

    node = Node(topic)
    rospy.spin()
