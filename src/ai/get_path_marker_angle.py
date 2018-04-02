#!/usr/bin/python2
import numpy as np
import math
import cv2
from robosub.srv import get_path_angle
from robosub_msgs.msg import DetectionArray
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
import rospy
from util import *

class MarkerTask():

    def __init__(self):
        self.s = rospy.Service('path_angle', get_path_angle,
                                self.getPathMarkerAngle)

        ts = ApproximateTimeSynchronizer(
                        [Subscriber("/camera/right/undistorted",
                        Image),
                        Subscriber("/vision/right", DetectionArray)], 10, .380)

        ts.registerCallback(self.callback)
        self.angle = 0
        self.vision_label = 'path_marker'
    def getPathMarkerAngle(self, vision_label):
        return self.angle



    def callback(self, image, detectionArray):

        detections = filterByLabel(detectionArray.detections,
                                   self.vision_label)
        vision_result = getMostProbable(detections, thresh=0.5)
        if vision_result == None:
            return

        br = CvBridge()
        img = br.imgmsg_to_cv2(image, desired_encoding="bgr8")
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        box_x = vision_result.x * 1384
        box_y = vision_result.y * 1032
        box_width = vision_result.width * 1384
        box_height = vision_result.height * 1032
        left = int(box_x - (box_width / 2))
        right = int(box_x + (box_width / 2))
        top = int(box_y - (box_height / 2))
        bottom = int(box_y + (box_height / 2))

        box = ([left, top], [right, bottom])

        height, width = img.shape[:2]

        lower_red = np.array([0,100,100])
        upper_red = np.array([20,255,255])

        mask = cv2.inRange(hsv, lower_red, upper_red)
        output = cv2.bitwise_and(hsv, hsv, mask = mask)

        box_height = box[1][1] - box[0][1]

        p1_start = (int(box[1][0]),
                    int(((0.33333*box_height)+box[0][1])))
        p2_start = (int(p1_start[0]),
                    int(((0.66667*box_height)+box[0][1])))

        x = p1_start[0]
        y = p1_start[1]


        pixel = output[y, x]
        black = np.zeros((1, 3), dtype=np.int)
        while np.array_equal(pixel, black[0]):
            output[y,x] = np.ndarray((1, 3), buffer=np.array([255, 0, 0]),
                                             dtype=np.int)
            x -= 1
            pixel = output[y, x]
        print (x,y)
        p1 = (x, y)

        x = p2_start[0]
        y = p2_start[1]


        pixel = output[y, x]
        while np.array_equal(pixel, black[0]):
            output[y,x] = np.ndarray((1, 3), buffer=np.array([255, 0, 0]),
                                             dtype=np.int)
            x -= 1
            pixel = output[y, x]
        print(x, y)
        print "-------------"
        p2 = (x, y)
        dx = abs(p1[0] - p2[0])
        dy = abs(p1[1] - p2[1])

        #if p1[0] == p2[0]:
        #    self.angle=0.0
        #    return

        angle = 90 - math.degrees(math.atan2(dy, dx))

        if p1[0] - p2[0] > 0:
            angle *= -1.0

        #dxdy = float(dx)/dy
        #dydx = 1.0/dxdy

        #while dxdy > 1.0:
        #    dxdy = abs(dxdy - 2.0)
        #while dydx > 1.0:
        #    dydx = abs(dydx - 2.0)
        #if p1[0] < p2[0]:
        #    angle = -1.0 * math.degrees(math.atan(dxdy))
        #else:
        #    angle = 90.0 - math.degrees(math.atan(dydx))

        #angle *= -1.0

        self.angle = angle
	print('angle =', angle)

if __name__ == "__main__":
    rospy.init_node('path_angle')
    node = MarkerTask()
    print "Hello"
    rospy.spin()
