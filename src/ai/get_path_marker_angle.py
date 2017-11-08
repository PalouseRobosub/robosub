#!/usr/bin/python2
import numpy as np
import math
import cv2
from rs_yolo.msg import DetectionArray
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import rospy

class getPathMarkerAngle:

    def getPathMarkerAngle(vision_label):

        self.vision_label = vision_label

        ts = ApproximateTimeSynchronizer(Subscriber("/camera/image/raw_image",
                                         sensor_msgs/Image),
                                         Subscriber("/vision", DetectionArray),
                                         0.33)
        ts.registerCallback(callback)

    def callback(image, detectionArray):

        detections = filterByLabel(detectionArray.detections,
                                   self.vision_label)
        vision_result = getMostProbable(detections, thresh=0.5)

        orange = ([14, 25, 180], [35, 45, 255])

        #br = CvBridge()
        #dtype, n_channels = br.encoding_as_cvtype2('8UC3')
        im = np.ndarray(shape=(1384, 1032, n_channels), dtype=dtype)
        img = image

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

        lower = np.array(orange[0], dtype = "uint8")
        upper = np.array(orange[1], dtype = "uint8")

        mask = cv2.inRange(img, lower, upper)
        output = cv2.bitwise_and(img, img, mask = mask)

        box_height = box[1][1] - box[0][1]
        p1_start = (int(box[1][0]*width),
                    int(((0.33333*box_height)+box[0][1])*height))
        p2_start = (int(p1_start[0]),
                    int(((0.66667*box_height)+box[0][1])*height))

        x = p1_start[0]
        y = p1_start[1]
        pixel = output[y, x]
        black = np.zeros((1, 3), dtype=np.int)
        while np.array_equal(pixel, black[0]):
            output[y,x] = np.ndarray((1, 3), buffer=np.array([255, 0, 0]),
                                             dtype=np.int)
            x -= 1
            pixel = output[y, x]
        p1 = (x, y)

        x = p2_start[0]
        y = p2_start[1]
        pixel = output[y, x]
        while np.array_equal(pixel, black[0]):
            output[y,x] = np.ndarray((1, 3), buffer=np.array([255, 0, 0]),
                                             dtype=np.int)
            x -= 1
            pixel = output[y, x]
        p2 = (x, y)

        dx = abs(p1[0] - p2[0])
        dy = abs(p1[1] - p2[1])

        dxdy = float(dx)/dy
        dydx = 1.0/dxdy

        while dxdy > 1.0:
            dxdy = abs(dxdy - 2.0)
        while dydx > 1.0:
            dydx = abs(dydx - 2.0)
        if p1[0] < p2[0]:
            angle = -1.0 * math.degrees(math.atan(dxdy))
        else:
            angle = 90.0 - math.degrees(math.atan(dydx))

        angle *= -1.0

        return angle

def path_angle_server():
    rospy.init_node('path_angle')
    s = rospy.Service('path_angle', get_path_angle, getPathMarkerAngle)
    print "Ready to calculate angle"
    rospy.spin()

if __name__ == "__main__":
    path_angle_server()
