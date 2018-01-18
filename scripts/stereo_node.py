#!/usr/bin/env python
import cv2
import math
import message_filters
import numpy as np
import rospy

from robosub.msg import ObstaclePosArray, ObstaclePos
from rs_yolo.msg import DetectionArray


# The distance between the two cameras (in meters).
camera_separation = 0.40

# The focal distance of the camera.
camera_focal_distance = 442

# The dimensions of the images in pixels.
width = 1384
height = 1032


def callback(left_detections, right_detections):
    """ROS callback for left and right detections (when in sync).

    Args:
        left_detections: Detections from the left camera network.
        right_detections: Detections from the right camera network.
    """
    position_msg = ObstaclePosArray()
    for left_detection in left_detections.detections:
        best_index = -1
        best_distance = 500000
        for i in range(0, len(right_detections.detections)):
            right_detection = right_detections.detections[i]
            if right_detection.label == left_detection.label:
                distance = math.sqrt((right_detection.x - left_detection.x)**2 +
                                     (right_detection.y - left_detection.y)**2)
                if distance < best_distance:
                    best_distance = distance
                    best_index = i
        if best_index == -1:
            continue
        right_detection = right_detections.detections[best_index]
        rospy.loginfo('{}'.format(best_index))

        # Calculate the left image and right image (x, y) coordinates in pixels.
        xl = (left_detection.x - 0.5) * width
        yl = -1 * (left_detection.y - 0.5) * height

        xr = (right_detection.x - 0.5) * width
        yr = -1 * (right_detection.y - 0.5) * height

        # Calculate the disparity of the detection.
        disparity = (xl - xr)
        d = camera_separation
        f = camera_focal_distance

        # Finally, convert the left and right (x, y) coordinates into a global
        # (x, y, z) coordinate from the center of the two cameras by following
        # the disparity calculations here:
        # https://users.cs.cf.ac.uk/Dave.Marshall/Vision_lecture/node11.html
        # After calculation, convert them to robosub's coordinate system.
        data = ObstaclePos()
        data.name = left_detection.label
        data.x = d * f / disparity
        data.y = -1 * (d / 2 * (xl + xr) / disparity)
        data.z = d / 2 * (yl + yr) / disparity

        position_msg.data.append(data)

    # Publish the relative position of each detected object.
    pub.publish(position_msg)


if __name__ == '__main__':
    rospy.init_node('visual_depth')

    focal_distance = rospy.get_param('~focal_distance')
    camera_separation = rospy.get_param('~camera_separation')
    width = rospy.get_param('~width', default=1384)
    width = rospy.get_param('~height', default=1032)

    left_sub = message_filters.Subscriber('/vision/left', DetectionArray)
    right_sub = message_filters.Subscriber('/vision/right', DetectionArray)

    # Synchronize messages from the left and right vision networks for stereo
    # detection.
    ts = message_filters.ApproximateTimeSynchronizer([left_sub, right_sub], 10, 0.180)
    ts.registerCallback(callback)

    pub = rospy.Publisher('vision/relative', ObstaclePosArray, queue_size=10)

    rospy.spin()
