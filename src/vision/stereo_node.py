#!/usr/bin/env python
import cv2
import math
import message_filters
import numpy as np
import rospy
import tf
import tf2_ros

from robosub.msg import ObstaclePosArray, ObstaclePos
from rs_yolo.msg import DetectionArray

from geometry_msgs.msg import TransformStamped, QuaternionStamped, Quaternion, \
                              Point, PointStamped


class Pair:
    """ Defines a possible stereo pair of detections.

    Attributes:
        left_index: The left index of the detection in a list.
        left_detection: The left image detection.
        right_index: The right index of the detection in a list.
        right_detection: The right image detection.
        distance: The calculated distance (in pixels) of the two detections.
    """

    def __init__(self, l_index, l_detection, r_index, r_detection):
        """ Initializes the stereo image pair. """
        self.left_index = l_index
        self.right_index = r_index
        self.left_detection = l_detection
        self.right_detection = r_detection

        self.distance = math.sqrt(
                (self.right_detection.x - self.left_detection.x)**2 +
                (self.right_detection.y - self.left_detection.y)**2)


def detection_pairs(left_detections, right_detections):
    """ Calculates the likely detection pairs in two lists of detections. """
    detected_labels = list(set().union(
            [detection.label for detection in left_detections],
            [detection.label for detection in right_detections]))

    pairs = []
    for label in detected_labels:
        # Get all elements from each list with the respective label.
        left = [x for x in left_detections if x.label == label]
        right = [x for x in right_detections if x.label == label]

        while len(left) >= 1 and len(right) >= 1:
            # Enumerate all possible pairings of left and right detections.
            possible_pairs = []
            for l_index, l_detection in enumerate(left):
                for r_index, r_detection in enumerate(right):
                    possible_pairs.append(Pair(l_index, l_detection,
                                               r_index, r_detection))

            # Find the most likely pair of objects from the list
            best_pair = possible_pairs[0]
            for pair in possible_pairs:
                if pair.distance < best_pair.distance:
                    best_pair = pair


            # Add the most likely pair to detected pairs and repeat until no
            # more pairs can be created.
            pairs.append((best_pair.left_detection, best_pair.right_detection))
            left.pop(best_pair.left_index)
            right.pop(best_pair.right_index)

    return pairs


class Node:
    """ Defines a stereo vision ROS node.

    Attributes:
        focal_distance: Double representing the camera focal distance in meters.
        camera_separation: Double representing meters between stereo cameras.
        size: Tuple representing image size.
        ts: Approximate time synchronizer object to sync image messages.
        pub: ROS publisher of relative position messages.
    """

    def __init__(self, focal_distance, camera_separation, size):
        """ Initializes a stereo detection node.

        Args:
            focal_distance: A double representing the camera focal distance in
                meters.
            camera_separation: A double representing the distance between the
                two cameras in meters.
            size: A tuple representing the (width, height) of images.


        """
        self.focal_distance = focal_distance
        self.camera_separation = camera_separation
        self.size = size

        left_sub = message_filters.Subscriber('vision/left', DetectionArray)
        right_sub = message_filters.Subscriber('vision/right', DetectionArray)

        # Synchronize messages from the left and right vision networks for
        # stereo detection.
        self.ts = message_filters.ApproximateTimeSynchronizer(
                [left_sub, right_sub], 10, 0.600)
        self.ts.registerCallback(self.detection_callback)

        self.pub = rospy.Publisher('vision/relative',
                ObstaclePosArray, queue_size=10)


    def detection_callback(self, left_detections, right_detections):
        """ROS callback for left and right detections (when in sync).

        Args:
            left_detections: Detections from the left camera network.
            right_detections: Detections from the right camera network.
        """
        position_msg = ObstaclePosArray()

        detected_pairs = detection_pairs(left_detections.detections,
                                         right_detections.detections)
        for left_detection, right_detection in detected_pairs:

            # Calculate the left image and right image (x, y) coordinates in
            # pixels.
            xl = (left_detection.x - 0.5) * self.size[0]
            yl = -1 * (left_detection.y - 0.5) * self.size[1]

            xr = (right_detection.x - 0.5) * self.size[0]
            yr = -1 * (right_detection.y - 0.5) * self.size[1]

            # Calculate the disparity of the detection.
            disparity = (xl - xr)
            d = self.camera_separation
            f = self.focal_distance

            # Finally, convert the left and right (x, y) coordinates into a
            # global (x, y, z) coordinate from the center of the two cameras by
            # following the disparity calculations here:
            # https://users.cs.cf.ac.uk/Dave.Marshall/Vision_lecture/node11.html
            # After calculation, convert them to robosub's coordinate system.
            data = ObstaclePos()
            data.name = left_detection.label
            data.x = d * f / disparity
            data.y = -1 * (d / 2 * (xl + xr) / disparity)
            data.z = d / 2 * (yl + yr) / disparity

            position_msg.data.append(data)

        # Publish the relative position of each detected object.
        self.pub.publish(position_msg)


if __name__ == '__main__':
    rospy.init_node('stereo_vision')

    # The distance between the two cameras (in meters).
    camera_separation = rospy.get_param('~camera_separation')

    # The focal distance of the camera (in meters).
    focal_distance = rospy.get_param('~focal_distance')

    width = rospy.get_param('~width', default=1384)
    height = rospy.get_param('~height', default=1032)


    # Start the stereo node.
    node = Node(focal_distance, camera_separation, (width, height))

    rospy.spin()
