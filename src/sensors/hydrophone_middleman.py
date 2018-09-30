#!/usr/bin/python

import math

import rospy

from robosub_msgs.msg import Float32Stamped


from geometry_msgs.msg import Vector3Stamped


# TODO comeup with a better name
class MiddleNode:
    """
        Republishes only accurate pinger bearings to a separate topic

    """

    def __init__(self):
        self.repub = rospy.Publisher('hydrophones/accurate_bearing',
                                     Vector3Stamped,
                                     queue_size=10)

        self.magnitude_pub = rospy.Publisher('hydrophones/bearing/magnitude',
                                             Float32Stamped,
                                             queue_size=10)

        self.sub = rospy.Subscriber('hydrophones/bearing', Vector3Stamped, self.bearing_callback)

        # TODO make these ros parameters
        self.lower_bound = 0.8
        self.upper_bound = 1.20

    def bearing_callback(self, msg):
        
        magnitude = self._get_magnitude(msg.vector)

        mag_msg = Float32Stamped()
        mag_msg.header.stamp = rospy.Time.now()
        mag_msg.data = magnitude

        self.magnitude_pub.publish(mag_msg)

        if magnitude > self.lower_bound and magnitude < self.upper_bound:
            self.repub.publish(msg)

    def _get_magnitude(self, vector):
        return math.sqrt(vector.x**2 + vector.y**2 + vector.z**2)

if __name__ == "__main__":
    rospy.init_node('hydrophone_middleman')

    node = MiddleNode()
    rospy.spin()

