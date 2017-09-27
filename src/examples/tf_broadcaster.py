#!/usr/bin/env python

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import QuaternionStamped, TransformStamped

class Node():
    def __init__(self, child_frame):
        self.sub = rospy.Subscriber('orientation', QuaternionStamped,
                                    self.callback)
        self.br = tf2_ros.TransformBroadcaster()
        self.child_frame = child_frame

    def callback(self, orientation):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = self.child_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation = orientation.quaternion

        self.br.sendTransform(t)


if __name__ == "__main__":
    rospy.init_node('example_broadcaster')
    child = rospy.get_param("~child_frame", "base_link")
    node = Node(child)
    rospy.spin()
