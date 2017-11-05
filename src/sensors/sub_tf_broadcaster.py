#!/usr/bin/env python

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import QuaternionStamped, TransformStamped, \
                                Quaternion, PointStamped, Point
from robosub.msg import Float32Stamped

class Node():
    def __init__(self, child_frame):
        self.sub = rospy.Subscriber('orientation', QuaternionStamped,
                                    self.orientation_callback)
        self.sub = rospy.Subscriber('depth', Float32Stamped,
                                    self.depth_callback)
        self.sub = rospy.Subscriber("position/point", PointStamped,
                                    self.location_callback)
        self.br = tf2_ros.TransformBroadcaster()
        self.child_frame = child_frame
        self.depth = 0.0
        self.point = Point()
        self.quaternion = Quaternion(0, 0, 0, 1)
        self.update_tf_tree()

    def depth_callback(self, msg):
        self.depth = msg.data
        self.update_tf_tree()

    def orientation_callback(self, orientation):
        self.quaternion = orientation.quaternion
        self.update_tf_tree()

    def location_callback(self, point):
        self.point = point.point
        self.update_tf_tree()



    def update_tf_tree(self):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = self.child_frame
        t.transform.translation.x = self.point.x
        t.transform.translation.y = self.point.y
        t.transform.translation.z = self.depth
        t.transform.rotation = self.quaternion

        self.br.sendTransform(t)


if __name__ == "__main__":
    rospy.init_node('sub_tf_broadcaster')
    child = rospy.get_param("~child_frame", "base_link")
    node = Node(child)
    rospy.spin()
