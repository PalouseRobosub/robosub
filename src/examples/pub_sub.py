#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

class Node():
    def __init__(self):
        self.sub = rospy.Subscriber('degrees', Float32, self.callback)
        self.pub = rospy.Publisher('radians', Float32, queue_size=1)

    def callback(self, degrees):
        radians = degrees.data * 3.14/180
        self.pub.publish(Float32(radians))

if __name__ == "__main__":
    rospy.init_node('degree_converter')
    node = Node()
    rospy.spin()
