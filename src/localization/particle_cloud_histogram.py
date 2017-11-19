#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import argparse
import rospy
import rostopic
import std_msgs.msg
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32

class Histogram():
    
    def callback(self, msg):
        
        rospy.loginfo("Plotting Update")
      
        n, bins, patches = plt.hist(msg.channels[0].values, int(200),
                                        histtype='bar')

        plt.ylabel('Weight')
        
        plt.xlabel('Value')
        plt.title('Histogram')

        plt.legend()

        plt.pause(0.05)
        plt.gca().clear()
         

    def __init__(self):

        # Subscribe to the topic
        self.sub = rospy.Subscriber("/localization/particles", PointCloud, self.callback)

        rospy.loginfo("Subscribed to /localization/particles")

        plt.ion()
        plt.show()


    def __del__(self):
        self.sub.unregister()


if __name__ == "__main__":
    global h

    rospy.init_node("Histogram", anonymous=True)

    h = Histogram()

    rospy.spin()