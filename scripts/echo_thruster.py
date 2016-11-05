#!/usr/bin/env python

import rospy
from robosub.msg import thruster

def print_pretty(name, value):
    print "{:17s}: {:5}".format(name, value)

def callback(msg):
    global names
    for i in range(0,len(msg.data)):
        print_pretty(names[i]['name'], msg.data[i])
    print "---"

if __name__ == "__main__":
    global names

    names = rospy.get_param('thrusters')

    sub = rospy.Subscriber('thruster', thruster, callback=callback, queue_size=1)
    rospy.init_node('thruster_echo', anonymous=True)

    rospy.spin()
