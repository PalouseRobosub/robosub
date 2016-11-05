#!/usr/bin/env python

import rospy
from robosub.msg import thruster
from std_msgs.msg import Float64

def print_pretty(name, value):
    print "{:17s}: {: 5f}".format(name, value)

def callback(msg):
    global names, pub
    for i in range(0,len(msg.data)):
        print_pretty(names[i]['name'], msg.data[i])
        pub[i].publish(msg.data[i])
    print "---"

if __name__ == "__main__":
    global names, pub

    names = rospy.get_param('thrusters')
    pub = {}
    sub = rospy.Subscriber('thruster', thruster, callback=callback, queue_size=1)
    for i in range(0,len(names)):
        pub[i] = rospy.Publisher('pretty/thruster/{}'.format(names[i]['name']), Float64, queue_size=1)

    rospy.init_node('thruster_echo', anonymous=True)

    rospy.spin()
