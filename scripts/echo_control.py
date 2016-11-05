#!/usr/bin/env python

import rospy
from robosub.msg import control

state_types = {
    0: "NONE",
    1: "ABSOLUTE",
    2: "RELATIVE",
    3: "ERROR"
}

def print_pretty(which, state, value):
    print "{:10s} {:10} {}".format(which, state_types[state], value)

def callback(msg):
    print_pretty("forward", msg.forward_state, msg.forward)
    print_pretty("strafe", msg.strafe_state, msg.strafe_right)
    print_pretty("dive", msg.dive_state, msg.dive)
    print_pretty("roll", msg.roll_state, msg.roll_right)
    print_pretty("pitch", msg.pitch_state, msg.pitch_up)
    print_pretty("yaw", msg.yaw_state, msg.yaw_right)
    print "---"

if __name__ == "__main__":
    sub = rospy.Subscriber('control', control, callback=callback, queue_size=1)
    rospy.init_node('control_pretty_printer', anonymous=True)

    rospy.spin()
