#!/usr/bin/env python

import rospy
from robosub.msg import thruster
from std_msgs.msg import Float64

'''
' This function takes in a thruster name (string) and the current normalized
'  thrust value for the thruster (Float64) and prints it in a nice format
'''
def print_pretty(name, value):
    print "{:17s}: {: 5f}".format(name, value)

'''
' This function gets called every time a message is published to the thruster
'  topic and calls print_pretty on each thruster in the message
'''
def callback(msg):
    global names, pub

    # Loop over all the thrusters in the message
    for i in range(0, len(msg.data)):
        # Pretty Print the thruster name and thrust
        print_pretty(names[i]['name'], msg.data[i])

        # Republish the data for nice viewing in rqt
        pub[i].publish(msg.data[i])

    # Print a nice delimiter at the end of each message
    print "---"

if __name__ == "__main__":
    global names, pub

    # Get the thruster names from the parameter server
    names = rospy.get_param('thrusters/mapping')

    # Declare and initialize the pub variable as a dictionary
    pub = {}

    # Loop over the thruster names in the parameter server generating a topic
    #   for each
    for i in range(0, len(names)):
        # Push the new publisher to the dictionary
        pub[i] = rospy.Publisher('pretty/thruster/{}'.format(names[i]['name']),
                                 Float64, queue_size=1)

    # Subscribe to the thruster topic
    sub = rospy.Subscriber('thruster', thruster, callback=callback,
                           queue_size=1)

    # Initialize the node (Anonymously)
    rospy.init_node('thruster_echo', anonymous=True)

    # Process Callbacks as needed
    rospy.spin()
