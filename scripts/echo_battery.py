#!/usr/bin/env python

import rospy
from robosub.msg import BatteryDetailed
from datetime import timedelta
'''
' This function returns a string representing the status of the Battery 
'''
def statusToString(status):
    string = ""
    for i in status:
        string += {
                    0:"\x1b[31mUNKNOWN\x1b[0m",
                    1:"\x1b[36mCHARGING\x1b[0m",
                    2:"\x1b[34mDISCHARGING\x1b[0m",
                    3:"\x1b[33mNOT CHARGING\x1b[0m",
                    4:"\x1b[32mFULL\x1b[0m",
                    5:"\x1b[31OVERLOAD\x1b[0m",
                    6:"ON LINE", 
                    7:"ON BATTERY",
                    8:"\x1b[31mLOW BATTERY\x1b[0m",
                    9:"\x1b[31mHIGH BATTERY\x1b[0m",
                    10:"\x1b[1;31mREPLACE BATTERY\x1b[0m"\
                  }.get(i,"\x1b[31mUNKNOWN\x1b[0m")
    return string

'''
' This function gets called every time a message is published to the thruster
'  topic and calls print_pretty on each thruster in the message
'''
def callback(msg):
    print "UPS Battery:"
    print "Alive: {}".format(msg.alive)
    print "Inputs:  Current: {: 9f} Voltage: {: 9f}".format(msg.currentInput, msg.voltageInput)
    print "Outputs: Current: {: 2f} Voltage: {: 2f}".format(msg.currentOutput, msg.voltageOutput)
    print "Battery: Current: {: 2f} Voltage: {: 2f}".format(msg.currentBattery, msg.voltageBattery)
    print "Status: {}".format(statusToString(msg.status))
    print u"Temperature: {: 2f} \u00b0F".format(float(msg.temperature))
    print "Runtime Left: {}".format(timedelta(seconds = msg.runtime.secs))
    print "Percent Available: {} %".format(msg.percentage*100)
    print "Charge: {} Capacity: {}".format(msg.charge, msg.capacity)

    # Print a nice delimiter at the end of each message
    print "---"

if __name__ == "__main__":

    # Subscribe to the UPS battery topic
    sub = rospy.Subscriber('power/ups', BatteryDetailed, callback=callback, queue_size=1)
    
    # Initialize the node (Anonymously)
    rospy.init_node('battery_echo', anonymous=True)

    # Process Callbacks as needed
    rospy.spin()
