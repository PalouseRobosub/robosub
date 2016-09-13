#! /usr/bin/env python

import PyNUT
import rospy
from sensor_msgs.msg import BatteryState

def upsNode():
    pub = rospy.Publisher('power/ups', BatteryState, queue_size=1)

    rospy.init_node('upsNode', anonymous=False)

    rate = rospy.Rate(10)

    client = PyNUT.PyNUTClient()

    while not rospy.is_shutdown():
        vars = client.GetUPSVars('openups')

        state = BatteryState()
        if (vars is None):
            state.present = False
        else:
            state.present = True
            state.voltage = float(vars['battery.voltage'])
            state.charge = float(vars['battery.charge'])


        rospy.loginfo("Sending msg");
        pub.publish(state)
        rate.sleep()




if __name__=="__main__":
    upsNode()
