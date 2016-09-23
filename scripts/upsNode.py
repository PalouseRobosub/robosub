#! /usr/bin/env python

import PyNUT
import rospy
from sensor_msgs.msg import BatteryState

def upsDataToChargeStatus(status):
    if "CHRG" in status:
        return POWER_SUPPLY_STATUS_CHARGING
    if "DISCHRG" in status:
        return POWER_SUPPLY_STATUS_DISCHARGING
    if not status:
        return POWER_SUPPLY_STATUS_UNKNOWN
    
    return POWER_SUPPLY_STATUS_NOT_CHARGING
#    return {
#        'CHRG':1,
#        'OB DISCHRG':2,
#        'OB':3,
#        'OL':4
#    }.get(x,0)

#def upsDataToHealth(status):
#    if "HB" in status:
#        return POWER_SUPPLY_HEALTH_OVERVOLTAGE
#    if "LB" in status:
#        return POWER_SUPPLY_HEALTH_
#    if ""

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
            state.power_supply_technology = POWER_SUPPLY_TECHNOLOGY_LIFE
            
            state.power_supply_status = upsDataToChargeStatus(vars['ups.status']);


        rospy.loginfo("Sending msg");
        pub.publish(state)
        rate.sleep()




if __name__=="__main__":
    upsNode()
