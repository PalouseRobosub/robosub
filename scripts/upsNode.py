#! /usr/bin/env python

import PyNUT
import rospy
from robosub.msg import BatteryDetailed

def upsDataToChargeStatus(status):
    if "DISCHRG" in status:
        return BatteryDetailed.POWER_SUPPLY_STATUS_DISCHARGING
    if "CHRG" in status:
        return BatteryDetailed.POWER_SUPPLY_STATUS_CHARGING
    if not status:
        return BatteryDetailed.POWER_SUPPLY_STATUS_UNKNOWN

    return BatteryDetailed.POWER_SUPPLY_STATUS_NOT_CHARGING
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
#    if "RB" in status:
#        return POWER_SUPPLY_HEALTH_

def upsNode():
    pub = rospy.Publisher('power/ups', BatteryDetailed, queue_size=1)

    rospy.init_node('upsNode', anonymous=False)

    rate = rospy.Rate(10)

    client = PyNUT.PyNUTClient()

    while not rospy.is_shutdown():
        vars = client.GetUPSVars('openups')

        state = BatteryDetailed()

        if (vars is None):
            state.alive = False
        else:
            state.alive = True
            state.voltageBattery = float(vars['battery.voltage'])
            state.chargeBattery = (float(vars['battery.charge']) / 100.0) * float(vars['battery.capacity'])
            state.capacity = float(vars['battery.capacity'])
            state.current = float(vars['battery.current'])

            state.status = upsDataToChargeStatus(vars['ups.status']);

            state.percentage = float(vars['battery.charge']) / 100.0

            state.temperature = ((9 * float(vars['battery.temperature'])) / 5) + 32

            state.runtime = rospy.Time(secs = int(vars['battery.runtime']))

            state.voltageInput = float(vars['input.voltage'])

            state.currentInput = float(vars['input.current'])

        rospy.loginfo("Sending msg");
        pub.publish(state)
        rate.sleep()




if __name__=="__main__":
    upsNode()
