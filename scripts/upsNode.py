#! /usr/bin/env python

import PyNUT
import rospy
from robosub.msg import BatteryDetailed

def upsDataToChargeStatus(status):
    statuses = status.split()  # Split on spaces
    ret = []

    for i in statuses:
        if "DISCHRG" in i:
            ret.append(BatteryDetailed.POWER_SUPPLY_STATUS_DISCHARGING)
        elif "CHRG" in i:
            ret.append(BatteryDetailed.POWER_SUPPLY_STATUS_CHARGING)
        elif "OL" in i:
            ret.append(BatteryDetailed.POWER_SUPPLY_STATUS_ON_LINE)
        elif "OB" in i:
            ret.append(BatteryDetailed.POWER_SUPPLY_STATUS_ON_BATTERY)
        elif "RB" in i:
            ret.append(BatteryDetailed.POWER_SUPPLY_STATUS_REPLACE)
        elif "LB" in i:
            ret.append(BatteryDetailed.POWER_SUPPLY_STATUS_LOW_BATTERY)
        elif "HB" in i:
            ret.append(BatteryDetailed.POWER_SUPPLY_STATUS_HIGH_BATTERY)
        elif "OVER" in i:
            ret.append(BatteryDetailed.POWER_SUPPLY_STATUS_OVERLOAD)
    if not status:
        return BatteryDetailed.POWER_SUPPLY_STATUS_UNKNOWN

    return ret

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
            rospy.logwarn("UPS not detected: Is the UPS plugged in?")
        else:
            state.alive = True
            state.voltageBattery = float(vars['battery.voltage'])
            state.capacity = float(vars['battery.capacity'])
            state.charge = ((float(vars['battery.charge']) / 100.0) *
                            state.capacity)
            state.currentBattery = float(vars['battery.current'])

            state.status = upsDataToChargeStatus(vars['ups.status'])

            state.percentage = float(vars['battery.charge']) / 100.0

            state.temperature = (((9 * float(vars['battery.temperature'])) /
                                 5) + 32)

            state.runtime = rospy.Time(secs=int(vars['battery.runtime']))

            state.voltageInput = float(vars['input.voltage'])
            state.currentInput = float(vars['input.current'])

            state.voltageOutput = float(vars['output.voltage'])
            state.currentOutput = float(vars['output.current'])

        rospy.loginfo("Sending msg")
        pub.publish(state)
        rate.sleep()

if __name__ == "__main__":
    upsNode()
