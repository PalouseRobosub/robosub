#!/usr/bin/python
import rospy
from control_wrapper import control_wrapper
import smach
import smach_ros


class move_forward(smach.State):
    def __init__(self, time, value, poll_rate=10):
        smach.State.__init__(self, outcomes=["success"])
        self.time = time
        self.value = value
        self._poll_rate = rospy.Rate(poll_rate)

        # wait for time to be non-zero
        while(rospy.Time.now() == rospy.Time(0)):
            rospy.sleep(1.0/self._poll_rate)

    def execute(self, userdata):
        c = control_wrapper()
        c.levelOut()
        c.forwardError(self.value)
        exit_time = rospy.Time.now() + rospy.Duration(self.time)
        while not rospy.is_shutdown() and rospy.Time.now() < exit_time:
            c.publish()
            self._poll_rate.sleep()
        return 'success'

if __name__ == "__main__":
    pass
