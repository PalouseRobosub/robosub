#!/usr/bin/python
import rospy
import smach
import smach_ros

class SubscribeState(smach.State):
    def __init__(self, topic, msg_type, sub_callback, outcomes, setup_callback=None, timeout=None):
        if 'timeout' not in outcomes:
            outcomes.append('timeout')
        smach.State.__init__(self, outcomes=outcomes)
        self._topic = topic
        self._msg_type = msg_type
        self._sub_callback = callback
        self._timeout = timeout
        self._setup_callback = setup_callback
        self._rate = rospy.Rate(10)

        # public variables
        self.outcome = None
        self.done = False

    def execute(self, user_data):
        self.done = False
        self.outcome = None
        self._sub = rospy.Subscriber(self._topic, self._msg_type,
                                     self._sub_callback)
        if self._setup_callback:
            self.setup_callback()

        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            if self._timeout and rospy.Time.now() - start_time > self._timeout:
                self.outcome = 'timeout'
                break
            if self.done is True:
                break
            else:
                self._rate.sleep()

        self._sub.unregister()
        return self.outcome

if __name__ == "__main__":
    pass
