#!/usr/bin/python
import rospy
import smach
import smach_ros

"""
This state class is designed for subscribing to a topic and performing
operations when the subscribed topic callback is executed. The user must define
outcomes, and optionally can set a timeout so the state returns the "timeout"
outcome if the state does not transition out before the timeout occurs.
"""
class SubscribeState(smach.State):
    def __init__(self, topic, msg_type, sub_callback, outcomes, input_keys=[],
                output_keys=[], setup_callback=None, timeout=None,
                poll_rate=10):
        rospy.logdebug('base class "init" running')
        if timeout is not None and 'timeout' not in outcomes:
            outcomes.append('timeout')
        smach.State.__init__(self, outcomes=outcomes,
                            input_keys=input_keys, output_keys=output_keys)
        self._topic = topic
        self._msg_type = msg_type
        self._sub_callback = sub_callback
        if timeout is not None:
            self._timeout = rospy.Duration(timeout)
        else:
            self._timeout = None
        self._setup_callback = setup_callback
        self._outcome = None
        self._done = False
        self._poll_rate = rospy.Rate(poll_rate)

    def exit(self, outcome):
        self._done = True
        self._outcome = outcome

    def execute(self, user_data):
        rospy.logdebug('SubscribeState base class "execute" running')
        self._done = False
        self._outcome = None
        self._sub = rospy.Subscriber(self._topic, self._msg_type,
                                     self._sub_callback, callback_args=user_data)
        if self._setup_callback:
            self._setup_callback()

        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            if self._timeout and rospy.Time.now() - start_time > self._timeout:
                self._outcome = 'timeout'
                break
            if self._done is True:
                break
            else:
                self._poll_rate.sleep()

        self._sub.unregister()
        return self._outcome

if __name__ == "__main__":
    pass
