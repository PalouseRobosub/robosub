#!/usr/bin/python
import rospy
import smach
import smach_ros
import message_filters

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
                                    self._sub_callback,
                                    callback_args=user_data)
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


class SynchronousSubscribeState(smach.State):
    def __init__(self, topic, msg_type, topic2, msg_type2, time, sub_callback,
                 outcomes, input_keys=[], output_keys=[], setup_callback=None,
                 timeout=None, poll_rate=10):
        rospy.logdebug('base class "init" running')
        if timeout is not None and 'timeout' not in outcomes:
            outcomes.append('timeout')
        smach.State.__init__(self, outcomes=outcomes,
                            input_keys=input_keys, output_keys=output_keys)
        self._sync_time = time
        self._topic1 = topic
        self._msg_type1 = msg_type
        self._topic2 = topic2
        self._msg_type2 = msg_type2
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


    def _sync_callback(self, msg1, msg2):
        self._sub_callback(msg1, msg2, self._user_data)


    def execute(self, user_data):
        rospy.logdebug('SynchronousSubscribeState base class "execute" running')
        self._done = False
        self._outcome = None
        self._user_data = user_data

        self._sub1 = message_filters.Subscriber(self._topic1, self._msg_type1)
        self._sub2 = message_filters.Subscriber(self._topic2, self._msg_type2)

        self._sub = message_filters.ApproximateTimeSynchronizer(
                [self._sub1, self._sub2], 10, self._sync_time)
        self._sub.registerCallback(self._sync_callback)

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

        self._sub1.unregister()
        self._sub2.unregister()

        return self._outcome

if __name__ == "__main__":
    pass
