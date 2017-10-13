#!/usr/bin/python
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool
from SubscribeState import SubscribeState

class start_switch(SubscribeState):
    def __init__(self, min_count=3):
        # First, call base class's init function with all the necessary
        # parameters:
        # (topic_name, msg_type, subscriber_callback, list_of_outcomes,
        # setup_function)
        SubscribeState.__init__(self, "start_switch", Bool, self.callback,
                               ["success"], setup_callback=self.setup)
        self.min_count = min_count

    # define a setup function to be run every time we enter the state
    def setup(self):
        self.start_counter = 0

    # define the callback to be called when a message comes in on the
    # "start_switch" topic
    def callback(self, msg, userdata):
        # If the switch is activated, increment the counter. If we have
        # received min_count number of positive activations in a row, we are
        # fairly confident the switch has actually been pressed, and can
        # transition out of the state. To exit the state, we set the
        # "self.outcome" variable to the desired outcome, and set the
        # "self.done" variable to True.
        if msg.data is True:
            self.start_counter += 1
            if self.start_counter >= self.min_count:
                self.exit("success")
        # If the switch is not activated, reset the count to zero. This is to
        # prevent the switch from being accidentally tripped
        else:
            self.start_counter = 0

        rospy.logdebug("start switch counter: {}".format(self.start_counter))

if __name__ == "__main__":
    # To see debug messages add log_level=rospy.DEBUG argument to init_node
    rospy.init_node('start_switch_test')

    sm = smach.StateMachine(outcomes=['success'])
    with sm:
        smach.StateMachine.add('START_SWITCH', start_switch(),
                               transitions={'success': 'success'})

    outcome = sm.execute()
