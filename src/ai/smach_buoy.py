#!/usr/bin/python
from std_msgs.msg import Bool

class start_switch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])
        self.sub = rospy.Subscriber("start_swith", Bool, self.switch_callback)
        self.start_counter = 0
        self.done = False

    def switch_callback(self, msg):
        if i == True:
            self.start_counter += 1
            if self.start_counter >= 3:
                self.done = True
        else: # i == False
            self.start_counter = 0

    def execute(self, userdata):
        rospy.loginfo('waiting for start switch')
        while not rospy.is_shutdown():
            if self.done == True:
                return 'success'
            rospy.sleep(0.1)




if __name__ == "__main__":
    rospy.init_node('buoy_ai', log_level=rospy.INFO)

    sm = smach.StateMachine(outcomes=['success', 'failure'])

    with sm:
        smach.StateMachine.add('start_switch', start_switch(),
                               transitions={'success':'HIT_BUOY'})
        smach.StateMachine.add('HIT_BUOY_RED', hit_buoy(),
                               transitions={'success':'RESET',
                                            'failure':'RESET'},
                               )
        smach.StateMachine.add('RESET'
