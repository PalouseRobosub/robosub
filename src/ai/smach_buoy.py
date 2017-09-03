#!/usr/bin/python
import rospy
from std_msgs.msg import Bool
from rs_yolo.msg import DetectionArray as detection_array
from util import *
import control_wrapper

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

class track_buoy(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])
        self.sub = rospy.Subscriber("vision", detection_array, self.callback)
        self.done = False

        # How close to center the buoy needs to be (abs)
        self.errorGoal = 0.1

        # How close the sub has to get before reversing
        self.distGoal = 0.06

        self.yaw_speed_factor = -25
        self.dive_speed_factor = -1

    def callback(self, msg):
        c = control_wrapper()
        c.levelOut()

        detections = filterByLabel(detection.detections, self.label_name)

        vision_result = getMostProbable(detections, thresh=0.5)

        normalize(vision_result)

        if abs(vision_result.x) > self.errorGoal:
                # Center X (yaw) first
                msg.yaw_state = control.STATE_RELATIVE
                # Calculation found by testing, will be updated with magnitude
                # changes.
                yaw_left = (vision_result.x *
                                (1 - (vision_result.width *
                                      vision_result.height * 10)) *
                                self.yaw_speed_factor)
                c.yawLeftRelative(yaw_left)
                rospy.loginfo("Yaw relative: {}".format(yaw_left))
                # Maintain depth
                c.diveRelative(0)
            else:
                # Now center Y (Dive) and maintain yaw
                c.yawLeftRelative(0)
                # Calculation found by testing, will be updated with magnitude
                # changes.
                dive = (vision_result.y *
                            ((1 - (vision_result.width * vision_result.height *
                                   10)) * self.dive_speed_factor))
                c.diveRelative(dive)
                #also move slowly forward
                c.ForwardError(0.5)

        c.publish()

    def execute(self.userdata):
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
