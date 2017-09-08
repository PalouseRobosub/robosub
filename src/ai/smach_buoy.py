#!/usr/bin/python
import rospy
from std_msgs.msg import Bool
from rs_yolo.msg import DetectionArray as detection_array
from util import *
from control_wrapper import control_wrapper
import smach
import smach_ros

class start_switch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])
        self.start_counter = 0
        self.done = False

    def switch_callback(self, msg):
        if msg.data == True:
            self.start_counter += 1
            if self.start_counter >= 3:
                self.done = True
        else: # i == False
            self.start_counter = 0

    def execute(self, userdata):
        self.sub = rospy.Subscriber("start_switch", Bool, self.switch_callback)
        rospy.loginfo('waiting for start switch')
        while not rospy.is_shutdown():
            if self.done == True:
                rospy.loginfo('got here')
                return 'success'
            rospy.sleep(0.1)
        self.sub.unregister()


class track_buoy(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'lost_buoy'])
        self.done = False
        self.label_name = 'red_buoy'

        # How close to center the buoy needs to be (abs)
        self.errorGoal = 0.1

        # How close the sub has to get before reversing
        self.distGoal = 0.06

        self.yaw_speed_factor = -50
        self.dive_speed_factor = -1

    def callback(self, msg):
        c = control_wrapper()
        c.levelOut()
        c.forwardError(0.0)

        detections = filterByLabel(msg.detections, self.label_name)

        vision_result = getMostProbable(detections, thresh=0.5)

        normalize(vision_result)

        if vision_result is not None:
            if abs(vision_result.x) > self.errorGoal:
                    # Center X (yaw) first
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
                c.forwardError(0.2)

        c.diveAbsolute(-0.4)

        c.publish()

    def execute(self, userdata):
        rospy.sleep(2)
        return 'success'
        self.sub = rospy.Subscriber("vision", detection_array, self.callback)
        while not rospy.is_shutdown():
            if self.done == True:
                return 'success'
            rospy.sleep(0.1)

        self.sub.unregister()

class find_buoy(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.sleep(2)
        return 'success'

class ram_buoy(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.sleep(2)
        return 'success'

class hit_buoy(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success'])
        with self:
            smach.StateMachine.add("FIND_BUOY", find_buoy(),
                                   transitions={'success': 'TRACKING'})
            smach.StateMachine.add("TRACKING", track_buoy(),
                                   transitions={'success': 'RAM_BUOY',
                                                'lost_buoy': 'FIND_BUOY'})
            smach.StateMachine.add("RAM_BUOY", ram_buoy(),
                                   transitions={'success': 'success'})


class reset_position(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.sleep(2)
        return 'success'

if __name__ == "__main__":
    rospy.init_node('buoy_ai', log_level=rospy.INFO)

    sm = smach.StateMachine(outcomes=['success'])

    with sm:
        smach.StateMachine.add('START_SWITCH', start_switch(),
                               transitions={'success':'HIT_BUOY_RED'})
        smach.StateMachine.add('HIT_BUOY_RED', hit_buoy(),
                               transitions={'success':'RESET_FOR_GREEN'})
        smach.StateMachine.add('RESET_FOR_GREEN', reset_position(),
                               transitions={'success':'HIT_BUOY_GREEN'})
        smach.StateMachine.add('HIT_BUOY_GREEN', hit_buoy(),
                               transitions={'success':'RESET_FOR_YELLOW'})
        smach.StateMachine.add('RESET_FOR_YELLOW', reset_position(),
                               transitions={'success':'HIT_BUOY_YELLOW'})
        smach.StateMachine.add('HIT_BUOY_YELLOW', hit_buoy(),
                               transitions={'success':'success'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()

    sis.stop()
