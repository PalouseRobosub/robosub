#!/usr/bin/python
import rospy
from std_msgs.msg import Bool
from rs_yolo.msg import DetectionArray
from util import *
from control_wrapper import control_wrapper
from start_switch import start_switch
from SubscriberState import SubscriberState
import blind_movement
import smach
import smach_ros


class track_buoy(SubscriberState):
    def __init__(self, vision_label):
        smach.State.__init__(self, "vision", DetectionArray, self.callback,
            outcomes=['success', 'lost_buoy'], self.setup)
        self.vision_label = vision_label

        # How close to center the buoy needs to be (abs)
        self.errorGoal = 0.05

        # How close the sub has to get before reversing
        self.distGoal = 0.050

        self.yaw_speed_factor = -35
        self.dive_speed_factor = -1

    def setup(self):
        self.lost_counter = 0
        self.lost_counter_max = 5

    def callback(self, msg):
        c = control_wrapper()
        c.levelOut()
        c.forwardError(0.0)

        detections = filterByLabel(msg.detections, self.vision_label)

        vision_result = getMostProbable(detections, thresh=0.5)

        normalize(vision_result)


        if vision_result is not None:
            vision_result.x -= 0.25
            self.lost_counter = 0
            if abs(vision_result.x) > self.errorGoal:
                    # Center X (yaw) first
                    # Calculation found by testing, will be updated with
                    # magnitude changes.
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
                rospy.loginfo("Dive relative: {}".format(dive))
                c.diveRelative(dive)

            volume = abs(vision_result.width * vision_result.height)

            if (volume > self.distGoal*2):
                self.exit("success")
                return
            # if we are far away and roughly centered on the buoy
            elif (abs(vision_result.x) < self.errorGoal) and \
                 (abs(vision_result.y) < self.errorGoal):

                # if we are close enough, transition to ram state
                volume = abs(vision_result.width * vision_result.height)
                rospy.loginfo("volume: {}".format(volume))
                if volume > self.distGoal:
                    self.exit("success")
                    return
                else:  # move slowly forward
                    c.forwardError(0.2)
        else:  # we can't see the buoy
            self.lost_counter += 1
            if self.lost_counter > self.lost_counter_max:
                self.exit("lost_buoy")
            return

        c.publish()

class find_buoy(smach.State):
    def __init__(self, vision_label):
        smach.State.__init__(self, "vision", DetectionArray, self.callback,
            outcomes=['success'])
        self.vision_label = vision_label
        self.errorGoal = 0.1
        self.yaw_speed_factor = -25

    def callback(self, msg):
        c = control_wrapper()
        c.levelOut()
        c.forwardError(0.0)

        detections = filterByLabel(msg.detections, self.vision_label)

        vision_result = getMostProbable(detections, thresh=0.5)

        normalize(vision_result)

        if vision_result is None:
            # if we can't see the object in question, yaw
            yaw_left = 5
            c.yawLeftRelative(yaw_left)
        else:  # we can see the object
            # if we aren't close to center
            if abs(vision_result.x) > self.errorGoal:
                    # Center X (yaw) first
                    # Calculation found by testing, will be updated with
                    # magnitude changes.
                    yaw_left = (vision_result.x *
                                    (1 - (vision_result.width *
                                          vision_result.height * 10)) *
                                    self.yaw_speed_factor)
                    c.yawLeftRelative(yaw_left)
                    rospy.loginfo("Yaw relative: {}".format(yaw_left))
            else:  # object is close to center of screen, this state is done
                self.exit("success")

        c.publish()

class hit_buoy(smach.StateMachine):
    def __init__(self, vision_label):
        smach.StateMachine.__init__(self, outcomes=['success'])
        with self:
            smach.StateMachine.add("FIND_BUOY", find_buoy(vision_label),
                transitions={'success': 'TRACKING'})

            smach.StateMachine.add("TRACKING", track_buoy(vision_label),
                transitions={'success': 'RAM_BUOY',
                'lost_buoy': 'FIND_BUOY'})

            smach.StateMachine.add("RAM_BUOY",
                blind_movement.move_forward(time=4, value=0.4),
                transitions={'success': 'success'})


if __name__ == "__main__":
    rospy.init_node('buoy_ai', log_level=rospy.INFO)

    sm = smach.StateMachine(outcomes=['success'])

    with sm:
        smach.StateMachine.add('START_SWITCH', start_switch(),
                               transitions={'success': 'HIT_BUOY_RED'})

        smach.StateMachine.add('HIT_BUOY_RED', hit_buoy('red_buoy'),
                               transitions={'success': 'RESET_FOR_GREEN'})

        smach.StateMachine.add('RESET_FOR_GREEN',
                               move_forward(time=10, value=-0.8),
                               transitions={'success': 'HIT_BUOY_GREEN'})

        smach.StateMachine.add('HIT_BUOY_GREEN', hit_buoy('green_buoy'),
                               transitions={'success': 'RESET_FOR_YELLOW'})

        smach.StateMachine.add('RESET_FOR_YELLOW',
                               move_forward(time=10, value=-0.8),
                               transitions={'success': 'HIT_BUOY_YELLOW'})

        smach.StateMachine.add('HIT_BUOY_YELLOW', hit_buoy('yellow_buoy'),
                               transitions={'success': 'BACKUP'})

        smach.StateMachine.add('BACKUP',
                               move_forward(time=10, value=-0.8),
                               transitions={'success': 'success'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()

    sis.stop()
