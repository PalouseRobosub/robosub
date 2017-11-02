#!/usr/bin/python
import rospy
from util import *
from start_switch import start_switch
from robosub.msg import control
from rs_yolo.msg import DetectionArray
from SubscribeState import SubscribeState
from control_wrapper import control_wrapper
from blind_movement import move_forward
import smach
import smach_ros

# Move to marker and center
class center_on_marker(SubscribeState):
    def __init__(self, vision_label):
        SubscribeState.__init__(self, "vision", DetectionArray,
                                self.callback, outcomes=['success', 'nothing'])
        self.vision_label = vision_label
        self.errorGoal = rospy.get_param("ai/center/errorGoal")
        self.strafe_factor = rospy.get_param("ai/center/strafe_factor")
        self.forward_factor = rospy.get_param("ai/center/forward_factor")

    def callback_vision(self, detectionArray):
        self.detectionArray = detectionArray
        c = control_wrapper()
        c.levelOut()
        # forward_factor is a parameter
        c.forwardError(self.forward_factor)

        print ("detectionArray: {}".format(self.detectionArray.detections))

        detections = filterByLabel(self.detectionArray.detections,
                                  self.vision_label)

        print ("detections: {}".format(detections))


        vision_result = getMostProbable(detections, thresh=0.5)
        normalize(vision_result)

        if (len(detections) < 1):
            self.exit("nothing")

        markerXPos = vision_result.x
        markerYPos = vision_result.y


        rospy.loginfo(("Marker X: {}\tMarker Y:{}".format(markerXPos,
                                                          markerYPos)))

        if abs(markerXPos+0.5) > self.errorGoal:
            # If we are not centered by width
            strafe_left = (markerXPos+0.5) * self.strafe_factor
            rospy.loginfo("Trying to strafe: {}".format(strafe_left))
            c.strafeLeftRelative(strafe_left * 2)
        elif abs(markerYPos+0.5) > self.errorGoal:
            # If we are not centered by height
            forward = (markerYPos+0.5) * self.forward_factor
            rospy.loginfo("Trying to center Y: {}".format(forward))
            c.forwardRelative(forward)
        else:
            self.exit('centred')

        c.publish()
# Used if angle topic is created
    #def yaw_to_angle(SubscribeState):
    #    SubscribeState.__init__(self, '[angle_topic]', [angle], self.callback,
    #                            outcomes=['success'])
    #    self.path_angle = [angle];
    #    self.errorGoal = rospy.get_param("ai/center/errorGoal")
    #    self.yaw_factor = rospy.get_param("ai/center/yaw_factor")
    #    c = control_wrapper();
    #    c.levelOut()
    #    if self.path_angle > self.errorGoal:
    #        c.yawLeftRelative(self, yaw_factor)
    #    else:
    #        self.exit('success')

    #    c.publish()

class marker_task(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success'])
        self.time = rospy.get_param("ai/gate_task/forward_time")
        self.speed = rospy.get_param("ai/gate_task/forward_speed")
        with self:
            smach.StateMachine.add('CENTER_ON_MARKER',
                center_on_marker('path_marker'),
                transitions={'nothing': 'CENTER_ON_MARKER',
                            # 'success': 'YAW_TO_ANGLE',
                             'success': 'BLIND_FORWARD'})

            # Will be used if angle topic is created
            #smach.StateMachine.add('YAW_TO_ANGLE', yaw_to_angle('angle'),
            #    transitions={'success': 'BLIND_FORWARD'})

            smach.StateMachine.add('BLIND_FORWARD',
                move_forward(self.time, self.speed),
                transitions={'success':'success'})

if __name__ == '__main__':
    rospy.init_node('ai')
    sm = marker_task()
    with sm:
        smach.StateMachine.add('START_SWITCH', start_switch(),
                               transitions={'success': 'PATH_TASK'})
        smach.StateMachine.add('PATH_TASK', marker_task(),
                               transitions={'success': 'success'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcomes = sm.execute()
    sis.stop()
