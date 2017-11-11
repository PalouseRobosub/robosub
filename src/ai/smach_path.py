#!/usr/bin/python
import rospy
from util import *
from start_switch import start_switch
from robosub.msg import control
from rs_yolo.msg import DetectionArray
from SubscribeState import SubscribeState
from control_wrapper import control_wrapper
from blind_movement import move_forward
from robosub.srv import get_path_angle
import smach
import smach_ros

# Move to marker and center
class center_on_marker(SubscribeState):
    def __init__(self, vision_label):
        SubscribeState.__init__(self, "vision", DetectionArray,
                                self.callback_vision, outcomes=['success', 'nothing'])
        self.vision_label = vision_label
        self.errorGoal = rospy.get_param("ai/center_path/error_goal")
        self.strafe_factor = rospy.get_param("ai/center_path/strafe_factor")
        self.forward_factor = rospy.get_param("ai/center_path/forward_factor")

    def callback_vision(self, detectionArray, userdata):
        self.detectionArray = detectionArray
        c = control_wrapper()
        c.levelOut()
        # forward_factor is a parameter
        c.forwardError(self.forward_factor)

        detections = filterByLabel(self.detectionArray.detections,
                                  self.vision_label)


        vision_result = getMostProbable(detections, thresh=0.5)

        if (len(detections) < 1):
            self.exit("nothing")

        markerXPos = vision_result.x
        markerYPos = vision_result.y


        rospy.loginfo(("Marker X: {}\tMarker Y:{}".format(markerXPos,
                                                          markerYPos)))

        if abs(markerXPos-0.5) > self.errorGoal:
            # If we are not centered by width
            strafe_left = (markerXPos-0.5) * self.strafe_factor
            rospy.loginfo("Trying to strafe: {}".format(strafe_left))
            c.strafeLeftAbsolute(strafe_left)
        elif abs(markerYPos-0.5) > self.errorGoal:
            # If we are not centered by height
            forward = (markerYPos-0.5) * self.forward_factor
            rospy.loginfo("Trying to center Y: {}".format(forward))
            c.forwardAbsolute(forward)
        else:
            self.exit('success')

        c.publish()
# rotate to center
class yaw_to_angle(smach.State):
    def __init__(self, outcomes=['success']):
        rospy.wait_for_service('path_angle')
        self.errorGoal = rospy.get_param("ai/center_path/error_goal")
        self.yaw_factor = rospy.get_param("ai/center_path/yaw_factor")
        self._outcomes = outcomes
        self._input_keys = None
        try:
           self.path_angle = rospy.ServiceProxy('path_angle', get_path_angle)
           response = self.path_angle('path_marker')
           self.angle = response.angle
        except rospy.ServiceException, e:
           print "Service call failed: %s"%e

    def execute(self, userdata):
        c = control_wrapper();
        c.levelOut()
        if abs(self.angle) > 1:
            yaw_amount = (self.angle - 5) * yaw_factor
            rospy.loginfo("Trying to center yaw: {}".format(yaw_amount))
            c.yawLeftRelative(yaw_amount)
        else:
            return 'success'

        c.publish()

class marker_task(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success'])
        self.time = rospy.get_param("ai/gate_task/forward_time")
        self.speed = rospy.get_param("ai/gate_task/forward_speed")
        with self:
            smach.StateMachine.add('CENTER_ON_MARKER',
                center_on_marker('path_marker'),
                transitions={'nothing': 'CENTER_ON_MARKER',
                             'success': 'YAW_TO_ANGLE'})

            smach.StateMachine.add('YAW_TO_ANGLE', yaw_to_angle(),
                transitions={'success': 'BLIND_FORWARD'})

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
