#!/usr/bin/python
import rospy
from get_path_marker_angle import *
from util import *
from sensor_msgs.msg import Image
from robosub.msg import control
from rs_yolo.msg import DetectionArray
from SubscribeState import SubscribeState
from control_wrapper import control_wrapper
from blind_movement import move_forward
import smach
import smach_ros

# Move to marker and center
class center_on_marker(SubscribeState):
    def __init__(self, vision_label, poll_rate=10):
        SubscribeState.__init__(self, "vision", DetectionArray,
                                self.callback, outcomes=['success', 'nothing'])
        self.vision_label = vision_label
        self.errorGoal = rospy.get_param("ai/center/errorGoal")
        self.yaw_factor = rospy.get_param("ai/center/yaw_factor")
        self.strafe_factor = rospy.get_param("ai/center/strafe_factor")
        self.forward_factor = rospy.get_param("ai/center/forward_factor")

    def callback(self, detectionArray):
        print ("detectionArray: {}".format(detectionArray.detections))
        self.detectionArray = detectionArray
        self.image = rospy.Subscriber("/camera/right/image_raw", Image,
                                      self.callback_vision)

    def callback_vision(self, Image):
        c = control_wrapper()
        c.levelOut()
        # forward_factor is a parameter
        c.forwardError(self.forward_factor)

        print ("detectionArray: {}".format(self.detectionArray.detections))

        detections = filterByLabel(self.detectionArray.detections,
                                  self.vision_label)

        print ("detections: {}".format(detections))


        vision_result = getMostProbable(detections, thresh=0.5)
        print ("vision_result_x: {}".format(vision_result.x))
        normalize(vision_result)
        print ("vision_result_x: {}".format(vision_result.x))

        if (len(detections) < 1):
            self.exit("nothing")

        markerXPos = (vision_result.x + vision_result.width) / 2
        markerYPos = (vision_result.y + vision_result.height) / 2
        
        marker_angle = getPathMarkerAngle(Image, vision_result.x,
                                         vision_result.y, vision_result.width,
                                         vision_result.height)

        rospy.loginfo(("Marker X: {}\tMarker Y:{}".format(markerXPos, markerYPos)))

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
        elif abs(marker_angle) > self.errorGoal:
            # If we are not centered by yaw
            yaw_left = (marker_angle+0.5) * self.yaw_factor
            rospy.loginfo("Trying to yaw: {}".format(yaw_left))
            c.yawLeftRelative(yaw_left * 2)
        else:
            self.exit('centred')

        c.publish()

class marker_task(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success'])
        self.time = rospy.get_param("ai/gate_task/forward_time")
        self.speed = rospy.get_param("ai/gate_task/forward_speed")
        with self:
            smach.StateMachine.add('CENTER_ON_MARKER',
                center_on_marker('path_marker'),
                transitions={'success': 'BLIND_FORWARD',
                             'nothing': 'CENTER_ON_MARKER'})

            smach.StateMachine.add('BLIND_FORWARD',
                move_forward(self.time, self.speed),
                transitions={'success':'success'})

if __name__ == '__main__':
    rospy.init_node('marker_task')
    sm = marker_task()

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcomes = sm.execute()
    sis.stop()
