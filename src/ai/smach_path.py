#!/usr/bin/python
import rospy
import tf
import numpy as np
import util
from util import *
from start_switch import start_switch
from robosub_msgs.msg import control
from robosub_msgs.msg import DetectionArray
from geometry_msgs.msg import QuaternionStamped
from SubscribeState import SubscribeState
from control_wrapper import control_wrapper
from blind_movement import move_forward
from robosub.srv import get_path_angle
import smach
import smach_ros

# Move to marker and center
class center_on_marker(SubscribeState):
    def __init__(self, vision_label):
        SubscribeState.__init__(self, "vision/bottom", DetectionArray,
                                self.callback_vision, outcomes=['success',
                                                                'nothing'])
        self.vision_label = vision_label
        self.errorGoal = rospy.get_param("ai/center_path/error_goal")
        self.strafe_factor = rospy.get_param("ai/center_path/strafe_factor")
        self.forward_factor = rospy.get_param("ai/center_path/forward_factor")

    def callback_vision(self, detectionArray, userdata):
        self.detectionArray = detectionArray
        c = control_wrapper()
        c.clearState()
        c.levelOut()
        c.diveAbsolute(-1.50)

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
            c.strafeLeftError(strafe_left)
        elif abs(markerYPos-0.5) > self.errorGoal:
            # If we are not centered by height
            forward = (markerYPos-0.5) * self.forward_factor
            rospy.loginfo("Trying to center Y: {}".format(forward))
            c.forwardError(forward)
        else:
            self.exit('success')

        c.publish()
# rotate to center
class yaw_to_angle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'trying'],
                output_keys=['yaw_left'])
        self.errorGoal = rospy.get_param("ai/center_path/error_goal")
        self.yaw_factor = rospy.get_param("ai/center_path/yaw_factor")

    def execute(self, userdata):
        rospy.wait_for_service('path_angle')
        try:
           self.path_angle = rospy.ServiceProxy('path_angle', get_path_angle)
           response = self.path_angle('path_marker')
           self.angle = response.angle
        except rospy.ServiceException, e:
           print "Service call failed: %s"%e

        c = control_wrapper();
        c.clearState()
        c.levelOut()
        c.forwardError(0)
        c.diveAbsolute(-1.5)
        if abs(self.angle) > 5:
            yaw_amount = (self.angle) * self.yaw_factor
            rospy.loginfo("Trying to center yaw: {}".format(yaw_amount))
            c.yawLeftRelative(yaw_amount)
            c.publish()
            return 'trying'
        else:
            c.yawLeftRelative(0)
            c.publish()
            userdata.yaw_left = 0
            return 'success'

class YawRelative(SubscribeState):
    """Yaws relative to the starting yaw and waits for stability.
       Attributes: target_yaw: The target yaw of the state.
       max_error: The maximum allowed error in yaw for success (in degrees).
    """
    def __init__(self, max_duration=30, max_error=5):
        """Initializes the SMACH state.
        Args:
        max_duration: The maximum duration of the state in seconds.
        max_error: The maximum yaw error allowed in degrees.
        """
        SubscribeState.__init__(self,'orientation', QuaternionStamped,
                self.orientation_callback, outcomes=['success'],
                input_keys=['yaw_left'], output_keys=['yaw_left'],
                timeout=max_duration, setup_callback=self.setup)
        self.target_yaw = None
        self.max_error = max_error
    def setup(self):
       """Setup function for the state."""
       self.target_yaw = None

    def orientation_callback(self, orientation_msg, user_data):
        q = orientation_msg.quaternion
        euler = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
        yaw = euler[2] * 180 / np.pi
        rospy.logdebug('Yaw is {} -> Target: {}'.format(yaw, self.target_yaw))
        if self.target_yaw is None:
            self.target_yaw = util.wrap_yaw(yaw + user_data.yaw_left)
            c = control_wrapper.control_wrapper()
            c.levelOut()
            c.yawLeftRelative(user_data.yaw_left)
            c.forwardError(0)
            c.strafeLeftError(0)
            c.publish()
            return

        if abs(util.wrap_yaw(self.target_yaw - yaw)) < self.max_error:
            user_data.yaw_left = 60
            self.exit('success')

class follow_path(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'done'],
                input_keys=['yaw_left'], output_keys=['yaw_left'])
        self.errorGoal = rospy.get_param("ai/center_path/error_goal")

    def execute(self, user_data):
        c = control_wrapper();
        c.clearState()
        c.levelOut()
        c.diveAbsolute(-1.5)

        rospy.loginfo("Following Path")
        c.forwardError(.5)
        c.publish()
        rospy.sleep(1)
        c.forwardError(0)
        c.publish()

        if user_data.yaw_left == 0:
            user_data.yaw_left = -30
        elif user_data.yaw_left == -30:
            user_data.yaw_left = 60
        elif user_data.yaw_left == 60:
            user_data.yaw_left = -30
        else:
            return 'done'

        return 'success'

class marker_task(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success'])
        self.time = rospy.get_param("ai/gate_task/forward_time")
        self.speed = rospy.get_param("ai/gate_task/forward_speed")
        with self:
            smach.StateMachine.add('CENTER_ON_MARKER',
                center_on_marker('path_marker'),
                transitions={'success': 'YAW_TO_ANGLE',
                             'nothing': 'CENTER_ON_MARKER'})

            smach.StateMachine.add('YAW_TO_ANGLE',
                yaw_to_angle(),
                transitions={'success': 'CREEP_FORWARD',
                             'trying' : 'YAW_TO_ANGLE'})

            smach.StateMachine.add('CREEP_FORWARD',
                follow_path(),
                transitions={'success': 'YAW_RELATIVE',
                             'done': 'success'},
                remapping={'yaw_left': 'yaw_left'})

            smach.StateMachine.add('YAW_RELATIVE',
                    YawRelative(),
                    transitions={'success': 'CREEP_FORWARD',
                                 'timeout': 'YAW_RELATIVE'},
                    remapping={'yaw_left': 'yaw_left'})

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
