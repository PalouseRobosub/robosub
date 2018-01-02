"""
Author: Ryan Summers
Date: 1-2-2018

Description: Describes a number of basic Smach states for AI development.
"""
import control_wrapper
import numpy as np
import rospy
import smach
import tf

from geometry_msgs.msg import QuaternionStamped
from robosub.msg import Float32Stamped
from std_srvs.srv import Empty
from SubscribeState import SubscribeState

class Stabilize(SubscribeState):
    """Waits a specified timeout to check for stable tilt.

    attributes:
        test_duration: The length (in seconds) that the state may run for.
        stable_duration: The length (in seconds) that the sub must remain stable
            for.
        max_tilt: The maximum angle (in degrees) that the sub may pitch or roll
            to during a stable period.
        timeout: The ros time when the task will force quit
    """

    def __init__(self):
        """Initialization."""
        SubscribeState.__init__(self,
                                'orientation',
                                QuaternionStamped,
                                self.orientation_callback,
                                outcomes=['success', 'fail'],
                                setup_callback=self.setup)
        self.timeout = 0
        self.test_duration = 15
        self.stable_duration = 2
        self.max_tilt = 5


    def setup(self):
        """Initializes state timeouts."""
        self.timeout = rospy.get_time() + self.test_duration
        self.stable_timeout = rospy.get_time() + self.stable_duration


    def orientation_callback(self, orientation_msg, user_data):
        """Callback message for orientation messages."""
        q = orientation_msg.quaternion
        euler = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
        pitch = euler[1] * 180 / np.pi
        roll = euler[0] * 180 / np.pi

        if abs(roll) > self.max_tilt or abs(pitch) > self.max_tilt:
            self.stable_timeout = rospy.get_time() + self.stable_duration
        else:
            if rospy.get_time() > self.stable_timeout:
                self.exit('success')

        c = control_wrapper.control_wrapper()
        c.levelOut()
        c.strafeLeftError(0)
        c.forwardError(0)
        c.publish()

        if rospy.get_time() > self.timeout:
            self.exit('fail')


class GoToDepth(SubscribeState):
    """State the goes to a specific depth.

    attributes:
        timeout: The ros time at which the task will fail.
        max_duration: The maximum length (in seconds) that the state may run.
        depth: The target depth (in meters) to dive to.
        max_error: The maximum error in depth that is acceptable (positive or
            negative).
    """

    def __init__(self, depth, max_duration=10, max_error=0.2):
        """Initializes the state.

        Args:
            max_duration: The max duration (in seconds) that the state may run
                for.
            max_error: The maximum depth error (in meters) that denotes success.
        """
        SubscribeState.__init__(self,
                                'depth',
                                Float32Stamped,
                                self.depth_callback,
                                outcomes=['success', 'fail'],
                                setup_callback=self.setup)
        self.timeout = 0
        self.max_duration = max_duration
        self.depth = depth
        self.max_error = max_error


    def setup(self):
        """Initializes the state timeout timer when the state begins."""
        self.timeout = rospy.get_time() + self.max_duration


    def depth_callback(self, depth_msg, user_data):
        """Callback for ROS depth messages."""
        c = control_wrapper.control_wrapper()
        c.diveAbsolute(self.depth)
        c.levelOut()
        c.publish()

        if abs(abs(depth_msg.data) - abs(self.depth)) < self.max_error:
            self.exit('success')

        if rospy.get_time() > self.timeout:
            self.exit('fail')


class LocateObject(SubscribeState):
    """Rotates the sub in a circle until an object is detected.

    Attributes:
        label: The label of the object that is being searched for.
        yaw_speed: The speed at which the submarine yaws to find objects.
        fov_scale: The angle (in degrees) that the front camera can see.
        max_duration: The maximum duration in seconds for the state to execute.
    """

    def __init__(self, label, yaw_speed=20, fov_scale=90, max_duration=30):
        """Initializes the state.

        Args:
            label: The label of the object that is being searched for.
            yaw_speed: The speed at which the submarine yaws to find objects.
            fov_scale: The field of view (in degrees) of the front camera.
            max_duration: The maximum length of the state in seconds.
        """
        SubscribeState.__init__(self,
                                'vision',
                                DetectionArray,
                                self.detection_callback,
                                outcomes=['success', 'fail'],
                                setup_callback=self.setup)
        self.label = label
        self.yaw_speed = yaw_speed
        self.fov_scale = fov_scale
        self.max_duration = max_duration


    def setup(self):
        """Sets up the state timeout."""
        self.timeout = rospy.get_time() + self.max_duration


    def detection_callback(self, detections, user_data):
        """Callback for detected objects."""
        relevent_detections = util.filterByLabel(detections.detections,
                self.label, thresh=0.5)
        detection = util.getMostProbable(relevent_detections)

        c = control_wrapper.control_wrapper()
        c.levelOut()
        if detection is None:
            rospy.loginfo('{} was not found. Searching left {} '
                    'degrees.'.format(self.label, self.yaw_speed))
            if rospy.get_time() > self.timeout:
                self.exit('fail')

            c.yawLeftRelative(self.yaw_speed)
            c.publish()
        else:
            rospy.loginfo('{} was found.'.format(self.label))
            if 0.25 <= detection.x <= 0.75:
                c.yawLeftRelative(0)
                c.publish()
                self.exit('success')
            else:
                rospy.loginfo('Centering camera on {}'.format(self.label))
                c.yawLeftRelative((0.5 - detection.x) / 0.5 *
                        (self.fov_scale / 2))
                c.publish()


class FireTorpedo(smach.State):
    """Fires a single torpedo.

    Args:
        outcomes: A list of 2 elements, where the first denotes a
            successful outcome.
    """

    def __init__(self, outcomes, input_keys=[], output_keys=[]):
        """Initializes the state."""
        smach.State.__init__(self,
                             outcomes=outcomes,
                             input_keys=input_keys,
                             output_keys=output_keys)
        self.outcomes = outcomes


    def execute(self, user_data):
        """Executes the state."""
        rospy.logdebug('Waiting for shoot_torpedo server.')
        rospy.wait_for_service('shoot_torpedo')
        service = rospy.ServiceProxy('shoot_torpedo', Empty)

        rospy.loginfo('Firing torpedo.')
        service()
        return self.outcomes[0]


class DropMarker(smach.State):
    """Drops a single marker.

    Args:
        outcomes: A list of 2 elements, where the first denotes a
            successful outcome.
    """

    def __init__(self, outcomes, input_keys=[], output_keys=[]):
        """Initializes the state."""
        smach.State.__init__(self,
                             outcomes=outcomes,
                             input_keys=input_keys,
                             output_keys=output_keys)
        self.outcomes = outcomes


    def execute(self, user_data):
        """Executes the state."""
        rospy.logdebug('Waiting for drop_marker server.')
        rospy.wait_for_service('drop_marker')
        service = rospy.ServiceProxy('drop_marker', Empty)

        rospy.loginfo('Dropping marker.')
        service()
        return self.outcomes[0]
