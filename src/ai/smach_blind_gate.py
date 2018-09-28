#!/usr/bin/python
import rospy
from blind_movement import move_forward
from gate_util import *
from start_switch import start_switch
import smach
import smach_ros
from tf.transformations import euler_from_quaternion
from SubscribeState import SubscribeState
from geometry_msgs.msg import QuaternionStamped
from control_wrapper import control_wrapper
import math
class take_heading(SubscribeState):
    def __init__(self):
        SubscribeState.__init__(self, "orientation", QuaternionStamped,
                                self.callback, outcomes=['success'],
                                output_keys=['heading_output'])

    def callback(self, msg, userdata):
        # get heading
        # convert to yaw
        # output yaw
        quaternion = msg.quaternion
        quaternion = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]*180/math.pi
        userdata.heading_output = yaw
        self.exit("success")

class rotate_to_heading(SubscribeState):
    def __init__(self, yaw_error):
        SubscribeState.__init__(self, "orientation", QuaternionStamped,
                                self.callback, outcomes=['success'],
                                input_keys=['heading_input'])
        self.yaw_error = yaw_error

    def callback(self, msg, userdata):
        # get current heading
        # check if current heading is facing the gate
        # continue turning if not facing the gate

        quaternion = msg.quaternion
        quaternion = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]*180/math.pi
        if abs(yaw-userdata.heading_input) < self.yaw_error:
            self.exit("success")

        c = control_wrapper()
        c.levelOut()
        c.yawLeftAbsolute(userdata.heading_input)
        c.publish()


class blind_gate_task(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success'])

        # these variables will not be found without using rosparam load ai.yaml
        self.time = rospy.get_param("ai/blind_gate_task/forward_time")
        self.speed = rospy.get_param("ai/blind_gate_task/forward_speed")
        self.yaw_error = rospy.get_param("ai/blind_gate_task/yaw_error")

        with self:
            smach.StateMachine.add('TAKE_HEADING', take_heading(),
                                   transitions={'success': 'START_SWITCH'},
                                   remapping={'heading_output': 'heading'})

            smach.StateMachine.add('START_SWITCH', start_switch(),
                                   transitions={'success': 'ROTATE_TO_HEADING'})

            smach.StateMachine.add('ROTATE_TO_HEADING',
                                   rotate_to_heading(self.yaw_error),
                                   transitions={'success': 'MOVE_FORWARD'},
                                   remapping={'heading_input': 'heading'})

            smach.StateMachine.add('MOVE_FORWARD',
                                   move_forward(self.time, self.speed),
                                   transitions={'success': 'success'})

if __name__ == '__main__':
    rospy.init_node('ai')

    while rospy.get_time() == 0:
        continue

    sm = smach.StateMachine(outcomes=['success'])
    with sm:
        smach.StateMachine.add('BLIND_GATE_TASK', blind_gate_task(),
                               transitions={'success': 'success'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcomes = sm.execute()
    sis.stop()
