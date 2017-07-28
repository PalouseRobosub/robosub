#!/usr/bin/env python

import rospy
import roslaunch

from robosub.msg import control

# Uses the roslaunch API to spin up nodes in sequence
class AiManager():

    def __init__(self):
        # Create a roslauncher and fetch task list
        self.launcher = roslaunch.ROSLaunch()
        self.tasks = rospy.get_param("~tasks")
        rospy.loginfo("Init done")

    def begin(self):
        self.launcher.start()

        # Iterate over every task in the list
        for task in self.tasks:
            rospy.loginfo("Running node: {0} with remaps: {1}".format(
                          task["node"],
                          task["remap_args"]))
            remap_arguments = []
            # Add remap arguments in form required by roslaunch
            for val in task["remap_args"]:
                remap_arguments.append((val["from"], val["to"]))
            # Launch the node with requested parameters
            node = roslaunch.core.Node("robosub", task["node"],
                                       args=task["args"],
                                       remap_args=remap_arguments,
                                       output="screen")
            try:
                self.process = self.launcher.launch(node)
            except roslaunch.RLException as e:
                rospy.logerr(e.message)
                break

            rospy.loginfo("Running " + task["name"] + " Task")
            launch_time = rospy.get_rostime()

            # Wait for either the process to die or for its timeout to expire
            while (self.process.is_alive() and
                   rospy.get_rostime() - launch_time <
                   rospy.Duration(task["timeout_secs"])):
                pass

            # Kill the process if it has timed out.
            self.process.stop()

    def shutdown_hook(self):
        rospy.logwarn("AI Manager killed! Shutting down...")
        self.process.stop()
        self.tasks = []
        publisher = rospy.Publisher('control', control, queue_size=1)
        msg = control()

        msg.forward_state = control.STATE_ERROR
        msg.strafe_state = control.STATE_ERROR
        msg.dive_state = control.STATE_ERROR
        msg.yaw_state = control.STATE_ERROR
        msg.pitch_state = control.STATE_ERROR
        msg.roll_state = control.STATE_ERROR

        publisher.publish(msg)

if __name__ == "__main__":
    rospy.init_node('aiManager')
    manager = AiManager()
    rospy.on_shutdown(manager.shutdown_hook)
    manager.begin()
    rospy.spin()
