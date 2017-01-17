#!/usr/bin/env python

import rospy
import roslaunch

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
                process = self.launcher.launch(node)
            except roslaunch.RLException as e:
                rospy.logerr(e.message)
                break

            rospy.loginfo("Running " + task["name"] + " Task")
            launch_time = rospy.get_rostime()

            # Wait for either the process to die or for its timeout to expire
            while (process.is_alive() and
                   rospy.get_rostime() - rospy.Duration(task["timeout_secs"]) <
                   launch_time):
                pass

            # Kill the process if it has timed out.
            process.stop()

if __name__ == "__main__":
    rospy.init_node('aiManager')
    manager = AiManager()
    manager.begin()
    rospy.spin()
