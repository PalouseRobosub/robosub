#!/usr/bin/env python

import rospy
import roslaunch

class AiManager():

    def __init__(self):
        self.launcher = roslaunch.ROSLaunch()
        self.tasks = rospy.get_param("~tasks")
        rospy.loginfo("Init done")

    def begin(self):
        self.launcher.start()

        for l in self.tasks:
            node = roslaunch.core.Node("robosub", l["node"], args=l["args"],
                                       remap_args=l["remap_args"],
                                       output="screen")
            try:
                process = self.launcher.launch(node)
            except roslaunch.RLException as e:
                rospy.logerror(e.strerror)

            rospy.loginfo("Running " + l["name"] + " Task")
            while process.is_alive():
                pass            # wait


if __name__ == "__main__":
    rospy.init_node('aiManager')
    manager = AiManager()
    manager.begin()
    rospy.spin()
