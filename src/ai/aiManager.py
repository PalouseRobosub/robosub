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
            rospy.loginfo("Running node: {0} with remaps: {1}".format(l["node"],
                          l["remap_args"]))
            remap_arguments = []
            for val in l["remap_args"]:
                remap_arguments.append((val["from"], val["to"]))
            node = roslaunch.core.Node("robosub", l["node"], args=l["args"],
                                       remap_args=remap_arguments,
                                       output="screen")
            try:
                process = self.launcher.launch(node)
            except roslaunch.RLException as e:
                rospy.logerr(e.strerror)

            rospy.loginfo("Running " + l["name"] + " Task")
            launch_time = rospy.get_rostime()
            while (process.is_alive() and
                   rospy.get_rostime() - rospy.Duration(l["timeout_secs"]) <
                   launch_time):
                pass            # wait

            process.stop()

if __name__ == "__main__":
    rospy.init_node('aiManager')
    manager = AiManager()
    manager.begin()
    rospy.spin()
