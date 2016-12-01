#!/usr/bin/python

import rospy

from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from robosub.msg import visionPosArray

def orientationCallBack(msg):
	return

def depthCallBack(msg):
	return

def redBuoyVisionCallBack(msg):
	return

def greenBuoyVisionCallBack(msg):
	return

def blueBuoyVisionCallBack(msg):
	return

topicNames = {"/orientation"       : (Quaternion, orientationCallBack), 
			  "/depth"             : (Float32, depthCallBack), 
			  "/vision/buoy/red"   : (visionPosArray, redBuoyVisionCallBack), 
			  "/vision/buoy/green" : (visionPosArray, blueBuoyVisionCallBack), 
			  "/vision/buoy/blue"  : (visionPosArray, greenBuoyVisionCallBack)}

#vision/buoy/(red|green|blue)

def main():
	print("main()")

	for topic in topicNames:
		print("Subscribing to topic " + topic + " type: " + str(topicNames[topic][0]))
		#rospy.Subscriber(topic, topicNames[topic][0], topicNames[topic][1])

main()