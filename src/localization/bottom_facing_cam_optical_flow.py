import numpy as np
import cv2
import rospy
import rostopic
import std_msgs.msg
from sensor_msgs.msg import Image



class OpticalFlow():
    def __init__(self, arguments):

        # Subscribe to the bottom facing camera topic
        sub = rospy.Subscriber('camera/bottom/image_raw', 'Image', 
                        callback=callback, queue_size=1)


    def callback(self, msg, topic):

        for pixel in msg.data:
            print pixel


if __name__ == "__main__":

    rospy.init_node("OpticalFlow", anonymous=True)

    rospy.spin()
