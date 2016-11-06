#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from robosub.msg import visionPos
import cv2
import numpy as np
from image_converter import ToOpenCV, depthToOpenCV


#this function does our image processing
#returns the location and "size" of the detected object
def process_image(image):
    #convert color space from BGR to HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    #create bounds for our color filter
    lower_bound = np.array([0, 10, 10])
    upper_bound = np.array([10,255,255])

    lower_bound[0] = rospy.get_param("/vision/red/min/hue")
    lower_bound[1] = rospy.get_param("/vision/red/min/sat")
    lower_bound[2] = rospy.get_param("/vision/red/min/val")
    
    upper_bound[0] = rospy.get_param("/vision/red/max/hue")
    upper_bound[1] = rospy.get_param("/vision/red/max/sat")
    upper_bound[2] = rospy.get_param("/vision/red/max/val")
    
    #execute the color filter, returns a binary black/white image
    mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
    #mask = cv2.dilate(mask, np.ones((5,5),np.uint8), iterations=3)

    #display the results of the color filter
    cv2.imshow("image_mask", mask)

    #calculate the centroid of the results of the color filer
    M = cv2.moments(mask)
    location = None
    magnitude = 0

    imWidth = len(image)
    imHeight = len(image[0])
    if M['m00'] > 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        magnitude = M['m00'] / (imWidth * imHeight)#M['m00']
        location = (cx-(imWidth / 2), -1 * (cy-(imHeight / 2))) #scale so that 0,0 is center of screen
        #draw a circle image where we detected the centroid of the object
        cv2.circle(image, (cx,cy), 3, (0,0,255), -1)

    #display the original image with the centroid drawn on the image
    cv2.imshow("processing result", image)

    #waitKey() is necessary for making all the cv2.imshow() commands work
    cv2.waitKey(1)
    return location, magnitude


class Node:
    def __init__(self):
        #register a subscriber callback that receives images
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback, queue_size=1)

        #create a publisher for sending commands to turtlebot
        self.processed_pub = rospy.Publisher('vision/buoy/red', visionPos, queue_size=1)

    #this function wll get called every time a new image comes in
    #all logic occurs in this function
    def image_callback(self, ros_image):
        # convert the ros image to a format openCV can use
        cv_image = np.asarray(ToOpenCV(ros_image))

        #run our vision processing algorithm to pick out the object
        #returns the location (x,y) of the object on the screen, and the
        #"size" of the discovered object. Size can be used to estimate
        #distance
        #None/0 is returned if no object is seen
        location, magnitude = process_image(cv_image)

        #log the processing results
        rospy.logdebug("image location: {}\tmagnitude: {}".format(location, magnitude))


        msg = visionPos()
        
        if location:
            msg.xPos.append(location[0])
            msg.yPos.append(location[1])
            msg.magnitude.append(magnitude)
        #publish command to the turtlebot
        self.processed_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("vision")
    node = Node()

    #this function loops and returns when the node shuts down
    #all logic should occur in the callback function
    rospy.spin()
