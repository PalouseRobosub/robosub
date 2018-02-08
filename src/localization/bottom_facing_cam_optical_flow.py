import numpy as np
import cv2
import rospy
import rostopic
import std_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class OpticalFlow():
   
    def __init__(self):

        # Subscribe to the bottom facing camera topic
        sub = rospy.Subscriber("/camera/bottom/image_raw", Image, 
                        self.callback, queue_size=1)
        # params for ShiTomasi corner detection
        self.feature_params = dict( maxCorners = 100,
                            qualityLevel = 0.3,
                            minDistance = 7,
                            blockSize = 7 )
        
        # Parameters for lucas kanade optical flow
        self.lk_params = dict( winSize  = (15,15),
                        maxLevel = 2,
                        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        
        # Create some random colors
        self.color = np.random.randint(0,255,(100,3))

        self.old_gray = None
        self.p0 = None
        self.mask = None


        #ros image to opencv image converter
        self.bridge = CvBridge()
        self.previous_time_seconds = rospy.get_time()
        self.current_time_seconds = rospy.get_time()

    def callback(self, msg):

        self.previous_time_seconds = self.current_time_seconds
        self.current_time_seconds = rospy.get_time()
        time_difference = self.current_time_seconds - self.previous_time_seconds

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg)
        except CvBridgeError as e:
            rospy.logerr(e)

        # Create a mask image for drawing purposes
        if self.mask is None:
            mask = np.zeros_like(cv_image)

        frame_gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
            
        
        if self.old_gray is not None:
            # calculate optical flow
            p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, frame_gray, p0, None, **self.lk_params)
            
            # Select good points
            good_new = p1[st==1]
            good_old = p0[st==1]
            
            # draw the tracks
            for i,(new,old) in enumerate(zip(good_new,good_old)):
                a,b = new.ravel()
                c,d = old.ravel()

                dist = ((a-c)**2 + (b-d)**2)**0.5
                velocity = dist/time
                rospy.loginfo("Velocity estimate: " + velocity)

                #mask = cv2.line(mask, (a,b),(c,d), self.color[i].tolist(), 2)
                #frame = cv2.circle(cv_image,(a,b),5,self.color[i].tolist(),-1)
            
            img = cv2.add(frame,mask)
            
            cv2.imshow('frame',img)
            
            # Now update the previous frame and previous points
            old_gray = frame_gray.copy()
            p0 = good_new.reshape(-1,1,2)

        else:
            # Take first frame and find corners in it
            old_gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
            p0 = cv2.goodFeaturesToTrack(old_gray, mask = None, **self.feature_params)


        #cv2.destroyAllWindows()
        #cap.release()



if __name__ == "__main__":

    rospy.init_node("OpticalFlow", anonymous=True)

    optFlow = OpticalFlow()

    rospy.spin()
