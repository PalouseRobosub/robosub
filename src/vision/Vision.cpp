#include "VisionProcessor.hpp"
#include "wfov_camera_msgs/WFOVImage.h"
#include "robosub/visionPos.h"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <vector>
#include <string>

using std::vector;

ros::Publisher pub;

void leftCamCallback(const wfov_camera_msgs::WFOVImage::ConstPtr& msg)
{
    //Create a vision processor
    VisionProcessor vp;

    //Copy the image for processing
    //TODO: This should be optimized to avoid copying by sharing the pointer
    Image imgCopy = msg->image;

    //Process the image using the VisionProcessor
    Mat processed = vp.process(imgCopy);

    //Clone the output image for showing
    Mat procOut = processed.clone();

    //Create a Mat of the original image for showing
    Mat original = cv_bridge::toCvShare(msg->image, msg,
                                    sensor_msgs::image_encodings::BGR8)->image;

    //Find contours
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(processed, contours, hierarchy, CV_RETR_TREE,
                 CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    //Create the output message
    robosub::visionPos outMsg;

    //If there are no contours, no calculations needed
    if (contours.size() >= 1)
    {
        //Find the area of the first contour
        double largestArea = contourArea(contours[0], false);
        int largestIndex = 0;

        //Compare the first contour to other areas to find contour with the
        //largest area
        for (unsigned int i = 1; i < contours.size(); i++)
        {
            double area = contourArea(contours[i], false);
            if (area > largestArea)
            {
                largestArea = area;
                largestIndex = i;
            }
        }

        //Find the moments (physical properties) of the contour
        Moments moment;
        moment = moments(contours[largestIndex], false);

        int cx = -1;
        int cy = -1;
        int imWidth = msg->image.width;
        int imHeight = msg->image.height;
        //Using moments, find the center point of the contour
        if (moment.m00 > 0)
        {
            //Find x and y coordinates with 0,0 being top left corner
            cx = static_cast<int>(moment.m10/moment.m00);
            cy = static_cast<int>(moment.m01/moment.m00);
            Point2f center = cv::Point2f(cx, cy);
            std::cout << "Center at: " << "[" << cx - (imWidth/2) << "," <<
                                    -1*(cy-(imHeight / 2)) << "]" << std::endl;
            circle(original, center, 5, Scalar(255, 255, 255), -1);
            //Draw a circle on the original image for location visualization
            circle(original, center, 4, Scalar(0, 0, 255), -1);
        }

        ROS_DEBUG_STREAM("Preparing output");
        //Prepare the output message
        outMsg.xPos = cx - (imWidth / 2);
        ROS_DEBUG_STREAM("X prepared");
        outMsg.yPos = cy - (imHeight / 2);
        ROS_DEBUG_STREAM("Y prepared");
        outMsg.magnitude = static_cast<double>(largestArea) /
                           static_cast<double>(imWidth * imHeight);
        ROS_DEBUG_STREAM("Magnitude prepared");
    }

    //Show images
    namedWindow("Original");
    imshow("Original", original);

    namedWindow("left_mask");
    imshow("left_mask", procOut);

    //Wait for 1 millisecond to show images
    waitKey(1);

    //Publish output message
    ROS_DEBUG_STREAM("Publishing message");

    robosub::visionPosArray arrayOut;
    arrayOut.data.push_back(outMsg);
    pub.publish(arrayOut);
}

void rightCamCallback(const wfov_camera_msgs::WFOVImage::ConstPtr& msg)
{
    robosub::visionPosArray outMsg;


    pub.publish(outMsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision");

    ros::NodeHandle n;

                            //Topics reversed for use with current rosbag
    ros::Subscriber leftCamSub =
                         n.subscribe("camera/right/image", 1, leftCamCallback);
    ros::Subscriber rightCamSub =
                         n.subscribe("camera/left/image", 1, rightCamCallback);

    string topic;

    pub =
      n.advertise<robosub::visionPosArray>("vision/output_default" + topic, 1);

    ros::spin();

    return 0;
}
