#include "ros/ros.h"
#include "wfov_camera_msgs/WFOVImage.h"
#include "robosub/visionPos.h"
#include "VisionProcessor.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

ros::Publisher pub;

void leftCamCallback(const wfov_camera_msgs::WFOVImage::ConstPtr& msg)
{
    //Create a vision processor
    VisionProcessor vp;
    
    //Copy the image for processing
    //TODO: This should be optimized to avoid copying by sharing the pointer
    sensor_msgs::Image imgCopy = msg->image;

    //Process the image using the VisionProcessor
    cv::Mat processed = vp.process(imgCopy);

    //Clone the output image for showing
    cv::Mat procOut = processed.clone();

    //Create a Mat of the original image for showing
    cv::Mat original = cv_bridge::toCvShare(msg->image, msg, sensor_msgs::image_encodings::BGR8)->image;

    //Find contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    findContours(processed, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0)); 

    //Create the output message
    robosub::visionPos outMsg;

    //If there are no contours, no calculations needed
    if (contours.size() >= 1)
    {
        //Find the area of the first contour
        double largestArea = cv::contourArea(contours[0], false);
        int largestIndex = 0;
        
        //Compare the first contour to other areas to find contour with the largest area
        for (int i = 1; i < contours.size(); i++)
        {
            double area = cv::contourArea(contours[i], false);
            if (area > largestArea)
            {
                largestArea = area;
                largestIndex = i;
            }
        }

        //Find the moments (physical properties) of the contour
        cv::Moments moments;
        moments = cv::moments(contours[largestIndex], false);

        int cx = -1;
        int cy = -1;
        int imWidth = msg->image.width;
        int imHeight = msg->image.height;
        //Using moments, find the center point of the contour
        if (moments.m00 > 0)
        {
            //Find x and y coordinates with 0,0 being top left corner
            cx = (int)(moments.m10/moments.m00);
            cy = (int)(moments.m01/moments.m00);
            cv::Point2f center = cv::Point2f(cx,cy);
            std::cout << "Center at: " << "[" << cx - (imWidth/2) << "," << -1*(cy-(imHeight / 2)) << "]" << std::endl;
            cv::circle(original, center, 4, cv::Scalar(0,0,255), -1); //Draw a circle on the original image for location visualization
        }

        //Prepare the output message
        outMsg.xPos = cx - (imWidth / 2);
        outMsg.yPos = cy - (imHeight / 2);
        outMsg.magnitude = (double)largestArea / (double)(imWidth * imHeight);

    }
    
    //Show images
    cv::namedWindow("Original");
    cv::imshow("Original", original);

    cv::namedWindow("left_mask");
    cv::imshow("left_mask", procOut);

    //Wait for 1 millisecond to show images
    cv::waitKey(1);
    
    //Publish output message
    pub.publish(outMsg);
}

void rightCamCallback(const wfov_camera_msgs::WFOVImage::ConstPtr& msg)
{
    robosub::visionPos outMsg;


    pub.publish(outMsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision");

    ros::NodeHandle n;

    ros::Subscriber leftCamSub = n.subscribe("/camera/right/image", 1, leftCamCallback);
    ros::Subscriber rightCamSub = n.subscribe("/camera/left/image", 1, rightCamCallback);

    pub = n.advertise<robosub::visionPos>("/vision/buoy/red", 1);

    ros::spin();

    return 0;
}
