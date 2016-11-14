#include "ros/ros.h"
#include "wfov_camera_msgs/WFOVImage.h"
#include "robosub/visionPos.h"
#include "VisionProcessor.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

ros::Publisher pub;

void leftCamCallback(const wfov_camera_msgs::WFOVImage::ConstPtr& msg)
{

    std::cout << "Left Camera Callback" << std::endl;
    robosub::visionPos outMsg;

    VisionProcessor vp;
    
    sensor_msgs::Image imgCopy = msg->image;

    std::cout << "Processing" << std::endl;
    cv::Mat processed = vp.process(imgCopy);

    cv::Mat procOut = processed.clone();

    cv::Mat original = cv_bridge::toCvShare(msg->image, msg, sensor_msgs::image_encodings::BGR8)->image;

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    findContours(processed, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0)); 

    cv::Moments moments;
    moments = cv::moments(contours[0], false);

   cv::Point2f center = cv::Point2f(moments.m10/moments.m00, moments.m01/moments.m00);

    std::cout << "Center at: " << center << std::endl;

    cv::circle(original, center, 3, cv::Scalar(0,0,255), -1);
    cv::circle(procOut, center, 3, cv::Scalar(0,0,255), -1);

    cv::namedWindow("Original");
    cv::imshow("Original", original);

    cv::namedWindow("left_mask");
    cv::imshow("left_mask", procOut);

    cv::waitKey(1);

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
