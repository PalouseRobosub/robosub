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

    cv::namedWindow("left_mask");
    cv::imshow("left_mask", processed);

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
