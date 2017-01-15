#include <ros/ros.h>
#include "wfov_camera_msgs/WFOVImage.h"
#include <image_transport/image_transport.h>

image_transport::Publisher leftPub;
image_transport::Publisher rightPub;
image_transport::Publisher bottomPub;

void leftCallback(const wfov_camera_msgs::WFOVImage::ConstPtr &msg)
{
    leftPub.publish(msg->image);    
}

void rightCallback(const wfov_camera_msgs::WFOVImage::ConstPtr &msg)
{
    rightPub.publish(msg->image);
}

void bottomCallback(const wfov_camera_msgs::WFOVImage::ConstPtr &msg)
{
    bottomPub.publish(msg->image);
}

int main(int argv, char** argc)
{
    ros::init(argv, argc, "camera_republisher");

    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    ros::Subscriber rightSub = n.subscribe("camera/right/image", 1,
                                           rightCallback);
    ros::Subscriber leftSub = n.subscribe("camera/left/image", 1, leftCallback);
    ros::Subscriber bottomSub = n.subscribe("camera/bottom/image", 1,
                                            bottomCallback);

    rightPub = it.advertise("camera/right/undistorted", 1);
    leftPub = it.advertise("camera/left/undistorted", 1);
    bottomPub = it.advertise("camera/bottom/undistorted", 1);

    ros::spin();
}
