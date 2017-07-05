#include "ros/ros.h"

ros::Publisher pub;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "imu");

    ros::NodeHandle n;

    ROS_INFO_STREAM("name: " << ros::this_node::getName());
    ROS_INFO_STREAM("namespace: " << ros::this_node::getNamespace());

    ros::spin();

    return 0;
}
