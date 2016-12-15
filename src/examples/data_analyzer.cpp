#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "utility/test_tools.hpp"
#include <signal.h>

rs::SubscriberAnalyzer<std_msgs::Float32> analyzer;

void sighandler(int sig)
{
    analyzer.Stop();
    ROS_INFO("max : %lf", analyzer.GetMax());
    ROS_INFO("min : %lf", analyzer.GetMin());
    ROS_INFO("avg : %lf", analyzer.GetAverage());
    ROS_INFO("std. dev. : %lf", analyzer.GetStandardDeviation());

    ros::shutdown();
}

double data_extractor(const std_msgs::Float32::ConstPtr& msg)
{
    return msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_analyzer_example");
    ros::NodeHandle n;

    signal(SIGINT, sighandler);

    analyzer.Init("data", &data_extractor);
    analyzer.Start();

    ros::spin();
}
