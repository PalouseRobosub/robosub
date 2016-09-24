#include "ros/ros.h"
#include "utility/serial.hpp"
#include "robosub/thruster.h"

using namespace rs;

void Callback (const robosub::thruster::ConstPtr& msg)
{
    robosub::thruster message = *msg;
    for (int i = 0; i < message.data.size(); i++)
        ROS_INFO("%lf", message.data[i]);
}

int main(int argc, char **argv)
{
    Serial s;
    ros::init(argc, argv, "thruster");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("thruster", 1, Callback);
    ros::spin();
    return 0;
}
