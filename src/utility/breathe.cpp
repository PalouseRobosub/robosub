#include<iostream>
#include "ros/ros.h"
#include "std_msgs/Float32.h"

using namespace std;


// TODO finish this program.
int main(int argc, char **argv)
{
    ros::init(argc, argv, "LED");
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Rate loop_rate(1);

    std_msgs::Float32 outmsg;
    outmsg.data = 0.1;
    pub = n.advertise<std_msgs::Float32>("led", 1);
    float change = 0.1;
    while(true)
    {
        for(;outmsg.data < 1.0; outmsg.data += change)
        {
            pub.publish(outmsg);
            ros::spinOnce();
            loop_rate.sleep();
        }
        change = -change;
    }
    return 0;
}
