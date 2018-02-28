#include<iostream>
#include "ros/ros.h"
#include "std_msgs/Float32.h"

using namespace std;
ros::Publisher pub;

// TODO finish this program.
int main(int argc, char **argv)
{
    ros::init(argc, argv, "LED");
    ros::NodeHandle n;

    std_msgs::Float32 outmsg;
    outmsg.data = 0.1;
    pub = n.advertise<std_msgs::Float32>("led", 1);
    float change = 0.1;
    while(true)
    {
        for(;outmsg.data != 1.0; outmsg.data += change)
        {
            pub.publish(outmsg);
            sleep(0.2);
        }
        change = -change;
    }
    return 0;
}
