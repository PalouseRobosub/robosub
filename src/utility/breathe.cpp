#include<iostream>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include<signal.h>

using namespace std;
ros::Publisher pub;

void handler(int s)
{
    std_msgs::Float32 outmsg;
    ros::NodeHandle n;
    pub = n.advertise<std_msgs::Float32>("led", 1);
    outmsg.data = 0.1;
    pub.publish(outmsg);
    exit(1);
}

// TODO finish this program.
int main(int argc, char **argv)
{
    ros::init(argc, argv, "LED");
    ros::NodeHandle n;

    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    std_msgs::Float32 outmsg;
    outmsg.data = 0.1;
    pub = n.advertise<std_msgs::Float32>("led", 1);
    float change = 0.1;
    while(true)
    {
        for(;outmsg.data != 1.0; outmsg.data += change)
        {
            pub.publish(outmsg);
            sleep(1);
        }
        change = -change;
    }
    return 0;
}
