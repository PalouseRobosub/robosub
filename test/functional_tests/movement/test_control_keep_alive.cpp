#include "ros/ros.h"
#include "robosub_msgs/control.h"

ros::Publisher pub;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "control_system_keep_alive");
    ros::NodeHandle n;

    pub = n.advertise<robosub_msgs::control>("control", 1);

    robosub_msgs::control msg;

    // fill out empty control message
    msg.forward_state = robosub_msgs::control::STATE_NONE;
    msg.strafe_state = robosub_msgs::control::STATE_NONE;
    msg.yaw_state = robosub_msgs::control::STATE_NONE;
    msg.dive_state = robosub_msgs::control::STATE_NONE;
    msg.pitch_state = robosub_msgs::control::STATE_NONE;
    msg.roll_state = robosub_msgs::control::STATE_NONE;

    ros::Rate r(3);

    while (ros::ok())
    {
        pub.publish(msg);
        r.sleep();
    }

    return 0;
}
