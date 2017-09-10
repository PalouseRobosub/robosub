#include "ros/ros.h"
#include "robosub/control.h"

ros::Publisher pub;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "control_system_keep_alive");
    ros::NodeHandle n;

    pub = n.advertise<robosub::control>("control", 1);

    robosub::control msg;

    // fill out empty control message
    msg.forward_state = robosub::control::STATE_NONE;
    msg.strafe_state = robosub::control::STATE_NONE;
    msg.yaw_state = robosub::control::STATE_NONE;
    msg.dive_state = robosub::control::STATE_NONE;
    msg.pitch_state = robosub::control::STATE_NONE;
    msg.roll_state = robosub::control::STATE_NONE;

    ros::Rate r(3);

    while (ros::ok())
    {
        pub.publish(msg);
        r.sleep();
    }

    return 0;
}
