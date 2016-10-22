#include "ros/ros.h"
#include "robosub/Euler.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

#define _180_OVER_PI (180.0 / 3.1415 )

ros::Publisher pub;

void orientationCallback(const geometry_msgs::Quaternion::ConstPtr& quat_msg)
{
    // Quaternion to roll pitch yaw
    // This is apparently the best way to do it with built in ros stuff
    tf::Quaternion q(quat_msg->x, quat_msg->y, quat_msg->z, quat_msg->w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    //ROS_INFO_STREAM("Roll(deg): " << roll * _180_OVER_PI);
    //ROS_INFO_STREAM("Pitch(deg): " << pitch * _180_OVER_PI);
    //ROS_INFO_STREAM("Yaw(deg): " << yaw * _180_OVER_PI);
    //ROS_INFO_STREAM("\n");

    robosub::Euler outmsg;

    outmsg.yaw = yaw * _180_OVER_PI;
    outmsg.roll = roll * _180_OVER_PI;
    outmsg.pitch = pitch * _180_OVER_PI;

    pub.publish(outmsg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "print_orientation");

    ros::NodeHandle nh;
    pub = nh.advertise<robosub::Euler>("euler", 1);

	ros::Subscriber orient_sub = nh.subscribe("orientation", 1, orientationCallback);

    ros::spin();

    return 0;
}
