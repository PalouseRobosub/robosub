#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "robosub_msgs/Euler.h"
#include "ros/ros.h"
#include <string>
#include "tf/transform_datatypes.h"

#define _180_OVER_PI (180.0 / 3.14159)

ros::Publisher quaternion_publisher;
ros::Publisher euler_publisher;
ros::Publisher acceleration_publisher;

robosub_msgs::Euler quaternion_to_euler(
                         const geometry_msgs::QuaternionStamped::ConstPtr &msg)
{
    double roll, pitch, yaw;
    tf::Quaternion tf_quaternion;
    robosub_msgs::Euler euler_msg;

    tf::quaternionMsgToTF(msg->quaternion, tf_quaternion);

    tf::Matrix3x3 m(tf_quaternion);
    m.getRPY(roll, pitch, yaw);

    euler_msg.roll = roll * _180_OVER_PI;
    euler_msg.pitch = pitch * _180_OVER_PI;
    euler_msg.yaw = yaw * _180_OVER_PI;

    return euler_msg;
}

void orientation_callback(const geometry_msgs::QuaternionStamped::ConstPtr &msg)
{
    quaternion_publisher.publish(msg);
    euler_publisher.publish(quaternion_to_euler(msg));
}

void acceleration_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    acceleration_publisher.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu");

    ros::NodeHandle n;

    std::string active_imu;
    if(ros::param::get("~active_imu", active_imu) != true)
    {
        ROS_FATAL("~active_imu param not set.");
        return 1;
    }

    quaternion_publisher =
        n.advertise<geometry_msgs::QuaternionStamped>("orientation", 1);
    euler_publisher = n.advertise<robosub_msgs::Euler>("pretty/orientation", 1);
    acceleration_publisher =
        n.advertise<geometry_msgs::Vector3Stamped>("acceleration/linear", 1);

    ros::Subscriber orientation_sub =
        n.subscribe(active_imu + "/orientation", 1, orientation_callback);

    ros::Subscriber acceleration_sub =
        n.subscribe(active_imu + "/acceleration/linear", 1,
        acceleration_callback);

    ros::spin();

    return 0;
}
