#include "ros/ros.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "robosub/Euler.h"
#include "tf/transform_datatypes.h"

ros::Publisher quaternion_publisher;
ros::Publisher euler_publisher;

robosub::Euler quaternion_to_euler(
                         const geometry_msgs::QuaternionStamped::ConstPtr &msg)
{
    double roll, pitch, yaw;
    tf::Quaternion tf_quaternion;
    robosub::Euler euler_msg;

    tf::quaternionMsgToTF(msg->quaternion, tf_quaternion);

    tf::Matrix3x3 m(tf_quaternion);
    m.getRPY(roll, pitch, yaw);

    euler_msg.roll = roll;
    euler_msg.pitch = pitch;
    euler_msg.yaw = yaw;

    return euler_msg;
}

void orientation_callback(const geometry_msgs::QuaternionStamped::ConstPtr &msg)
{
    quaternion_publisher.publish(msg);
    euler_publisher.publish(quaternion_to_euler(msg));
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
    euler_publisher = n.advertise<robosub::Euler>("pretty/orientation", 1);

    ros::Subscriber sub = n.subscribe(active_imu + "/orientation", 1,
                                      orientation_callback);

    ros::spin();

    return 0;
}
