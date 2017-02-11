#include <eigen3/Eigen/Dense>
#include "obstacle_map.h"

using namespace Eigen;

tf::Vector3 sub_position;

void position_real_callback(const geometry_msgs::Vector3::ConstPtr &msg)
{
    sub_position[0] = msg->x;
    sub_position[1] = msg->y;
    sub_position[2] = msg->z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam");

    ros::NodeHandle nh;

    sub_position = tf::Vector3(0, 0, 0);

    /*
    geometry_msgs::Vector3Stamped
    ros::Subscriber accel_sub = nh.subscribe("acceleration/linear", 1,

    robosub::QuaternionStampedAccuracy
    ros::Subscriber orientation_sub = nh.subscribe("orientation", 1,

    robosub::Float32Stamped
    ros::Subscriber depth_sub = nh.subscribe("depth", 1,

    robosub::PositionArrayStamped
    ros::Subscriber hydrophones_position_sub = nh.subscribe("hydrophones/position", 1,
    */

    ros::Subscriber position_real_sub = nh.subscribe("position/real", 1, &position_real_callback);

    robosub::ObstacleMap map(&nh);

    ros::Rate r(2.0);

    while(ros::ok())
    {
        ros::spinOnce();

        ROS_INFO("sub position : <%f, %f, %f>", sub_position[0], sub_position[1], sub_position[2]);
        ROS_INFO("closest obstacle: %s\n", map.GetClosestToSub(sub_position).c_str());

        r.sleep();
    }

    return 0;
}
