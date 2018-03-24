#include "localization/robosub_sensors.h"
#include <tf/transform_datatypes.h>

RobosubSensors::RobosubSensors() :
    new_rel_lin_acl(false),
    new_depth(false),
    new_hydrophones(false),
    new_orientation(false),
    new_abs_lin_vel(false),
    new_position(false),
    last_rel_lin_acl_time(ros::Time::now()),
    last_depth_time(ros::Time::now()),
    last_hydrophones_time(ros::Time::now()),
    last_orientation_time(ros::Time::now()),
    last_abs_lin_vel_time(ros::Time::now()),
    last_position_time(ros::Time::now()),
    rel_lin_acl_dt(0.0),
    depth_dt(0.0),
    hydrophones_dt(0.0),
    orientation_dt(0.0),
    abs_lin_vel_dt(0.0),
    position_dt(0.0),
    rel_lin_acl(0.0, 0.0, 0.0),
    depth(0.0),
    hydrophones(0.0, 0.0, 0.0),
    orientation(0.0, 0.0, 0.0, 1.0),
    abs_lin_acl(0.0, 0.0, 0.0),
    abs_lin_vel(0.0, 0.0, 0.0),
    position(0.0, 0.0, 0.0)
{
}

void RobosubSensors::InputRelLinAcl(
        const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    rel_lin_acl = tf::Vector3(msg->vector.x, msg->vector.y, msg->vector.z);
    rel_lin_acl_dt = (msg->header.stamp - last_rel_lin_acl_time).toSec();
    new_rel_lin_acl = true;
    last_rel_lin_acl_time = msg->header.stamp;

    calculate_absolute_lin_accel();
}

void RobosubSensors::InputAccelMsg(const geometry_msgs::Accel::ConstPtr &msg)
{
    rel_lin_acl = tf::Vector3(msg->linear.x, msg->linear.y, msg->linear.z);
    rel_lin_acl_dt = (ros::Time::now() - last_rel_lin_acl_time).toSec();
    new_rel_lin_acl = true;
    last_rel_lin_acl_time = ros::Time::now();

    calculate_absolute_lin_accel();
}

void RobosubSensors::InputDepth(
        const robosub_msgs::Float32Stamped::ConstPtr &msg)
{
    depth = msg->data;
    depth_dt = (msg->header.stamp - last_depth_time).toSec();
    new_depth = true;
    last_depth_time = msg->header.stamp;
}

void RobosubSensors::InputHydrophones(const
        geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    tf::vector3MsgToTF(msg->vector, hydrophones);
    hydrophones_dt = (msg->header.stamp - last_hydrophones_time).toSec();
    new_hydrophones = true;
    last_hydrophones_time = msg->header.stamp;
}

void RobosubSensors::InputOrientation(const
        geometry_msgs::QuaternionStamped::ConstPtr &msg)
{
    orientation = tf::Quaternion(msg->quaternion.x, msg->quaternion.y,
                                 msg->quaternion.z, msg->quaternion.w);
    orientation_dt = (msg->header.stamp - last_orientation_time).toSec();
    new_orientation = true;
    last_orientation_time = msg->header.stamp;
}

void RobosubSensors::InputAbsLinAcl(const tf::Vector3 lin_acl)
{
    ros::Time stamp = ros::Time::now();
    abs_lin_acl = lin_acl;
    abs_lin_acl_dt = (stamp - last_abs_lin_acl_time).toSec();
    new_abs_lin_acl = true;
    last_abs_lin_acl_time = stamp;
}

void RobosubSensors::InputAbsLinVel(const tf::Vector3 lin_vel)
{
    ros::Time stamp = ros::Time::now();
    abs_lin_vel = lin_vel;
    abs_lin_vel_dt = (stamp - last_abs_lin_vel_time).toSec();
    new_abs_lin_vel = true;
    last_abs_lin_vel_time = stamp;
}

void RobosubSensors::InputPosition(const tf::Vector3 pos)
{
    ros::Time stamp = ros::Time::now();
    position = pos;
    position_dt = (stamp - last_position_time).toSec();
    new_position = true;
    last_position_time = stamp;
}

tf::Vector3 RobosubSensors::GetRelLinAcl()
{
    new_rel_lin_acl = false;
    return rel_lin_acl;
}

double RobosubSensors::GetDepth()
{
    new_depth = false;
    return depth;
}

tf::Vector3 RobosubSensors::GetHydrophones()
{
    new_hydrophones = false;
    return hydrophones;
}

tf::Quaternion RobosubSensors::GetOrientation()
{
    new_orientation = false;
    return orientation;
}

tf::Vector3 RobosubSensors::GetAbsLinAcl()
{
    new_abs_lin_acl = false;
    return abs_lin_acl;
}

tf::Vector3 RobosubSensors::GetAbsLinVel()
{
    new_abs_lin_vel = false;
    return abs_lin_vel;
}

tf::Vector3 RobosubSensors::GetPosition()
{
    new_position = false;
    return position;
}

bool RobosubSensors::NewRelLinAcl()
{
    return new_rel_lin_acl;
}

bool RobosubSensors::NewDepth()
{
    return new_depth;
}

bool RobosubSensors::NewHydrophones()
{
    return new_hydrophones;
}

bool RobosubSensors::NewOrientation()
{
    return new_orientation;
}

bool RobosubSensors::NewAbsLinAcl()
{
    return new_abs_lin_acl;
}

bool RobosubSensors::NewAbsLinVel()
{
    return new_abs_lin_vel;
}

bool RobosubSensors::NewPosition()
{
    return new_position;
}

double RobosubSensors::GetRelLinAclDT()
{
    return rel_lin_acl_dt;
}

double RobosubSensors::GetDepthDT()
{
    return depth_dt;
}

double RobosubSensors::GetHydrophonesDT()
{
    return hydrophones_dt;
}

double RobosubSensors::GetOrientationDT()
{
    return orientation_dt;
}

double RobosubSensors::GetAbsLinAclDT()
{
    return abs_lin_acl_dt;
}

double RobosubSensors::GetAbsLinVelDT()
{
    return abs_lin_vel_dt;
}

double RobosubSensors::GetPositionDT()
{
    return position_dt;
}

void RobosubSensors::calculate_absolute_lin_accel()
{
    tf::Quaternion orientation_conjugate;
    orientation_conjugate[0] = orientation[0] * -1.0;
    orientation_conjugate[1] = orientation[1] * -1.0;
    orientation_conjugate[2] = orientation[2] * -1.0;
    orientation_conjugate[3] = orientation[3];

    tf::Matrix3x3 rot_m = tf::Matrix3x3(orientation_conjugate);

    InputAbsLinAcl(rot_m * rel_lin_acl);
}
