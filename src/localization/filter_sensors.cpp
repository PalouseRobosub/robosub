#include "filter_sensors.h"

FilterSensors::FilterSensors()
{
    new_rel_lin_acl = false;
    new_depth = false;
    new_hydrophones = false;
    new_orientation = false;
    new_abs_lin_vel = false;
    new_position = false;

    last_rel_lin_acl_time = ros::Time::now();
    last_depth_time = ros::Time::now();
    last_hydrophones_time = ros::Time::now();
    last_orientation_time = ros::Time::now();
    last_abs_lin_vel_time = ros::Time::now();
    last_position_time = ros::Time::now();

    rel_lin_acl_dt = 0.0;
    depth_dt = 0.0;
    hydrophones_dt = 0.0;
    orientation_dt = 0.0;
    abs_lin_vel_dt = 0.0;
    position_dt = 0.0;

    rel_lin_acl[0] = rel_lin_acl[1] = rel_lin_acl[2] = 0.0;
    depth = 0.0;
    hydrophones[0] = hydrophones[1] = hydrophones[2] = 0.0;
    orientation[0] = orientation[1] = orientation[2] = 0.0;
    orientation[3] = 1.0;
    abs_lin_acl[0] = abs_lin_acl[1] = abs_lin_acl[2] = 0.0;
    abs_lin_vel[0] = abs_lin_vel[1] = abs_lin_vel[2] = 0.0;
    position[0] = position[1] = position[2] = 0.0;
}

void FilterSensors::InputRelLinAcl(const
                                   geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    rel_lin_acl = tf::Vector3(msg->vector.x, msg->vector.y, msg->vector.z);
    rel_lin_acl_dt = (msg->header.stamp - last_rel_lin_acl_time).toSec();
    new_rel_lin_acl = true;
    last_rel_lin_acl_time = msg->header.stamp;

    calculate_absolute_lin_accel();
}

void FilterSensors::InputDepth(const robosub::Float32Stamped::ConstPtr &msg)
{
    depth = msg->data;
    depth_dt = (msg->header.stamp - last_depth_time).toSec();
    new_depth = true;
    last_depth_time = msg->header.stamp;
}

void FilterSensors::InputHydrophones(const
                                     robosub::PositionArrayStamped::ConstPtr &msg)
{
    hydrophones = tf::Vector3(msg->positions[0].position.x,
                              msg->positions[0].position.y, msg->positions[0].position.z);
    hydrophones_dt = (msg->header.stamp - last_hydrophones_time).toSec();
    new_hydrophones = true;
    last_hydrophones_time = msg->header.stamp;
}

void FilterSensors::InputOrientation(const
                                     robosub::QuaternionStampedAccuracy::ConstPtr &msg)
{
    orientation = tf::Quaternion(msg->quaternion.x, msg->quaternion.y,
                                 msg->quaternion.z, msg->quaternion.w);
    orientation_dt = (msg->header.stamp - last_orientation_time).toSec();
    new_orientation = true;
    last_orientation_time = msg->header.stamp;
}

void FilterSensors::InputAbsLinAcl(const tf::Vector3 lin_acl)
{
    ros::Time stamp = ros::Time::now();
    abs_lin_acl = lin_acl;
    abs_lin_acl_dt = (stamp - last_abs_lin_acl_time).toSec();
    new_abs_lin_acl = true;
    last_abs_lin_acl_time = stamp;
}

void FilterSensors::InputAbsLinVel(const tf::Vector3 lin_vel)
{
    ros::Time stamp = ros::Time::now();
    abs_lin_vel = lin_vel;
    abs_lin_vel_dt = (stamp - last_abs_lin_vel_time).toSec();
    new_abs_lin_vel = true;
    last_abs_lin_vel_time = stamp;
}

void FilterSensors::InputPosition(const tf::Vector3 pos)
{
    ros::Time stamp = ros::Time::now();
    position = pos;
    position_dt = (stamp - last_position_time).toSec();
    new_position = true;
    last_position_time = stamp;
}

tf::Vector3 FilterSensors::GetRelLinAcl()
{
    new_rel_lin_acl = false;
    return rel_lin_acl;
}

double FilterSensors::GetDepth()
{
    new_depth = false;
    return depth;
}

tf::Vector3 FilterSensors::GetHydrophones()
{
    new_hydrophones = false;
    return hydrophones;
}

tf::Quaternion FilterSensors::GetOrientation()
{
    new_orientation = false;
    return orientation;
}

tf::Vector3 FilterSensors::GetAbsLinAcl()
{
    new_abs_lin_acl = false;
    return abs_lin_acl;
}

tf::Vector3 FilterSensors::GetAbsLinVel()
{
    new_abs_lin_vel = false;
    return abs_lin_vel;
}

tf::Vector3 FilterSensors::GetPosition()
{
    new_position = false;
    return position;
}

bool FilterSensors::NewRelLinAcl()
{
    return new_rel_lin_acl;
}

bool FilterSensors::NewDepth()
{
    return new_depth;
}

bool FilterSensors::NewHydrophones()
{
    return new_hydrophones;
}

bool FilterSensors::NewOrientation()
{
    return new_orientation;
}

bool FilterSensors::NewAbsLinAcl()
{
    return new_abs_lin_acl;
}

bool FilterSensors::NewAbsLinVel()
{
    return new_abs_lin_vel;
}

bool FilterSensors::NewPosition()
{
    return new_position;
}

double FilterSensors::GetRelLinAclDT()
{
    return rel_lin_acl_dt;
}

double FilterSensors::GetDepthDT()
{
    return depth_dt;
}

double FilterSensors::GetHydrophonesDT()
{
    return hydrophones_dt;
}

double FilterSensors::GetOrientationDT()
{
    return orientation_dt;
}

double FilterSensors::GetAbsLinAclDT()
{
    return abs_lin_acl_dt;
}

double FilterSensors::GetAbsLinVelDT()
{
    return abs_lin_vel_dt;
}

double FilterSensors::GetPositionDT()
{
    return position_dt;
}

void FilterSensors::calculate_absolute_lin_accel()
{
    tf::Quaternion orientation_conjugate;
    orientation_conjugate[0] = orientation[0] * -1.0;
    orientation_conjugate[1] = orientation[1] * -1.0;
    orientation_conjugate[2] = orientation[2] * -1.0;
    orientation_conjugate[3] = orientation[3];

    tf::Matrix3x3 rot_m = tf::Matrix3x3(orientation_conjugate);

    InputAbsLinAcl(rot_m * rel_lin_acl);
}
