#ifndef FILTER_SENSORS_H
#define FILTER_SENSORS_H

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <utility>
#include <random>
#include <string>
#include <vector>

#include "ros/console.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"

#include "robosub/QuaternionStampedAccuracy.h"
#include "robosub/Float32ArrayStamped.h"
#include "robosub/Float32Stamped.h"
#include "robosub/PositionArrayStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_srvs/Empty.h"

class FilterSensors
{
public:
    FilterSensors();

    void InputRelLinAcl(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
    void InputDepth(const robosub::Float32Stamped::ConstPtr &msg);
    void InputHydrophones(const robosub::PositionArrayStamped::ConstPtr &msg);
    void InputOrientation(const robosub::QuaternionStampedAccuracy::ConstPtr
                          &msg);

    void InputAbsLinAcl(const tf::Vector3 lin_acl);
    void InputAbsLinVel(const tf::Vector3 lin_vel);
    void InputPosition(const tf::Vector3 pos);

    tf::Vector3 GetRelLinAcl();
    double GetDepth();
    tf::Vector3 GetHydrophones();
    tf::Quaternion GetOrientation();
    tf::Vector3 GetAbsLinAcl();
    tf::Vector3 GetAbsLinVel();
    tf::Vector3 GetPosition();

    bool NewRelLinAcl();
    bool NewDepth();
    bool NewHydrophones();
    bool NewOrientation();
    bool NewAbsLinAcl();
    bool NewAbsLinVel();
    bool NewPosition();

    double GetRelLinAclDT();
    double GetDepthDT();
    double GetHydrophonesDT();
    double GetOrientationDT();
    double GetAbsLinAclDT();
    double GetAbsLinVelDT();
    double GetPositionDT();

private:
    bool new_rel_lin_acl;
    bool new_depth;
    bool new_hydrophones;
    bool new_orientation;
    bool new_abs_lin_acl;
    bool new_abs_lin_vel;
    bool new_position;

    ros::Time last_rel_lin_acl_time;
    ros::Time last_depth_time;
    ros::Time last_hydrophones_time;
    ros::Time last_orientation_time;
    ros::Time last_abs_lin_acl_time;
    ros::Time last_abs_lin_vel_time;
    ros::Time last_position_time;

    double rel_lin_acl_dt;
    double depth_dt;
    double hydrophones_dt;
    double orientation_dt;
    double abs_lin_acl_dt;
    double abs_lin_vel_dt;
    double position_dt;

    tf::Vector3 rel_lin_acl;
    double depth;
    tf::Vector3 hydrophones;
    tf::Quaternion orientation;
    tf::Vector3 abs_lin_acl;
    tf::Vector3 abs_lin_vel;
    tf::Vector3 position;

    void calculate_absolute_lin_accel();
};

#endif
