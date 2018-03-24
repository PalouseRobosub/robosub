#ifndef ROBOSUB_SENSORS_H
#define ROBOSUB_SENSORS_H

#include "ros/console.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"

#include "geometry_msgs/QuaternionStamped.h"
#include "robosub_msgs/Float32Stamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/AccelStamped.h"

// RobosubSensors is responsible for storing and handling all sensor input
// (both from callbacks and derived info like linear velocity). It provides the
// following functions for each set of sensor data:
// Input Data
// Get Data
// Is New Data
// Get DT
// All data is stored as tf datatypes or primitive datatypes.

// Additionally RobosubSensors provides one other function: Calculating
// absolute linear acceleration whenever the relative linear acceleration
// callback occurs.
class RobosubSensors
{
public:
    RobosubSensors();

    // Sensor callbacks
    // DT is calculated using the msg stamp and last receive time
    void InputRelLinAcl(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
    void InputDepth(const robosub_msgs::Float32Stamped::ConstPtr &msg);
    void InputHydrophones(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
    void InputOrientation(const geometry_msgs::QuaternionStamped::ConstPtr
            &msg);

    //should read any arbitrary acceleration message passed to it
    void InputAccelMsg(const geometry_msgs::AccelStamped::ConstPtr &msg);

    // Inputs for derived data
    // Since we have no stamp for these inputs, DT is calculated using the time
    // received and the last received time
    void InputAbsLinAcl(const tf::Vector3 lin_acl);
    void InputAbsLinVel(const tf::Vector3 lin_vel);
    void InputPosition(const tf::Vector3 pos);

    // Getter functions.
    // These set the new field to false when called.
    tf::Vector3 GetRelLinAcl();
    double GetDepth();
    tf::Vector3 GetHydrophones();
    tf::Quaternion GetOrientation();
    tf::Vector3 GetAbsLinAcl();
    tf::Vector3 GetAbsLinVel();
    tf::Vector3 GetPosition();

    // Get new status
    bool NewRelLinAcl();
    bool NewDepth();
    bool NewHydrophones();
    bool NewOrientation();
    bool NewAbsLinAcl();
    bool NewAbsLinVel();
    bool NewPosition();

    // Get DTs
    double GetRelLinAclDT();
    double GetDepthDT();
    double GetHydrophonesDT();
    double GetOrientationDT();
    double GetAbsLinAclDT();
    double GetAbsLinVelDT();
    double GetPositionDT();

private:
    void calculate_absolute_lin_accel();

    // Stores whether sensor is new. Set to false when a particular sensors Get
    // method is called.
    bool new_rel_lin_acl;
    bool new_depth;
    bool new_hydrophones;
    bool new_orientation;
    bool new_abs_lin_acl;
    bool new_abs_lin_vel;
    bool new_position;

    // Stores last receive time. Comes from message stamp if possible.
    ros::Time last_rel_lin_acl_time;
    ros::Time last_depth_time;
    ros::Time last_hydrophones_time;
    ros::Time last_orientation_time;
    ros::Time last_abs_lin_acl_time;
    ros::Time last_abs_lin_vel_time;
    ros::Time last_position_time;

    // Stores dt from last sensor input
    double rel_lin_acl_dt;
    double depth_dt;
    double hydrophones_dt;
    double orientation_dt;
    double abs_lin_acl_dt;
    double abs_lin_vel_dt;
    double position_dt;

    // Stores the actual sensor data
    tf::Vector3 rel_lin_acl;
    double depth;
    tf::Vector3 hydrophones;
    tf::Quaternion orientation;
    tf::Vector3 abs_lin_acl;
    tf::Vector3 abs_lin_vel;
    tf::Vector3 position;
};

#endif // ROBOSUB_SENSORS_H
