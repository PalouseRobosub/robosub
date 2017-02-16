#ifndef LOCALIZATION_SYSTEM_HPP
#define LOCALIZATION_SYSTEM_HPP

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <random>
#include <vector>

#include "ros/console.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"

#include "geometry_msgs/Quaternion.h"
#include "robosub/QuaternionStampedAccuracy.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "robosub/Float32Stamped.h"
#include "robosub/PositionArrayStamped.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32.h"
#include "std_srvs/Empty.h"

#include "localization/particle_filter.h"

using namespace Eigen;

class LocalizationSystem
{
public:
    LocalizationSystem(int _num_particles);

    bool resetFilterCallback(std_srvs::Empty::Request &req,
                             std_srvs::Empty::Response &rep);
    void depthCallback(const robosub::Float32Stamped::ConstPtr &msg);
    void hydrophoneCallback(const robosub::PositionArrayStamped::ConstPtr &msg);
    void linAccelCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
    void orientationCallback(const robosub::QuaternionStampedAccuracy::ConstPtr
                             &msg);

    geometry_msgs::Vector3Stamped GetLocalizationMessage();

private:
    tf::Vector3 calculate_absolute_lin_accel(tf::Vector3 rel_lin_accel);

    ParticleFilter pf;

    ros::Time last_lin_accel_timestamp;
    ros::Duration dt;

    tf::Quaternion orientation;

public:
};
#endif //LOCALIZATION_SYSTEM_HPP
