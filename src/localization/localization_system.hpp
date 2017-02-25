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

#include "localization/filter_sensors.h"
#include "localization/lin_accel_kalman_filter.h"
#include "localization/particle_filter.h"

using namespace Eigen;

class LocalizationSystem
{
public:
    LocalizationSystem(FilterSensors *_sensors, ros::NodeHandle _nh, int _num_particles);

    bool resetFilterCallback(std_srvs::Empty::Request &req,
                             std_srvs::Empty::Response &rep);

    geometry_msgs::Vector3Stamped GetLocalizationMessage();

    void Update();

private:
    LinAccelKalmanFilter kf;
    ParticleFilter pf;
    FilterSensors *sensors;

public:
};
#endif //LOCALIZATION_SYSTEM_HPP
