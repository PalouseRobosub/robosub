#ifndef LOCALIZATION_SYSTEM_HPP
#define LOCALIZATION_SYSTEM_HPP

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>

#include "ros/console.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_srvs/Empty.h"
#include "tf2_msgs/TFMessage.h"

#include "localization/lin_accel_kalman_filter.h"
#include "localization/particle_filter.h"
#include "localization/robosub_sensors.h"
#include "utility/ThrottledPublisher.hpp"

using namespace Eigen;

class LocalizationSystem
{
public:
    LocalizationSystem(ros::NodeHandle &_nh, RobosubSensors &_sensors);

    bool ResetFilterCallback(std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &rep);

    /* pulls position data from the particle filter, as vector message */
    geometry_msgs::Vector3Stamped GetLocalizationMessage();

    /* pulls both position and orientation data from the particle filter, and 
    provides it as pose message */
    geometry_msgs::PoseStamped GetPoseMessage();

    /* pulls position data from the particle filter, as point message */
    geometry_msgs::PointStamped GetLocalizationPoint();

    /* Checks for new data in sensors class, and pushes this data from the
    sensors class into the filters. This in turn causes the filters to update
    their internal state. */
    void Update();

private:
    void publish_tf_message(tf::Vector3 pos);

    /* references to the sensors and nodehandle objects created in
    localization.cpp and passed in via the constructor. */
    RobosubSensors &sensors;
    ros::NodeHandle &nh;

    /* Storage for the kalman filter and particle filter object instances */
    LinAccelKalmanFilter kalman_filter;
    ParticleFilter particle_filter;

    // Publisher for tf data
    rs::ThrottledPublisher<tf2_msgs::TFMessage> transform_pub;

    ros::Time start_time;
};
#endif //LOCALIZATION_SYSTEM_HPP
