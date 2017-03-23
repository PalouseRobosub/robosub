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

#include "localization/robosub_sensors.h"
#include "localization/lin_accel_kalman_filter.h"
#include "localization/particle_filter.h"

using namespace Eigen;

class LocalizationSystem
{
public:
    LocalizationSystem(ros::NodeHandle *_nh, RobosubSensors *_sensors);

    bool ResetFilterCallback(std_srvs::Empty::Request &req,
                             std_srvs::Empty::Response &rep);

    geometry_msgs::Vector3Stamped GetLocalizationMessage();
    geometry_msgs::PoseStamped GetPoseMessage();

    void Update();

private:
    void publish_tf_message(tf::Vector3 pos);

    // Filter objects
    LinAccelKalmanFilter kf;
    ParticleFilter pf;

    // Publisher for tf data
    ros::Publisher tf_pub;

    // Objects inputted from localization main
    RobosubSensors *sensors;
    ros::NodeHandle *nh;

public:
};
#endif //LOCALIZATION_SYSTEM_HPP
