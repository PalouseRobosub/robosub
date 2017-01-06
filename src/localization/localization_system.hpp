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
#include "robosub/depth_stamped.h"
#include "robosub/PositionArrayStamped.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32.h"
#include "std_srvs/Empty.h"

#include "particle_filter.h"

using namespace Eigen;

class LocalizationSystem
{
public:
    LocalizationSystem(int _num_particles);

    bool resetFilterCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &rep);
    void depthCallback(const robosub::depth_stamped::ConstPtr &msg);
    void hydrophoneCallback(const robosub::PositionArrayStamped::ConstPtr &msg);
    void linAccelCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
    void orientationCallback(const robosub::QuaternionStampedAccuracy::ConstPtr &msg);

    void calculate_absolute_lin_accel();
    void Update();

    geometry_msgs::Vector3Stamped GetLocalizationMessage();

private:
    ParticleFilter pf;

    bool new_hydrophone;
    bool new_depth;
    bool new_lin_velocity;

    ros::Time last_lin_accel_receive_time;
    ros::Duration dt;

    tf::Quaternion orientation;
    tf::Vector3 abs_lin_accel;
    tf::Vector3 rel_lin_accel;

public:
};
