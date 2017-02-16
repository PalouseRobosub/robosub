#include "localization_system.hpp"

LocalizationSystem::LocalizationSystem(int _num_particles) : pf(_num_particles)
{
    orientation[0] = 0.0;
    orientation[1] = 0.0;
    orientation[2] = 0.0;
    orientation[3] = 1.0;

    last_lin_accel_timestamp = ros::Time::now();
}

geometry_msgs::Vector3Stamped LocalizationSystem::GetLocalizationMessage()
{
    geometry_msgs::Vector3Stamped s;

    tf::Vector3 pos = pf.GetPosition();

    s.vector.x = pos[0];
    s.vector.y = pos[1];
    s.vector.z = pos[2];
    s.header.stamp = ros::Time::now();

    return s;
}

tf::Vector3 LocalizationSystem::calculate_absolute_lin_accel(tf::Vector3 rel_lin_accel)
{
    tf::Quaternion orientation_conjugate;
    orientation_conjugate[0] = orientation[0] * -1.0;
    orientation_conjugate[1] = orientation[1] * -1.0;
    orientation_conjugate[2] = orientation[2] * -1.0;
    orientation_conjugate[3] = orientation[3];

    tf::Matrix3x3 rot_m = tf::Matrix3x3(orientation_conjugate);

    return rot_m * rel_lin_accel;
}

void LocalizationSystem::depthCallback(const robosub::Float32Stamped::ConstPtr
                                       &msg)
{
    pf.InputDepth(msg->data, msg->header.stamp);
}

void LocalizationSystem::hydrophoneCallback(const
        robosub::PositionArrayStamped::ConstPtr &msg)
{
    if(msg->positions.size() > 0)
    {
        tf::Vector3 pos(msg->positions[0].position.x,
                        msg->positions[0].position.y,
                        msg->positions[0].position.z);

        pf.InputHydrophone(pos, msg->header.stamp);
    }
}

void LocalizationSystem::linAccelCallback(const
        geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    dt = msg->header.stamp - last_lin_accel_timestamp;

    tf::Vector3 rel_lin_accel;
    rel_lin_accel[0] = msg->vector.x;
    rel_lin_accel[1] = msg->vector.y;
    rel_lin_accel[2] = msg->vector.z;

    tf::Vector3 abs_lin_accel = calculate_absolute_lin_accel(rel_lin_accel);

    pf.InputLinAccel(abs_lin_accel, dt.toSec());
    last_lin_accel_timestamp = msg->header.stamp;
}

void LocalizationSystem::orientationCallback(const
        robosub::QuaternionStampedAccuracy::ConstPtr &msg)
{
    orientation[0] = msg->quaternion.x;
    orientation[1] = msg->quaternion.y;
    orientation[2] = msg->quaternion.z;
    orientation[3] = msg->quaternion.w;
}

bool LocalizationSystem::resetFilterCallback(std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &rep)
{
    pf.Reset();
    return true;
}
