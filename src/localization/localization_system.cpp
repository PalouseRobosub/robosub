#include "localization_system.hpp"

LocalizationSystem::LocalizationSystem(int _num_particles) : pf(_num_particles)
{
    new_hydrophone = new_depth = new_lin_velocity = false;

    orientation.setW(1.0);
    orientation.setX(0.0);
    orientation.setY(0.0);
    orientation.setZ(0.0);

    last_lin_accel_time = ros::Time::now();
}

geometry_msgs::Vector3Stamped LocalizationSystem::GetLocalizationMessage()
{
    geometry_msgs::Vector3Stamped s;

    tf::Vector3 pos = pf.GetPosition();

    s.vector.x = pos.getX();
    s.vector.y = pos.getY();
    s.vector.z = pos.getZ();
    s.header.stamp = ros::Time::now();

    return s;
}

void LocalizationSystem::calculate_absolute_lin_accel()
{
    tf::Quaternion orientation_conjugate;
    orientation_conjugate.setX(orientation.getX() * -1.0);
    orientation_conjugate.setY(orientation.getY() * -1.0);
    orientation_conjugate.setZ(orientation.getZ() * -1.0);
    orientation_conjugate.setW(orientation.getW());

    tf::Matrix3x3 rot_m = tf::Matrix3x3(orientation_conjugate);

    abs_lin_accel = rot_m * rel_lin_accel;

    //ROS_INFO("rel_lin_accel:\n[%f, %f, %f]", rel_lin_accel.getX(), rel_lin_accel.getY(), rel_lin_accel.getZ());
    //ROS_INFO("abs_lin_accel:\n[%f, %f, %f]", abs_lin_accel.getX(), abs_lin_accel.getY(), abs_lin_accel.getZ());
}

void LocalizationSystem::depthCallback(const robosub::depth_stamped::ConstPtr &msg)
{
    pf.InputDepth(msg->depth, msg->header.stamp);
    new_depth = true;
}

void LocalizationSystem::hydrophoneCallback(const robosub::PositionArrayStamped::ConstPtr &msg)
{
    if(msg->positions.size() > 0)
    {
        tf::Vector3 pos(msg->positions[0].position.x,
                        msg->positions[0].position.y,
                        msg->positions[0].position.z);

        pf.InputHydrophone(pos, msg->header.stamp);
        new_hydrophone = true;
    }
}

void LocalizationSystem::linAccelCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    dt = msg->header.stamp - last_lin_accel_time;
    rel_lin_accel[0] = msg->vector.x;
    rel_lin_accel[1] = msg->vector.y;
    rel_lin_accel[2] = msg->vector.z;

    calculate_absolute_lin_accel();

    pf.InputLinAccel(abs_lin_accel, dt.toSec(), msg->header.stamp);
    new_lin_velocity = true;
    last_lin_accel_time = msg->header.stamp;
}

void LocalizationSystem::orientationCallback(const robosub::QuaternionStampedAccuracy::ConstPtr &msg)
{
    orientation.setX(msg->quaternion.x);
    orientation.setY(msg->quaternion.y);
    orientation.setZ(msg->quaternion.z);
    orientation.setW(msg->quaternion.w);

    //pf.InputOrientation(orientation, msg->header.stamp);
}

bool LocalizationSystem::resetFilterCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &rep)
{
    pf.Reset();
    return true;
}

void LocalizationSystem::Update()
{
    // Getto ros message filter
    // TODO: Figure out when to run update loop
    // TODO: This probably doesn't work anymore. Possibly run iteration on any
    // new sensor reading, keeping old readings same.
    if(new_hydrophone || (new_depth && new_lin_velocity))
    {
        pf.Update();

        new_hydrophone = new_depth = new_lin_velocity = false;
    }
}