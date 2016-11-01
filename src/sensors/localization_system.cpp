#include "localization_system.hpp"

LocalizationSystem::LocalizationSystem(double _dt)
{
    pos_x = pos_y = pos_z = 0.0;
    vel_x = vel_y = vel_z = 0.0;
    dt = _dt;

    new_orientation = new_accel = new_depth = false;
}

void LocalizationSystem::InputOrientation(geometry_msgs::Quaternion msg)
{
    orientation = msg;
    new_orientation = true;
}

void LocalizationSystem::InputAccel(geometry_msgs::Vector3 msg)
{
    accel = msg;
    new_accel = true;
}

void LocalizationSystem::InputDepth(std_msgs::Float32 msg)
{
    depth = msg;
    new_depth = true;
}

void LocalizationSystem::SetPosition(double _x, double _y, double _z)
{
    pos_x = _x;
    pos_y = _y;
    pos_z = _z;
}

void LocalizationSystem::SetVelocity(double _x, double _y, double _z)
{
    vel_x = _x;
    vel_y = _y;
    vel_z = _z;
}

void LocalizationSystem::FindLinearAccel()
{
    // Find actual linear acceleration by subtracting gravity vector
    // (found by converting orientation to an orientation vector)
    // The bno sensor of course already provides this so this step is temporary
    tf::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    tf::Matrix3x3 m(q);
    tf::Vector3 v(0.0, 0.0, 9.81); // Gravity vector

    tf::Vector3 gravity_vector = m * v;

    ROS_INFO_STREAM("gravity_vector.x: " << gravity_vector.x());
    ROS_INFO_STREAM("gravity_vector.y: " << gravity_vector.y());
    ROS_INFO_STREAM("gravity_vector.z: " << gravity_vector.z());

    //double accel_norm = std::sqrt(accel.x*accel.x + accel.y*accel.y + accel.z*accel.z);
    //ROS_INFO_STREAM("accel_norm: " << accel_norm);

    lin_accel_x = accel.x + gravity_vector.getX();
    lin_accel_y = accel.y - gravity_vector.getY();
    lin_accel_z = accel.z - gravity_vector.getZ();

    //double lin_accel_norm = std::sqrt(lin_accel_x*lin_accel_x + lin_accel_y*lin_accel_y + lin_accel_z*lin_accel_z);
    //ROS_INFO_STREAM("lin_accel_norm: " << lin_accel_norm);

    ROS_INFO_STREAM("accel_x: " << accel.x);
    ROS_INFO_STREAM("accel_y: " << accel.y);
    ROS_INFO_STREAM("accel_z: " << accel.z);

    ROS_INFO_STREAM("lin_accel_x: " << lin_accel_x);
    ROS_INFO_STREAM("lin_accel_y: " << lin_accel_y);
    ROS_INFO_STREAM("lin_accel_z: " << lin_accel_z);
}

void LocalizationSystem::Update()
{
    // Getto ros message filter
    // Should work for now since bno sends its data up all at one time
    if(new_orientation && new_accel && new_depth)
    {
        FindLinearAccel();

        vel_x += lin_accel_x * dt;
        vel_y += lin_accel_y * dt;
        vel_z += lin_accel_z * dt;

        pos_x += (vel_x * dt);
        pos_y += (vel_y * dt);
        pos_z += (vel_z * dt);

        ROS_INFO_STREAM("vel_x: " << vel_x);
        ROS_INFO_STREAM("vel_y: " << vel_y);
        ROS_INFO_STREAM("vel_z: " << vel_z);

        ROS_INFO_STREAM("pos_x: " << pos_x);
        ROS_INFO_STREAM("pos_y: " << pos_y);
        ROS_INFO_STREAM("pos_z: " << pos_z);

        new_orientation = new_accel = new_depth = false;

        ROS_INFO_STREAM("dt: " << dt);
        ROS_INFO_STREAM(std::endl);
    }
}
