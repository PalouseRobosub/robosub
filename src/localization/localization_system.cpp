#include "localization_system.hpp"

//LocalizationSystem::LocalizationSystem(ros::NodeHandle _nh, int _num_particles) : kf(_nh), pf(_num_particles)
LocalizationSystem::LocalizationSystem(FilterSensors *_sensors, ros::NodeHandle _nh, int _num_particles) : kf(_nh), pf(_num_particles)
{
    sensors = _sensors;
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

bool LocalizationSystem::resetFilterCallback(std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &rep)
{
    pf.Reset();
    kf.Reset();

    return true;
}

void LocalizationSystem::Update()
{
    if(sensors->NewDepth())
    {
        ROS_INFO_STREAM("NewDepth");
        pf.InputDepth(sensors->GetDepth(), sensors->GetDepthDT());
        kf.InputDepth(sensors->GetDepth(), sensors->GetDepthDT());
    }
    if(sensors->NewHydrophones())
    {
        ROS_INFO_STREAM("NewHydrophones");
        pf.InputHydrophones(sensors->GetHydrophones(), sensors->GetHydrophonesDT());
    }
    if(sensors->NewAbsLinAcl())
    {
        ROS_INFO_STREAM("NewAbsLinAcl");
        kf.InputAbsLinAcl(sensors->GetAbsLinAcl(), sensors->GetAbsLinAclDT());
    }
    if(sensors->NewAbsLinVel())
    {
        ROS_INFO_STREAM("NewAbsLinVel");
        pf.InputAbsLinVel(sensors->GetAbsLinVel(), sensors->GetAbsLinVelDT());
    }

    sensors->InputAbsLinVel(kf.GetLinVelocity());
    sensors->InputPosition(pf.GetPosition());

    ROS_INFO_STREAM("");
}
