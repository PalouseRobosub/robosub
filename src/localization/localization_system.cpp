#include "localization_system.hpp"

LocalizationSystem::LocalizationSystem(FilterSensors *_sensors, int _num_particles) : kf(), pf(_num_particles)
{
    sensors = _sensors;
    num_positions = 0;
}

geometry_msgs::Vector3Stamped LocalizationSystem::GetLocalizationMessage()
{
    geometry_msgs::Vector3Stamped msg;

    tf::Vector3 pos = pf.GetPosition();

    msg.vector.x = pos[0];
    msg.vector.y = pos[1];
    msg.vector.z = pos[2];
    msg.header.stamp = ros::Time::now();

    return msg;
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
    // Handle input to filters
    if(sensors->NewDepth())
    {
        pf.InputDepth(sensors->GetDepth(), sensors->GetDepthDT());
        kf.InputDepth(sensors->GetDepth(), sensors->GetDepthDT());
    }
    if(sensors->NewHydrophones())
    {
        pf.InputHydrophones(sensors->GetHydrophones(), sensors->GetHydrophonesDT());
    }
    if(sensors->NewAbsLinAcl())
    {
        kf.InputAbsLinAcl(sensors->GetAbsLinAcl(), sensors->GetAbsLinAclDT());
    }
    if(sensors->NewAbsLinVel())
    {
        pf.InputAbsLinVel(sensors->GetAbsLinVel(), sensors->GetAbsLinVelDT());
    }
    if(sensors->NewPosition())
    {
        num_positions++;
        if(num_positions % 100 == 1)
        {
            kf.InputPosition(sensors->GetPosition(), sensors->GetPositionDT());
        }
    }

    // Handle output from filters. On the next update the new data from the kf
    // will be fed to the pf and vice versa.
    if(kf.NewAbsLinVel())
    {
        sensors->InputAbsLinVel(kf.GetAbsLinVel());
    }
    if(pf.NewPosition())
    {
        sensors->InputPosition(pf.GetPosition());
    }
}
