#include "localization_system.hpp"

LocalizationSystem::LocalizationSystem(ros::NodeHandle *_nh, RobosubSensors
        *_sensors) : kf(_nh), pf(_nh)
{
    nh = _nh;
    sensors = _sensors;

    tf_pub = nh->advertise<tf2_msgs::TFMessage>("tf", 1);

    start_time = ros::Time::now();
}

bool LocalizationSystem::ResetFilterCallback(std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &rep)
{
    kf.Reset();
    pf.Reset();

    return true;
}

geometry_msgs::Vector3Stamped LocalizationSystem::GetLocalizationMessage()
{
    geometry_msgs::Vector3Stamped msg;

    tf::Vector3 pos = pf.GetPosition();

    msg.vector.x = pos[0];
    msg.vector.y = pos[1];
    msg.vector.z = pos[2];
    msg.header.stamp = ros::Time::now();

    publish_tf_message(pos);

    return msg;
}

geometry_msgs::PoseStamped LocalizationSystem::GetPoseMessage()
{
    geometry_msgs::PoseStamped msg;

    tf::Vector3 pos = pf.GetPosition();
    tf::Quaternion quat = sensors->GetOrientation();

    msg.pose.position.x = pos[0];
    msg.pose.position.y = pos[1];
    msg.pose.position.z = pos[2];

    msg.pose.orientation.x = quat[0];
    msg.pose.orientation.y = quat[1];
    msg.pose.orientation.z = quat[2];
    msg.pose.orientation.w = quat[3];

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";

    return msg;
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
        pf.InputHydrophones(sensors->GetHydrophones(),
                sensors->GetHydrophonesDT());

        // Since the pf will obtain the most accurate position estimate after a
        // hydrophone input, pass that position into the kf now.
        kf.InputPosition(sensors->GetPosition(), sensors->GetPositionDT());
    }
    if(sensors->NewAbsLinAcl())
    {
        kf.InputAbsLinAcl(sensors->GetAbsLinAcl(), sensors->GetAbsLinAclDT());
    }
    if(sensors->NewAbsLinVel())
    {
        // TODO: Get measure of pf convergence. Only input lin vel when pf
        // is converged and in agreement with the kf.
        if(ros::Time::now().toSec() - start_time.toSec() > 20.0)
        {
            pf.InputAbsLinVel(sensors->GetAbsLinVel(),
                    sensors->GetAbsLinVelDT());
        }
    }

    // Input output from filters to sensors class. On the next update the new
    // data from the kf will be fed to the pf and vice versa.
    if(kf.NewAbsLinVel())
    {
            sensors->InputAbsLinVel(kf.GetAbsLinVel());
    }
    if(pf.NewPosition())
    {
        sensors->InputPosition(pf.GetPosition());
    }
}

void LocalizationSystem::publish_tf_message(tf::Vector3 pos)
{
    tf2_msgs::TFMessage tm;

    geometry_msgs::TransformStamped robosub_transform;

    robosub_transform.header.frame_id = "world";
    robosub_transform.header.stamp = ros::Time::now();
    robosub_transform.child_frame_id = "cobalt";

    robosub_transform.transform.translation.x = pos[0];
    robosub_transform.transform.translation.y = pos[1];
    robosub_transform.transform.translation.z = pos[2];
    robosub_transform.transform.rotation.x = 0.0;
    robosub_transform.transform.rotation.y = 0.0;
    robosub_transform.transform.rotation.z = 0.0;
    robosub_transform.transform.rotation.w = 1.0;

    tm.transforms.push_back(robosub_transform);
    tf_pub.publish(tm);
}
