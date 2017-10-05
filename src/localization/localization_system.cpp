#include "localization_system.hpp"

LocalizationSystem::LocalizationSystem(ros::NodeHandle &_nh, RobosubSensors
        &_sensors):
    sensors(_sensors),
    nh(_nh),
    kalman_filter(_nh),
    particle_filter(_nh),
    transform_pub("tf", 1, 2.0),
    start_time(ros::Time::now())
{
}

bool LocalizationSystem::ResetFilterCallback(std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &rep)
{
    kalman_filter.Reset();
    particle_filter.Reset();

    return true;
}

geometry_msgs::Vector3Stamped LocalizationSystem::GetLocalizationMessage()
{
    geometry_msgs::Vector3Stamped msg;

    tf::Vector3 pos = particle_filter.GetPosition();

    msg.vector.x = pos[0];
    msg.vector.y = pos[1];
    msg.vector.z = pos[2];
    msg.header.stamp = ros::Time::now();

    return msg;
}

geometry_msgs::PoseStamped LocalizationSystem::GetPoseMessage()
{
    geometry_msgs::PoseStamped msg;

    tf::Vector3 pos = particle_filter.GetPosition();
    tf::Quaternion quat = sensors.GetOrientation();

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
    if(sensors.NewOrientation())
    {
        particle_filter.InputOrientation(sensors.GetOrientation());
    }

    if(sensors.NewDepth())
    {
        particle_filter.InputDepth(sensors.GetDepth());
        kalman_filter.InputDepth(sensors.GetDepth());
    }

    if(sensors.NewHydrophones())
    {
        particle_filter.InputHydrophones(sensors.GetHydrophones(),
                sensors.GetHydrophonesDT());

        // Since the particle filter will obtain the most accurate position
        // estimate after a hydrophone input, pass that position into the
        // kalman filter now.
        kalman_filter.InputPosition(particle_filter.GetPosition(),
                particle_filter.GetPositionDT());
    }

    if(sensors.NewAbsLinAcl())
    {
        kalman_filter.InputAbsLinAcl(sensors.GetAbsLinAcl(),
                sensors.GetAbsLinAclDT());
    }

    if(sensors.NewAbsLinVel())
    {
        // Since the particle filter takes some time to converge and is passing
        // positions to the kalman filter during that time, we cannot assume
        // that the linear velocity output from the kalman filter is accurate.
        // To alleviate this just delay the linear velocity from the kalman
        // filter from being input to the particle filter until things are
        // converged and stable.
        // TODO: Get measure of particle filter convergence so we don't have to
        // have a magic number here (or just parameterize the delay).
        if(ros::Time::now() - start_time > ros::Duration(20.0))
        {
            ROS_INFO_ONCE("Absolute linear velocity now being input to particle"
                    " filter");
            particle_filter.InputAbsLinVel(sensors.GetAbsLinVel(),
                    sensors.GetAbsLinVelDT());
        }
    }

    // Input output from filters to sensors class. On the next update the new
    // data from the kalman filter will be fed to the particle filter and vice
    // versa.
    if(kalman_filter.NewAbsLinVel())
    {
        sensors.InputAbsLinVel(kalman_filter.GetAbsLinVel());
    }

    if(particle_filter.NewPosition())
    {
        sensors.InputPosition(particle_filter.GetPosition());
    }

    tf::Vector3 pos = particle_filter.GetPosition();
    publish_tf_message(pos);
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
    transform_pub.publish(tm);
}
