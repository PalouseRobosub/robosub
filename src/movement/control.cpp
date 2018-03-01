#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/Accel.h"
#include "movement/control_system.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <utility/ThrottledPublisher.hpp>
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"

#include <string>

using namespace robosub;

/*
 * The control system object.
 */
ControlSystem *control_system;

/**
 * Specified true if the control system should be silenced.
 */
bool silence_control_system = false;

/*
 * Function to deal with Enable/Disable service call.
 */
bool disable(std_srvs::Empty::Request& req,
                   std_srvs::Empty::Response& res)
{
    control_system->setEnabled(false);
    return true;
}

/**
 * Service callback for silencing the control system.
 *
 * @param req The request.
 * @param res[out] The result of the request.
 *
 * @return True.
 */
bool toggle(std_srvs::SetBool::Request& req,
            std_srvs::SetBool::Response &res)
{
    silence_control_system = req.data;
    res.success = true;
    return true;
}

/**
 * Main entry point.
 *
 * @param argc, argv Arguments provided to ROS.
 *
 * @return Zero upon completion.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "control");

    ros::NodeHandle nh;

    control_system = new ControlSystem();

    ros::Subscriber depth_sub = nh.subscribe("depth", 1,
            &ControlSystem::InputDepthMessage, control_system);

    ros::Subscriber orientation_sub = nh.subscribe("orientation", 1,
            &ControlSystem::InputOrientationMessage, control_system);

    ros::Subscriber localization_sub = nh.subscribe("simulator/cobalt/position",
            1, &ControlSystem::InputLocalizationMessage, control_system);

    ros::Subscriber control_sub = nh.subscribe("control", 1,
            &ControlSystem::InputControlMessage, control_system);

    ros::Publisher pub = nh.advertise<robosub_msgs::thruster>("thruster", 1);

    ros::Publisher accel_pub = nh.advertise<geometry_msgs::Accel>(
        "control_acceleration", 1);

    rs::ThrottledPublisher<robosub_msgs::control_status>
            control_state_pub(std::string("control_status"), 1, 5);

    ros::Timer timer = nh.createTimer(ros::Duration(0.1),
            &ControlSystem::CheckTimeout, control_system);

    ros::ServiceServer disableService =
        nh.advertiseService("control/disable", disable);

    ros::ServiceServer silentService = nh.advertiseService("control/silence",
                                                           toggle);

    int rate;
    nh.getParam("rate/control", rate);
    ros::Rate r(rate);

    while(ros::ok())
    {
        control_state_pub.publish(control_system->GetControlStatus());
        if(control_system->isEnabled() && !silence_control_system)
        {
            pub.publish(control_system->CalculateThrusterMessage());
        }
        else
        {
            pub.publish(control_system->GetZeroThrusterMessage());
        }
        accel_pub.publish(control_system->GetAccelerationEstimate());
        ros::spinOnce();
        r.sleep();
    }

    delete control_system;

    return 0;
}
