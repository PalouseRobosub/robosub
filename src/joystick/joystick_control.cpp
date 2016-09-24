#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/Float64.h"
#include "robosub/joystick.h"
#include "robosub/control.h"

ros::Publisher pub;

// Settings
double min_depth = 0.0;
double max_depth = 0.0;
double axisXdeadzone = 0.0;
double axisYdeadzone = 0.0;
double axisZdeadzone = 0.0;
double x_scaling_power = 0.0;
double y_scaling_power = 0.0;
double z_scaling_power = 0.0;

/*
void dead_scale(double *value, double deadzone, double scaling_power)
{
    double num;
    num = (fabs(*value) - deadzone);
    if( num < 0)
        num = 0;

    double sgn = (*value > 0) ? 1.0 : -1.0;
    *value = pow( num/(1-deadzone), scaling_power ) * sgn;
}
*/

void joystickToControlCallback(const robosub::joystick msg)
{

    //scale and apply deadzones
    //dead_scale(msg.axisX, axisXdeadzone, x_scaling_power);
    //dead_scale(msg.axisY, axisYdeadzone, y_scaling_power);
    //dead_scale(msg.axisZ, axisZdeadzone, z_scaling_power);

    //Generate a control packet out of the type to send to control module
    robosub::control outmsg;
    outmsg.forward_state = outmsg.STATE_ERROR;
    outmsg.strafe_state  = outmsg.STATE_ERROR;
    outmsg.dive_state    = outmsg.STATE_ABSOLUTE;
    outmsg.yaw_state     = outmsg.STATE_RELATIVE;
    outmsg.forward = (double)msg.axisX;
    outmsg.strafe_right = msg.axisY;
    outmsg.yaw_right = msg.axisZ;
    outmsg.dive = (double) -((max_depth-min_depth)*(double)msg.throttle + min_depth);

    if (msg.axisZ)
    {
        outmsg.yaw_state = outmsg.STATE_RELATIVE;
        outmsg.yaw_right = msg.axisZ;
    }
    else
    {
        outmsg.yaw_state = outmsg.STATE_NONE;
        outmsg.yaw_right = 0;
    }

    if (!msg.buttons[0])
    {
        if (msg.hatX)
        {
            outmsg.pitch_state = outmsg.STATE_RELATIVE;
            outmsg.pitch_up = (double) msg.hatX * -10;
        }
        else
        {
            outmsg.pitch_state = outmsg.STATE_NONE;
            outmsg.pitch_up = 0;
        }
        if (msg.hatY)
        {
            outmsg.roll_state  = outmsg.STATE_RELATIVE;
            outmsg.roll_right = (double) msg.hatY * 10;
        }
        else
        {
            outmsg.roll_state = outmsg.STATE_NONE;
            outmsg.roll_right = 0;
        }

    }
    else
    {
        outmsg.pitch_state = outmsg.STATE_ABSOLUTE;
        outmsg.roll_state  = outmsg.STATE_ABSOLUTE;
        outmsg.yaw_state  = outmsg.STATE_ABSOLUTE;
        outmsg.roll_right = 0;
        outmsg.pitch_up = 0;
        outmsg.yaw_right = 0;
    }
    outmsg.yaw_right *= -15;
    outmsg.forward *= 2;

    pub.publish(outmsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joystick_control");

    ros::NodeHandle nh("joystick_control");

    // Load settings
    nh.getParam("min_depth", min_depth);
    nh.getParam("max_depth", max_depth);
    nh.getParam("axisXdeadzone", axisXdeadzone);
    nh.getParam("axisYdeadzone", axisYdeadzone);
    nh.getParam("axisZdeadzone", axisZdeadzone);
    nh.getParam("x_scaling_power", x_scaling_power);
    nh.getParam("y_scaling_power", y_scaling_power);
    nh.getParam("z_scaling_power", z_scaling_power);

    ROS_INFO("min_depth %f", min_depth);
    ROS_INFO("max_depth %f", max_depth);
    ROS_INFO("axisXdeadzone %f", axisXdeadzone);
    ROS_INFO("axisYdeadzone %f", axisYdeadzone);
    ROS_INFO("axisZdeadzone %f", axisZdeadzone);
    ROS_INFO("x_scaling_power %f", x_scaling_power);
    ROS_INFO("y_scaling_power %f", y_scaling_power);
    ROS_INFO("z_scaling_power %f", z_scaling_power);

    ros::Subscriber sub = nh.subscribe("joystick_driver", 1, joystickToControlCallback);
    pub = nh.advertise<robosub::control>("joystick_control", 1);

    ros::spin();

    return 0;
}
