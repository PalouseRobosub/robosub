#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/Float64.h"
#include "robosub_msgs/joystick.h"
#include "robosub_msgs/control.h"
#include "std_srvs/Empty.h"

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

void reloadDepthParams()
{
    ros::param::getCached("joystick_control/min_depth", min_depth);
    ros::param::getCached("joystick_control/max_depth", max_depth);
}

bool checkArming(const robosub_msgs::joystick msg)
{
    int armingButtonsPressed = static_cast<int>(msg.buttons[2])
                            + static_cast<int>(msg.buttons[3])
                            + static_cast<int>(msg.buttons[4])
                            + static_cast<int>(msg.buttons[5]);
    if (armingButtonsPressed > 1)
    {
        ROS_WARN("More than one arming button is pressed!!!!");
    }
    return (armingButtonsPressed == 1);
}

void joystickToControlCallback(const robosub_msgs::joystick msg)
{
    reloadDepthParams();

    //Generate a control packet out of the type to send to control module
    robosub_msgs::control outmsg;
    outmsg.forward_state = outmsg.STATE_ERROR;
    outmsg.strafe_state  = outmsg.STATE_ERROR;
    outmsg.dive_state    = outmsg.STATE_ABSOLUTE;
    outmsg.yaw_state     = outmsg.STATE_RELATIVE;
    outmsg.forward = static_cast<double>(msg.axisX);
    outmsg.strafe_left = -msg.axisY;
    outmsg.yaw_left = msg.axisZ;
    outmsg.dive = static_cast<double>(
                            -((max_depth-min_depth) *
                            static_cast<double>(msg.throttle) + min_depth));

    if (msg.axisZ)
    {
        outmsg.yaw_state = outmsg.STATE_RELATIVE;
        outmsg.yaw_left = msg.axisZ;
    }
    else
    {
        outmsg.yaw_state = outmsg.STATE_NONE;
        outmsg.yaw_left = 0;
    }

    if (!msg.buttons[11])
    {
        if (msg.hatX)
        {
            outmsg.pitch_state = outmsg.STATE_RELATIVE;
            outmsg.pitch_down =  static_cast<double>(msg.hatX) * -10;
        }
        else
        {
            outmsg.pitch_state = outmsg.STATE_NONE;
            outmsg.pitch_down = 0;
        }
        if (msg.hatY)
        {
            outmsg.roll_state  = outmsg.STATE_RELATIVE;
            outmsg.roll_right = static_cast<double>(msg.hatY) * 10;
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
        outmsg.roll_right = 0;
        outmsg.pitch_down = 0;
    }
    outmsg.yaw_left *= -15;
    outmsg.forward *= 2;

    // Marker Droppers and Torpedos
    // buttons 3-6 are arming buttons, trigger fires
    // 3 and 4 arm marker droppers, 5 and 6 arm torpedos
    // (Subtract one from button number to get array index)
    ros::NodeHandle n;
    ros::ServiceClient client =
        n.serviceClient<std_srvs::Empty>("drop_marker");
    std_srvs::Empty e;

    if (msg.buttons[2] && checkArming(msg) && msg.buttons[0])
    {
        // Fire left marker dropper
        if (client.call(e))
        {
            ROS_INFO("Left Marker Dropped");
        }
        else
        {
            ROS_WARN("Failed to drop left marker");
        }
    }
    else if (msg.buttons[3] && checkArming(msg) && msg.buttons[0])
    {
        // Fire right marker dropper
        if (client.call(e))
        {
            ROS_INFO("Right Marker Dropped");
        }
        else
        {
            ROS_WARN("Failed to drop Right marker");
        }
    }
    else if (msg.buttons[4] && checkArming(msg) && msg.buttons[0])
    {
        // Fire left torpedo
    }
    else if (msg.buttons[5] && checkArming(msg) && msg.buttons[0])
    {
        // Fire right torpedo
    }

    pub.publish(outmsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joystick_control");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("joystick_driver", 1,
                                       joystickToControlCallback);
    pub = nh.advertise<robosub_msgs::control>("control", 1);
    nh = ros::NodeHandle("joystick_control");

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

    ros::spin();

    return 0;
}
