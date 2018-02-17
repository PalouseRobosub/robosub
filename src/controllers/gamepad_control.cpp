#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "robosub_msgs/gamepad.h"
#include "robosub_msgs/control.h"
#include "std_srvs/Empty.h"

ros::Publisher pub;

// Settings
double axisXdeadzone = 0.0;
double axisYdeadzone = 0.0;
double axisZdeadzone = 0.0;
double axisRXdeadzone = 0.0;
double axisRYdeadzone = 0.0;
double axisRZdeadzone = 0.0;
double x_scaling_power = 0.0;
double y_scaling_power = 0.0;
double z_scaling_power = 0.0;
double rx_scaling_power = 0.0;
double ry_scaling_power = 0.0;
double rz_scaling_power = 0.0;

double dead_scale(double value, double deadzone, double scaling_power)
{
    double num;
    num = (fabs(value) - deadzone);
    if( num < 0)
        num = 0;

    double sgn = (value > 0) ? 1.0 : -1.0;
    return pow( num/(1-deadzone), scaling_power ) * sgn;
}

bool checkArmingAndTriggers(const robosub_msgs::gamepad msg)
{
    int armingButtonsPressed = 0;
    int triggersPressed = 0;
    if (msg.type == robosub_msgs::gamepad::XBOX)
    {
        armingButtonsPressed = static_cast<int>(msg.buttons[1])
                            + static_cast<int>(msg.buttons[2]);

        triggersPressed = static_cast<int>(msg.buttons[4])
                        + static_cast<int>(msg.buttons[5]);
    }
    else if (msg.type == robosub_msgs::gamepad::PS3)
    {
        armingButtonsPressed = static_cast<int>(msg.buttons[13])
                            + static_cast<int>(msg.buttons[15]);

        triggersPressed = static_cast<int>(msg.buttons[10])
                        + static_cast<int>(msg.buttons[11]);
    }
    if (armingButtonsPressed > 1)
    {
        ROS_WARN("More than one arming button is pressed!!!!");
    }

    if (triggersPressed > 1)
    {
        ROS_WARN("More than one firing trigger is pressed");
    }

    return (armingButtonsPressed == 1) && (triggersPressed == 1);
}

void gamepadToControlCallback(const robosub_msgs::gamepad msg)
{
    //scale and apply deadzones
    double axisX = dead_scale(msg.axisX, axisXdeadzone, x_scaling_power);
    double axisY = dead_scale(msg.axisY, axisYdeadzone, y_scaling_power);
    double axisZ = dead_scale(msg.axisZ, axisZdeadzone, z_scaling_power);
    double axisRX = dead_scale(msg.axisRX, axisRXdeadzone, rx_scaling_power);
    double axisRZ = dead_scale(msg.axisRZ, axisRZdeadzone, rz_scaling_power);

    //Generate a control packet out of the type to send to control module
    robosub_msgs::control outmsg;
    outmsg.forward_state = outmsg.STATE_ERROR;
    outmsg.strafe_state  = outmsg.STATE_ERROR;
    outmsg.dive_state    = outmsg.STATE_RELATIVE;
    outmsg.yaw_state     = outmsg.STATE_RELATIVE;
    outmsg.forward = -static_cast<double>(axisY);
    outmsg.strafe_left = -axisX;
    outmsg.yaw_left = axisRX;
    outmsg.dive = 0.5 * (static_cast<double>(axisZ) -
                  static_cast<double>(axisRZ));

    if (!axisRX)
    {
        outmsg.yaw_state = outmsg.STATE_NONE;
        outmsg.yaw_left = 0;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("drop_marker");
    std_srvs::Empty e;
    // Using Xbox controller
    if (robosub_msgs::gamepad::XBOX == msg.type && !msg.buttons[8])
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

        // Marker Droppers and Torpedos
        // buttons 2 (X) and 1 (Square) are arming buttons
        // RB and LB fire respective side
        if (msg.buttons[2] && checkArmingAndTriggers(msg)
                && msg.buttons[4])
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
        else if (msg.buttons[2] && checkArmingAndTriggers(msg)
                && msg.buttons[5])
        {
            // Fire right marker dropper
            if (client.call(e))
            {
                ROS_INFO("Right Marker Dropped");
            }
            else
            {
                ROS_WARN("Failed to drop right marker");
            }
        }
        else if (msg.buttons[1] && checkArmingAndTriggers(msg)
                && msg.buttons[4])
        {
            // Fire left torpedo
        }
        else if (msg.buttons[1] && checkArmingAndTriggers(msg)
                && msg.buttons[5])
        {
            // Fire right torpedo
        }
    }
    // Using PlayStation controller
    else if (robosub_msgs::gamepad::PS3 == msg.type && !msg.buttons[3])
    {
        if (msg.buttons[4] || msg.buttons[6])
        {
            outmsg.pitch_state = outmsg.STATE_RELATIVE;
            outmsg.pitch_down = static_cast<double>(msg.buttons[4]
                                                    - msg.buttons[6]) * 10;
        }
        else
        {
            outmsg.pitch_state = outmsg.STATE_NONE;
            outmsg.pitch_down = 0;
        }
        if (msg.buttons[5] || msg.buttons[7])
        {
            outmsg.roll_state = outmsg.STATE_RELATIVE;
            outmsg.roll_right = static_cast<double>(msg.buttons[5]
                                                    - msg.buttons[7]) * 10;
        }
        else
        {
            outmsg.roll_state = outmsg.STATE_NONE;
            outmsg.roll_right = 0;
        }

        // Marker Droppers and Torpedos
        // buttons 13 (Circle) and 15 (Square) are arming buttons
        // R1 and L1 fire respective side
        if (msg.buttons[15] && checkArmingAndTriggers(msg)
                && msg.buttons[10])
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
        else if (msg.buttons[15] && checkArmingAndTriggers(msg)
                && msg.buttons[11])
        {
            // Fire right marker dropper
            if (client.call(e))
            {
                ROS_INFO("Right Marker Dropped");
            }
            else
            {
                ROS_WARN("Failed to drop right marker");
            }
        }
        else if (msg.buttons[13] && checkArmingAndTriggers(msg)
                && msg.buttons[10])
        {
            // Fire left torpedo
        }
        else if (msg.buttons[13] && checkArmingAndTriggers(msg)
                && msg.buttons[11])
        {
            // Fire right torpedo
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

    pub.publish(outmsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gamepad_control");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("gamepad_driver", 1,
                                       gamepadToControlCallback);
    pub = nh.advertise<robosub_msgs::control>("control", 1);
    nh = ros::NodeHandle("gamepad_control");

    // Load settings
    nh.getParam("axisXdeadzone", axisXdeadzone);
    nh.getParam("axisYdeadzone", axisYdeadzone);
    nh.getParam("axisZdeadzone", axisZdeadzone);
    nh.getParam("axisRXdeadzone", axisRXdeadzone);
    nh.getParam("axisRYdeadzone", axisRYdeadzone);
    nh.getParam("axisRZdeadzone", axisRZdeadzone);
    nh.getParam("x_scaling_power", x_scaling_power);
    nh.getParam("y_scaling_power", y_scaling_power);
    nh.getParam("z_scaling_power", z_scaling_power);
    nh.getParam("rx_scaling_power", rx_scaling_power);
    nh.getParam("ry_scaling_power", ry_scaling_power);
    nh.getParam("rz_scaling_power", rz_scaling_power);

    ROS_INFO("axisXdeadzone %f", axisXdeadzone);
    ROS_INFO("axisYdeadzone %f", axisYdeadzone);
    ROS_INFO("axisZdeadzone %f", axisZdeadzone);
    ROS_INFO("axisRXdeadzone %f", axisRXdeadzone);
    ROS_INFO("axisRYdeadzone %f", axisRYdeadzone);
    ROS_INFO("axisRZdeadzone %f", axisRZdeadzone);
    ROS_INFO("x_scaling_power %f", x_scaling_power);
    ROS_INFO("y_scaling_power %f", y_scaling_power);
    ROS_INFO("z_scaling_power %f", z_scaling_power);
    ROS_INFO("rx_scaling_power %f", rx_scaling_power);
    ROS_INFO("ry_scaling_power %f", ry_scaling_power);
    ROS_INFO("rz_scaling_power %f", rz_scaling_power);

    ros::spin();

    return 0;
}
