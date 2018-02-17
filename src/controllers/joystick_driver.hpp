#ifndef JOYSTICK_DRIVER_HPP
#define JOYSTICK_DRIVER_HPP

#include <iostream>
#include <fcntl.h>
#include <linux/joystick.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include "robosub_msgs/joystick.h" // Joystick Msg

typedef struct
{
    double axisX;
    double axisY;
    double axisZ;

    int hatX;
    int hatY;

    double throttle;

    bool button[12];
} JOYSTICK_STATE;

class JoystickDriver
{
private:
    //helper functions
    void parse_event();
    //void dead_scale(double &value, double deadzone, double scaling_power);

    //constants
    int AXIS_MAX;
    int fd;
    js_event e;
    JOYSTICK_STATE joystick_data;

    ros::NodeHandle *node;

    //loadable parameters
    std::string device;

    double axisXdeadzone;
    double axisYdeadzone;
    double axisZdeadzone;

    double x_scaling_power;
    double y_scaling_power;
    double z_scaling_power;

    double min_depth;
    double max_depth;

public:
    robosub_msgs::joystick GetJoystickMessage();
    void shutdown();

    JoystickDriver(ros::NodeHandle *n);
    ~JoystickDriver() {}
};

#endif // JOYSTICK_DRIVER_HPP
