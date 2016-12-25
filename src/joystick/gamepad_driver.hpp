#ifndef GAMEPAD_DRIVER_HPP
#define GAMEPAD_DRIVER_HPP

#include <iostream>
#include <fcntl.h>
#include <linux/joystick.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include "robosub/joystick.h" // Joystick Msg

typedef struct
{
    double axisX;
    double axisY;
    double axisZ;

    int hatX;
    int hatY;

    double throttle;

    bool button[12];
} GAMEPAD_STATE;

class GamepadDriver
{
private:
    //helper functions
    void parse_event();

    //constants
    int AXIS_MAX;
    int fd;
    js_event e;
    GAMEPAD_STATE gamepad_data;

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
    robosub::joystick GetGamepadMessage();
    void shutdown();

    GamepadDriver(ros::NodeHandle *n);
    ~GamepadDriver() {}
};

#endif // GAMEPAD_DRIVER_HPP
