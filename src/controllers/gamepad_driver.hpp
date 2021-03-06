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
#include "robosub_msgs/gamepad.h" // Gamepad Msg

typedef struct
{
    double axisX;
    double axisY;
    double axisZ;

    double axisRX;
    double axisRY;
    double axisRZ;

    int hatX;
    int hatY;

    bool button[19];

    uint8_t type;
} GAMEPAD_STATE;

class GamepadDriver
{
private:
    //helper functions
    void parse_event();

    //constants
    int AXIS_MAX;
    int fd;
    char name[128];
    uint8_t num_btns;
    js_event e;
    GAMEPAD_STATE gamepad_data;

    ros::NodeHandle *node;

    //loadable parameters
    std::string device;

public:
    robosub_msgs::gamepad GetGamepadMessage();
    void shutdown();

    GamepadDriver(ros::NodeHandle *n);
    ~GamepadDriver() {}
};

#endif // GAMEPAD_DRIVER_HPP
