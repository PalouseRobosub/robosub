#ifndef JOYSTICKDRIVER_HPP
#define JOYSTICKDRIVER_HPP

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
    robosub::joystick GetJoystickMessage();
    void shutdown();

    JoystickDriver(ros::NodeHandle *n);
    ~JoystickDriver() {}
};

#endif

/*
#ifndef JOYSTICKDRIVER_HPP
#define JOYSTICKDRIVER_HPP

#include <iostream>
#include <fcntl.h>
#include <linux/joystick.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
//#include <string.h>

#include <string>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "robosub/joystick.h" // Joystick Msg
#include "robosub/control.h" // Control Msg

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

   ostream& operator<<(ostream& os, const js_event& e)
   {
   os << "time:  " << e.time << endl;
   os << "value: " << e.value << endl;
   os << "type:  ";
   switch(e.type & ~JS_EVENT_INIT)
   {
   case JS_EVENT_BUTTON:
   os << "BUTTON";
   break;
   case JS_EVENT_AXIS:
   os << "AXIS";
   break;
   default:
   os << "UNKNOWN";
   }
   if(e.type & JS_EVENT_INIT)
   os << " (INIT)";
   os << endl;
   os << "number:  " << (int) e.number << endl << endl;

   return os;
   }

   ostream& operator<<(ostream& os, JOYSTICK_STATE& state)
   {
   os << "axisX: " << state.axisX << endl;
   os << "axisY: " << state.axisY << endl;
   os << "axisZ: " << state.axisZ << endl;

   os << "hatX: " << state.hatX << endl;
   os << "hatY: " << state.hatY << endl;

   os << "throttle: " << state.throttle << endl;

   for (int i = 0; i < 12; ++i)
   {
   os << "button " << i+1 << ":  " << state.button[i] << endl;;
   }

   return os;
   }

class JoystickDriver
{
    private:
        void init();
        void update();
        void shutdown();

        //helper functions
        void parse_event();
        void print_js_event(JOYSTICK_STATE &js_state);
        string statestring(robosub::joystick state);
        void dead_scale(double &value, double deadzone, double scaling_power);

        //constants
        const int AXIS_MAX = 32767;
        int fd;
        js_event e;
        JOYSTICK_STATE js_state = {0};
        JOYSTICK_STATE js_state_copy = {0};

        //loadable parameters
        string device;

        double axisXdeadzone;
        double axisYdeadzone;
        double axisZdeadzone;

        double x_scaling_power;
        double y_scaling_power;
        double z_scaling_power;

        double min_depth;
        double max_depth;

    public:
        JoystickDriver();
        ~JoystickDriver() {}
};

#endif
*/
