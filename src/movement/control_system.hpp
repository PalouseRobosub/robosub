#ifndef CONTROLSYSTEM_HPP
#define CONTROLSYSTEM_HPP

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <string>
#include <stdint.h>

#include "rotation_engine.hpp"
#include "ros/ros.h"
#include "robosub/thruster.h"
#include "robosub/control.h"
#include "geometry_msgs/Quaternion.h"

using namespace Eigen;
using std::cout;
using std::endl;
using std::string;

typedef Matrix<double,6,1> Vector6d;
typedef Matrix<int,6,1> Vector6i;
typedef Matrix<double,12,1> Vector12d;
typedef Matrix<double,6,6> Matrix6d;

#define _PI_OVER_180 (3.1415 / 180.0)

int Sgn(float x)
{
    return (x > 0) - (x < 0);
}

class ControlSystem
{
private:
    void update();

    //string statestring(control_packet::State state);
    VectorXd motor_control(Vector12d state);
    double wraparound(double x, double min, double max);

    //internal state information
    Vector6d goals;
    Matrix<uint8_t, 6, 1> goal_types;
    Vector6d integral_state;
    VectorXd motor_commands;

    //loadable parameters
    Vector6d P, I, D;
    Vector6d windup, hysteresis;
    Vector6d sub_mass;
    Vector6d offsets;
    MatrixXd motors;
    MatrixXd position;
    MatrixXd orientation;
    MatrixXd M;
    double t_lim;
    double r_lim;
    double max_thrust;
    double back_thrust_ratio;
    Vector12d state_vector;
    double dt;
    short int num_thrusters;
    // need vector/matrix for submarine parameters like mass

    // ros stuff
    ros::NodeHandle *nh;
    ros::Publisher *pub;

public:
    //ControlSystem();
	ControlSystem(ros::NodeHandle *nh, ros::Publisher *pub);
    ~ControlSystem() {}

    void InputControlMessage(robosub::control msg);
    void InputOrientationMessage(geometry_msgs::Quaternion msg);
};

#endif
