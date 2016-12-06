#ifndef CONTROLSYSTEM_HPP
#define CONTROLSYSTEM_HPP

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <string>
#include <cstdint>

#include "rotation_engine.hpp"
#include "ros/ros.h"
#include "robosub/thruster.h"
#include "robosub/control.h"
#include "robosub/control_status.h"
#include "robosub/depth_stamped.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/QuaternionStamped.h"

#include "tf/transform_datatypes.h"

using namespace Eigen;
using std::cout;
using std::endl;
using std::string;

class ControlSystem
{
    /*
     * Define custom types for using Eigen.
     */
    using Vector6d = Matrix<double,6,1>;
    using Vector6 = Matrix<int,6,1> ;
    using Vector12d = Matrix<double,12,1>;
    using Matrix6d = Matrix<double,6,6>;

    /*
     * Defines conversion scale for converting between radians and degrees.
     */
    static constexpr double _PI_OVER_180 = (3.1415 / 180.0);
    static constexpr double _180_OVER_PI = (180.0 / 3.14159);

public:

    ControlSystem() = delete;
	ControlSystem(ros::Publisher *pub);

    void InputControlMessage(robosub::control msg);
    void InputOrientationMessage(geometry_msgs::QuaternionStamped quat_msg);
    void InputDepthMessage(robosub::depth_stamped depth_msg);
    void ReloadPIDParams();
    void CalculateThrusterMessage();
    void PublishThrusterMessage();
    robosub::control_status GetControlStatus();
private:

    void calculate_motor_control();
    double wraparound(double x, double min, double max);
    std::string state_to_string(uint8_t state);

    //internal state information
    Vector6d goals;
    Matrix<uint8_t, 6, 1> goal_types;

    /*
     * The current status of all the integrals for X, Y, Z, PSI, PHI, and
     * THETA. Elements are stored in that order.
     */
    Vector6d current_error;
    Vector6d current_integral;
    Vector6d current_derivative;

    /*
     * Defines the last calculated non-truncated translation control.
     */
    VectorXd translation_control;

    /*
     * Defines the last calculated non-truncated rotation control.
     */
    VectorXd rotation_control;

    /*
     * Defines the total control message of rotation and translation after
     * truncation has occurred.
     */
    VectorXd total_control;

    Vector6d P, I, D;
    Vector6d windup, hysteresis;
    Vector6d sub_mass;
    Vector6d offsets;

    /*
     * Define the motor matrix used to solve for the motor commands.
     */
    MatrixXd motors;

    /*
     * Defines the maximum absolute limit on a translational motor control
     * command.
     */
    double t_lim;

    /*
     * Defines the maximum absolute limit on a rotational motor control
     * command.
     */
    double r_lim;

    /*
     * Defines the maximum thrust a thruster can output in newtons.
     */
    double max_thrust;

    /*
     * Defines the maximum ratio of thruster when a thruster is operating in
     * reverse direction. For example, the back thrust ratio could be 80% of
     * nominal.
     */
    double back_thrust_ratio;

    /*
     * Defines the current state of the submarine. This vector stores elements
     * in the following order:
     *      X, Y, Z, X', Y', Z', Psi (roll), Phi (pitch), Theta (yaw), Psi',
     *      Phi', Theta'
     */
    Vector12d state_vector;

    /*
     * Defines the amount of time that passes between each control cycle.
     */
    double dt;

    /*
     * Defines the number of thrusters on the submarine.
     */
    short int num_thrusters;

    /*
     * Defines a node handle for reading control system parameters.
     */
    ros::NodeHandle nh;

    /*
     * Defines a publisher that can be used for publishing thruster messages.
     */
    ros::Publisher *pub;

    /*
     * The thruster message that should be published next.
     */
    robosub::thruster tp;

    /*
     * The previous quaternion orientation message received.
     */
    geometry_msgs::QuaternionStamped prev_quat_msg;

    /*
     * The previous depth message received.
     */
    robosub::depth_stamped prev_depth_msg;
};
#endif
