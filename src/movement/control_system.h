#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <string>
#include <deque>

#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/AccelStamped.h"
#include "robosub_msgs/control.h"
#include "robosub_msgs/control_status.h"
#include "robosub_msgs/Float32Stamped.h"
#include "robosub_msgs/thruster.h"
#include "ros/ros.h"
#include "rotation_engine.hpp"
#include "tf/transform_datatypes.h"

using namespace Eigen;
using std::string;

namespace robosub
{
class ControlSystem
{
    /*
     * Define custom types for using Eigen.
     */
    using Vector6d = Matrix<double, 6, 1>;
    using Vector6 = Matrix<int, 6, 1>;
    using Vector12d = Matrix<double, 12, 1>;
    using Matrix6d = Matrix<double, 6, 6>;

    /*
     * Defines conversion scale for converting between radians and degrees.
     */
    static constexpr double _PI_OVER_180 = (3.1415 / 180.0);

    static constexpr double _180_OVER_PI = (180.0 / 3.14159);

    /*
     * Define a custom type for storing a state measurement
     */
    struct StateMeasurement
    {
        StateMeasurement(const ros::Time _time, const double _val) :
            time(_time),
            val(_val)
        {
        }

        /*
         * The time at which the measurement was taken.
         */
        const ros::Time time;

        /*
         * The value of the measurement
         */
        const double val;
    };

public:
    ControlSystem();
    void InputControlMessage(const robosub_msgs::control::ConstPtr& msg);
    void InputOrientationMessage(
            const geometry_msgs::QuaternionStamped::ConstPtr& quat_msg);
    void InputLocalizationMessage(
            const geometry_msgs::PointStamped::ConstPtr& point_msg);
    void InputDepthMessage(
            const robosub_msgs::Float32Stamped::ConstPtr& depth_msg);
    void CheckTimeout(const ros::TimerEvent& timer_event);
    void ReloadPIDParams();
    robosub_msgs::thruster CalculateThrusterMessage();
    robosub_msgs::thruster GetZeroThrusterMessage();
    robosub_msgs::control_status GetControlStatus();
    geometry_msgs::AccelStamped GetAccelerationEstimate();

    bool isEnabled();
    void setEnabled(bool enable);

private:
    bool enabled;
    void calculate_motor_control();
    double wraparound(double x, double min, double max);
    std::string state_to_string(uint8_t state);
    ros::Time last_msg_time;

    /*
     * Defines the current goals of the submarine in the order of X, Y, PSI,
     * PHI, and THETA.
     */
    Vector6d goals;

    /*
     * Defines the type of goal as specified in the robosub_msgs::control::goal
     * enumeration.
     */
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

    /*
     * Defines the control PID parameters and the physical constants of the
     * submarine.
     */
    Vector6d P, I, D;
    Vector6d windup, hysteresis;
    Vector6d sub_mass;
    double buoyancy_offset = 0.0;

    /*
     * Define the motor matrix (inverted) used to solve for the motor commands.
     */
    MatrixXd motors_inverted;

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
     * Defines the current state of the submarine. This vector stores elements
     * in the following order:
     *      X, Y, Z, Psi (roll), Phi (pitch), Theta (yaw)
     */
    Vector6d state_vector;

    /*
     * Defines the number of thrusters on the submarine.
     */
    int num_thrusters;

    /*
     * Boolean vector indicating that new state measurements are available.
     */
    Matrix<bool, 6, 1> new_measurement_available;

    /*
     * The previous errors calculated.
     */
    Matrix<std::deque<StateMeasurement>, 6, 1> previous_error;

    /*
     * The previously calculated acceleration exerted on the submarine.
     */
    Vector6d acceleration_estimate;
};
}
#endif // CONTROL_SYSTEM_H
