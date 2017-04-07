#ifndef LIN_ACCEL_KALMAN_FILTER_H
#define LIN_ACCEL_KALMAN_FILTER_H

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <string>
#include <random>
#include <vector>

#include "ros/console.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"

#include "localization/filter_utilities.h"

using namespace Eigen;
using namespace filter_utilities;

class LinAccelKalmanFilter
{
public:
    LinAccelKalmanFilter(ros::NodeHandle &_nh);

    bool NewAbsLinVel();
    tf::Vector3 GetAbsLinVel();
    double GetAbsLinVelDT();
    void InputPosition(tf::Vector3 position, double dt);
    void InputAbsLinAcl(tf::Vector3 lin_acl, double dt);
    void InputDepth(double depth, double dt);
    void Reset();

private:
    void initialize();
    void reload_params();
    void update_A(double dt);
    void run_filter();
    void update();

    // Nodehandle received from localization_system
    ros::NodeHandle &nh;

    // Number of times update step has been run.
    int num_iterations;

    // Stores estimated linear velocity metadata.
    bool new_abs_lin_velocity;
    double abs_lin_velocity_dt;
    ros::Time last_abs_lin_velocity_time;

    // Stores necessary position input metadata.
    bool new_position;
    // This is unused but I'm leaving it here in case it becomes necessary.
    double position_dt;

    // Stores state information as column vector.
    // The state is as follows:
    // [ x_pos |
    // | y_pos |
    // | z_pos |
    // | x_vel |
    // | y_vel |
    // | z_vel |
    // | x_acl |
    // | y_acl |
    // | z_acl |
    Vector9d x;

    // Stores most current linear acceleration, depth, and position data as
    // column vector as follows:
    // | x_acl |
    // | y_acl |
    // | z_acl |
    // | depth |
    // | x_pos |
    // | y_pos |
    // | z_pos |
    Vector7d observation;

    // Stores system update matrix.
    Matrix<double, 9, 9> A;

    // Stores state to observation matrix.
    // The first 3 rows convert lin acl state to lin acl measurement
    // The 4th row converts depth state to depth measurement
    // The final 3 rows convert position state to position measurement
    Matrix<double, 7, 9> H;

    // Stores predicted error covariances of the overall filter prediction.
    // Initial values loaded from params.
    Matrix<double, 9, 9> P;

    // Stores covariance matrix of system update errors. Loaded from params.
    Matrix<double, 9, 9> Q;

    // Stores covariance matrix of observation errors. Loaded from params.
    Matrix<double, 7, 7> R;
};
#endif //LIN_ACCEL_KALMAN_FILTER_H
