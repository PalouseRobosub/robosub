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

#include "filter_utilities.h"

using namespace Eigen;
using namespace filter_utilities;

class LinAccelKalmanFilter
{
public:
    LinAccelKalmanFilter(ros::NodeHandle *_nh);
    ~LinAccelKalmanFilter();

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

    ros::NodeHandle *nh;

    int num_iterations;
    double lin_acl_dt;

    bool new_abs_lin_velocity;
    double abs_lin_velocity_dt;
    ros::Time last_abs_lin_velocity_time;

    bool new_position;
    double position_dt;

    Matrix<double, 7, 1> obs;

    Matrix<double, 9, 1> x0;
    Matrix<double, 9, 1> x;
    Matrix<double, 9, 1> x_prev;

    Matrix<double, 9, 9> P;

    Matrix<double, 9, 9> A;
    Matrix<double, 9, 8> B;
    Matrix<double, 9, 9> Q;
    Matrix<double, 7, 9> H;
    Matrix<double, 7, 7> R;

    Matrix<double, 9, 7> k;
    Matrix<double, 7, 1> y;
    Matrix<double, 7, 7> s;
};
#endif //LIN_ACCEL_KALMAN_FILTER_H
