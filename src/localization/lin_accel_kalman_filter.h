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

using namespace Eigen;

#define KF_PT_RATE 10
#define KF_PRINT_THROTTLE(x) if(num_iterations % KF_PT_RATE == 0) { x }
//#define KF_PRINT_THROTTLE(x) if(0 && num_iterations % KF_PT_RATE == 0) { x }

bool getParamCachedMatrix(std::string param_name,
                          Eigen::Ref<Eigen::MatrixXd> mat)
{
    XmlRpc::XmlRpcValue param;
    if(!ros::param::getCached(param_name, param))
    {
        return false;
    }

    int nrows = mat.rows();
    int ncols = mat.cols();

    if(param.size() != nrows)
    {
        ROS_WARN_STREAM("number of rows of param " << param_name <<
                        " does not match number of rows of inputted matix");
        return false;
    }

    int i = 0;
    int j = 0;
    for(i = 0; i < nrows; i++)
    {
        XmlRpc::XmlRpcValue row = param[i];

        for(j = 0; j < ncols; j++)
        {
            if(row.size() != ncols)
            {
                ROS_WARN_STREAM("number of columns of param " << param_name <<
                                " does not match number of columns of inputted matix");
                return false;
            }

            if(row[j].getType() == XmlRpc::XmlRpcValue::TypeDouble)
            {
                mat(i, j) = static_cast<double>(row[j]);
            }
            else if(row[j].getType() == XmlRpc::XmlRpcValue::TypeInt)
            {
                mat(i, j) = static_cast<double>(static_cast<int>(row[j]));
            }
        }
    }

    return true;
}

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
    Matrix<double, 9, 1> run_filter(Matrix<double, 7, 1> obs);
    void update(Matrix<double, 7, 1> obs, double dt);

    ros::NodeHandle *nh;

    int num_iterations;

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
