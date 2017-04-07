#ifndef FILTER_UTILITIES_H
#define FILTER_UTILITIES_H

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <string>
#include <random>
#include <vector>

#include "ros/console.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "XmlRpcException.h"

using namespace Eigen;

#define PRINT_THROTTLE_RATE 5
#define PRINT_THROTTLE(x) if(num_iterations % PRINT_THROTTLE_RATE == 0) { x }

constexpr double PI = (3.1415);
constexpr double RAD_TO_DEG = (180.0 / PI);
constexpr double DEG_TO_RAD = (PI / 180.0);

namespace filter_utilities
{
typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 9, 1> Vector9d;

std::default_random_engine rand_generator;
std::normal_distribution<double> norm_distribution(0.0, 1.0);
std::uniform_real_distribution<double> uniform_distribution(0.0, 1.0);

// Loads a nXm eigen matrix from a ros param consisting of an array of n lists
// of m numbers. n and m are determined by dimensions of input matrix.
bool getParamCachedMatrix(std::string param_name,
        Eigen::Ref<Eigen::MatrixXd> mat)
{
    // Load param.
    XmlRpc::XmlRpcValue param;
    if(!ros::param::getCached(param_name, param))
    {
        ROS_FATAL_STREAM("Param " << param_name << " not found.");
        exit(1);
    }

    int nrows = mat.rows();
    int ncols = mat.cols();

    if(param.size() != nrows)
    {
        ROS_WARN_STREAM("number of rows of param " << param_name <<
                " does not match number of rows of input matix");
        return false;
    }

    for(int i = 0; i < nrows; i++)
    {
        // Grab i'th row of param.
        XmlRpc::XmlRpcValue row;
        try
        {
            row = param[i];
        }
        catch(const XmlRpc::XmlRpcException& e)
        {
            ROS_FATAL_STREAM(e.getMessage());
            exit(1);
        }

        for(int j = 0; j < ncols; j++)
        {
            if(row.size() != ncols)
            {
                ROS_WARN_STREAM("number of columns of param " << param_name <<
                        " does not match number of columns of input matix");
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
            else
            {
                // See here for Type decoding: https://goo.gl/X7ey2d
                ROS_FATAL_STREAM("param " << param_name << " contains a" <<
                        " non-integer or non-double value (row: " << i
                        << " column: " << j << "): " << row[j] << " (type: " <<
                        row[j].getType() << ")");
                exit(1);
            }
        }
    }

    return true;
}

// This is the gaussian probability density function.
double gaussian_prob(double mean, double sigma, double x)
{
    return std::exp(- std::pow((mean - x), 2) / std::pow(sigma, 2) * 2.0) /
        std::sqrt(2.0 * PI * std::pow(sigma, 2));
}

// This is meant to emulate the matlab function randn which
// returns a random value from a normal distribution with
// mean = 0 std dev = 1
double rand_normal()
{
    return norm_distribution(rand_generator);
}

// Uniform 0-1 random real
double rand_uniform()
{
    return uniform_distribution(rand_generator);
}

// Returns a matrix with all elements equal to rand_normal().
MatrixXd randn_mat(int rows, int cols)
{
    MatrixXd r(rows, cols);
    for(int i = 0; i < rows; i++)
    {
        for(int j = 0; j < cols; j++)
        {
            r(i, j) = rand_normal();
        }
    }
    return r;
}

};

#endif //FILTER_UTILITIES_H
