#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <random>
#include <vector>

#include "ros/console.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"

#include "robosub/QuaternionStampedAccuracy.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_srvs/Empty.h"

using namespace Eigen;

#define PT_RATE 20
#define PRINT_THROTTLE(x) if(num_iterations % PT_RATE == 0) { x }
//#define PRINT_THROTTLE(x) if(0 && num_iterations % PT_RATE == 0) { x }

class LinAccelKalmanFilter
{
public:
    LinAccelKalmanFilter(ros::NodeHandle _nh);
    ~LinAccelKalmanFilter();
    bool reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &rep);

    void InputLinAccel(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
    void InputOrientation(const robosub::QuaternionStampedAccuracy::ConstPtr &msg);

private:
    void initialize();
    void reload_params();
    tf::Vector3 calculate_absolute_lin_accel(tf::Vector3 rel_lin_accel, tf::Quaternion orientation);
    void update_A(double dt);
    Matrix<double, 9,1> run_filter(Matrix<double,3,1> obs);
    void update(Matrix<double,3,1> obs, double dt);
    void publish(Matrix<double,9,1> predicted_state);

    ros::NodeHandle nh;
    ros::Publisher pub;

    int num_iterations;
    ros::Time last_lin_accel_time;
    ros::Duration dt;
    tf::Quaternion orientation;
    bool orientation_received;

    Matrix<double,9,1> x;
    Matrix<double,9,1> x_prev;

    Matrix<double,9,9> P;

    Matrix<double,9,9> A;
    Matrix<double,9,8> B;
    Matrix<double,9,9> Q;
    Matrix<double,3,9> H;
    Matrix<double,3,3> R;

    Matrix<double,9,3> k;
    Matrix<double,3,1> y;
    Matrix<double,3,3> s;
};
