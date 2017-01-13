#include "lin_accel_kalman_filter.h"

LinAccelKalmanFilter::LinAccelKalmanFilter(ros::NodeHandle _nh)
{
    nh = _nh;
    pub = nh.advertise<geometry_msgs::Vector3Stamped>("dead_reckoning", 1);

    initialize();
    reload_params();
}

LinAccelKalmanFilter::~LinAccelKalmanFilter()
{
}

void LinAccelKalmanFilter::initialize()
{
    num_iterations = 0;

    ROS_INFO_STREAM("Finished KF init");
}

void LinAccelKalmanFilter::reload_params()
{
    if(!ros::param::has("localization/"))
    {
        ROS_FATAL("localization parameters not found");
        ros::shutdown();
    }

    /*
    ros::param::getCached("localization/initial_state/x_pos", initial_state(0,0));
    ros::param::getCached("localization/initial_state/y_pos", initial_state(1,0));
    ros::param::getCached("localization/initial_state/z_pos", initial_state(2,0));
    ros::param::getCached("localization/initial_state/x_lin_vel", initial_state(3,0));
    ros::param::getCached("localization/initial_state/y_lin_vel", initial_state(4,0));
    ros::param::getCached("localization/initial_state/z_lin_vel", initial_state(5,0));

    */
}

bool LinAccelKalmanFilter::reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &rep)
{
    initialize();
    reload_params();

    return true;
}

tf::Vector3 LinAccelKalmanFilter::calculate_absolute_lin_accel(tf::Vector3 rel_lin_accel, tf::Quaternion orientation)
{
    tf::Quaternion orientation_conjugate;
    orientation_conjugate.setX(orientation.getX() * -1.0);
    orientation_conjugate.setY(orientation.getY() * -1.0);
    orientation_conjugate.setZ(orientation.getZ() * -1.0);
    orientation_conjugate.setW(orientation.getW());

    tf::Matrix3x3 rot_m = tf::Matrix3x3(orientation_conjugate);

    tf::Vector3 abs_lin_accel = rot_m * rel_lin_accel;

    return abs_lin_accel;
}

void LinAccelKalmanFilter::InputLinAccel(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    last_lin_accel_time = msg->header.stamp;
    dt = msg->header.stamp - last_lin_accel_time;

    tf::Vector3 rel_lin_accel;
    rel_lin_accel[0] = msg->vector.x;
    rel_lin_accel[1] = msg->vector.y;
    rel_lin_accel[2] = msg->vector.z;

    last_lin_accel_time = msg->header.stamp;

    tf::Vector3 abs_lin_accel = calculate_absolute_lin_accel(rel_lin_accel, orientation);

    Matrix<double,3,1> obs;
    obs(0,0) = abs_lin_accel[0];
    obs(1,0) = abs_lin_accel[1];
    obs(2,0) = abs_lin_accel[2];

    update(obs, dt.toSec());
}

void LinAccelKalmanFilter::InputOrientation(const robosub::QuaternionStampedAccuracy::ConstPtr &msg)
{
    orientation.setX(msg->quaternion.x);
    orientation.setY(msg->quaternion.y);
    orientation.setZ(msg->quaternion.z);
    orientation.setW(msg->quaternion.w);
}

void LinAccelKalmanFilter::update_A(double dt)
{

}

Matrix<double, 9,1> LinAccelKalmanFilter::run_filter(Matrix<double,3,1> obs, double dt)
{
    // predict state forward
    x = A * x_prev;
    // estimate prediction covariance
    P = A * P * A.transpose() + Q;
    // get error between reality and prediction
    y = obs - H * x;
    // add real error to predicted probability
    s = H * P * H.transpose() + R;
    // find kalman gain
    k = P * H.transpose() * s.inverse();
    // update predicted state with kalman gain
    x = x + k * y;
    // update predicted covariances with kalmain gain
    P = (Matrix<double,9,9>::Identity() - k * H) * P;

    return x;
}

void LinAccelKalmanFilter::update(Matrix<double,3,1> obs, double dt)
{
    PRINT_THROTTLE(ROS_INFO_STREAM("=================================="););

    reload_params();

    update_A(dt);

    run_filter(obs, dt);

    publish();

    num_iterations++;
}

void LinAccelKalmanFilter::publish()
{
    geometry_msgs::Vector3Stamped msg;
    msg.vector.x = x(0,0);
    msg.vector.y = x(1,0);
    msg.vector.z = x(2,0);

    msg.header.stamp = ros::Time::now();

    pub.publish(msg);
}
