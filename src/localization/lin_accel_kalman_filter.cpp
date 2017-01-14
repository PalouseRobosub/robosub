#include "lin_accel_kalman_filter.h"

LinAccelKalmanFilter::LinAccelKalmanFilter(ros::NodeHandle _nh)
{
    nh = _nh;
    pub = nh.advertise<robosub::Float32ArrayStamped>("dead_reckoning", 1);

    initialize();
}

LinAccelKalmanFilter::~LinAccelKalmanFilter()
{
}

void LinAccelKalmanFilter::initialize()
{
    num_iterations = 0;

    x.setZero();
    x_prev.setZero();
    P.setZero();
    A.setZero();
    B.setZero();
    Q.setZero();
    H.setZero();
    R.setZero();
    k.setZero();
    y.setZero();
    s.setZero();

    A.setIdentity();
    update_A(1.0/33.0);

    H(0,6) = H(1,7) = H(2,8) = 1.0;
    H(3,2) = 1.0;

    reload_params();

    x(0,0) = x_prev(0,0) = x0(0,0);
    x(1,0) = x_prev(1,0) = x0(1,0);
    x(2,0) = x_prev(2,0) = x0(2,0);

    orientation_initialized = false;
    depth_initialized = false;

    ROS_INFO_STREAM("Finished KF init");
}

void LinAccelKalmanFilter::reload_params()
{
    if(!ros::param::has("localization/"))
    {
        ROS_FATAL("localization parameters not found");
        ros::shutdown();
    }

    getParamCachedMatrix("kalman_filter/Q", Q);
    getParamCachedMatrix("kalman_filter/R", R);
    getParamCachedMatrix("kalman_filter/P", P);
    getParamCachedMatrix("kalman_filter/X0", x0);

    PRINT_THROTTLE(ROS_INFO_STREAM("Q: " << Q););
    PRINT_THROTTLE(ROS_INFO_STREAM("R: " << R););
    PRINT_THROTTLE(ROS_INFO_STREAM("P: " << P););
    PRINT_THROTTLE(ROS_INFO_STREAM("x0: " << x0););
}

bool LinAccelKalmanFilter::reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &rep)
{
    initialize();

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
    PRINT_THROTTLE(ROS_INFO_STREAM("=================================="););

    dt = msg->header.stamp - last_lin_accel_time;
    last_lin_accel_time = msg->header.stamp;

    tf::Vector3 rel_lin_accel;
    rel_lin_accel[0] = msg->vector.x;
    rel_lin_accel[1] = msg->vector.y;
    rel_lin_accel[2] = msg->vector.z;

    if(orientation_initialized && depth_initialized)
    {
        tf::Vector3 abs_lin_accel = calculate_absolute_lin_accel(rel_lin_accel, orientation);

        Matrix<double,4,1> obs;
        obs(0,0) = abs_lin_accel[0];
        obs(1,0) = abs_lin_accel[1];
        obs(2,0) = abs_lin_accel[2];
        // TODO: Load param
        double pinger_depth = -4.1;
        obs(3,0) = -1.0 * (pinger_depth - depth);

        update(obs, dt.toSec());
    }
}

void LinAccelKalmanFilter::InputOrientation(const robosub::QuaternionStampedAccuracy::ConstPtr &msg)
{
    orientation.setX(msg->quaternion.x);
    orientation.setY(msg->quaternion.y);
    orientation.setZ(msg->quaternion.z);
    orientation.setW(msg->quaternion.w);

    orientation_initialized = true;
}

void LinAccelKalmanFilter::InputDepth(const robosub::depth_stamped::ConstPtr &msg)
{
    depth = msg->depth;

    depth_initialized = true;
}

void LinAccelKalmanFilter::update_A(double dt)
{
    A(0,3) = A(1,4) = A(2,5) = A(3,6) = A(4,7) = A(5,8) = dt;
    A(0,6) = A(1,7) = A(2,8) = 0.5 * (dt * dt);

    PRINT_THROTTLE(ROS_INFO_STREAM("A:\n" << A););
}

Matrix<double, 9,1> LinAccelKalmanFilter::run_filter(Matrix<double,4,1> obs)
{
    PRINT_THROTTLE(ROS_INFO_STREAM("****************************"););

    // predict state forward
    x = A * x_prev;
    PRINT_THROTTLE(ROS_INFO_STREAM("x:\n" << x););
    // estimate prediction covariance
    P = A * P * A.transpose() + Q;
    PRINT_THROTTLE(ROS_INFO_STREAM("P:\n" << P););
    // get error between reality and prediction
    y = obs - H * x;
    PRINT_THROTTLE(ROS_INFO_STREAM("y:\n" << y););
    // add real error to predicted probability
    s = H * P * H.transpose() + R;
    PRINT_THROTTLE(ROS_INFO_STREAM("s:\n" << s););
    // find kalman gain
    k = P * H.transpose() * s.inverse();
    PRINT_THROTTLE(ROS_INFO_STREAM("k:\n" << k););
    // update predicted state with kalman gain
    x = x + k * y;
    PRINT_THROTTLE(ROS_INFO_STREAM("x:\n" << x););
    // update predicted covariances with kalmain gain
    P = (Matrix<double,9,9>::Identity() - k * H) * P;
    PRINT_THROTTLE(ROS_INFO_STREAM("P:\n" << P););

    PRINT_THROTTLE(ROS_INFO_STREAM("****************************"););

    x_prev = x;

    return x;
}

void LinAccelKalmanFilter::update(Matrix<double,4,1> obs, double dt)
{
    PRINT_THROTTLE(ROS_INFO_STREAM("obs: " << obs););

    reload_params();

    update_A(dt);

    Matrix<double, 9,1> predicted_state = run_filter(obs);

    PRINT_THROTTLE(ROS_INFO_STREAM("predicted_state:\n" << predicted_state););

    publish(predicted_state);

    num_iterations++;
}

// TODO: Publish whole state
void LinAccelKalmanFilter::publish(Matrix<double,9,1> predicted_state)
{
    robosub::Float32ArrayStamped msg;

    for(int i=0; i<9; i++)
    {
        msg.data.push_back(predicted_state(i,0));
    }

    msg.header.stamp = ros::Time::now();

    pub.publish(msg);
}
