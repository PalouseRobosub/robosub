#include "localization/lin_accel_kalman_filter.h"

LinAccelKalmanFilter::LinAccelKalmanFilter(ros::NodeHandle *_nh)
{
    nh = _nh;
    initialize();
}

bool LinAccelKalmanFilter::NewAbsLinVel()
{
    return new_abs_lin_velocity;
}

tf::Vector3 LinAccelKalmanFilter::GetAbsLinVel()
{
    new_abs_lin_velocity = false;
    return tf::Vector3(x(3, 0), x(4, 0),
                       x(5, 0));
}

double LinAccelKalmanFilter::GetAbsLinVelDT()
{
    return abs_lin_velocity_dt;
}

void LinAccelKalmanFilter::InputPosition(tf::Vector3 position, double dt)
{
    obs(4, 0) = position[0];
    obs(5, 0) = position[1];
    obs(6, 0) = position[2];

    position_dt = dt;

    new_position = true;
}

void LinAccelKalmanFilter::InputAbsLinAcl(tf::Vector3 lin_acl, double dt)
{
    obs(0, 0) = lin_acl[0];
    obs(1, 0) = lin_acl[1];
    obs(2, 0) = lin_acl[2];

    update_A(dt);

    update();
}

void LinAccelKalmanFilter::InputDepth(double depth, double dt)
{
    obs(3, 0) = depth;
}

void LinAccelKalmanFilter::Reset()
{
    initialize();
}

void LinAccelKalmanFilter::initialize()
{
    num_iterations = 0;

    // Zero everything.
    x.setZero();
    P.setZero();
    A.setZero();
    Q.setZero();
    H.setZero();
    R.setZero();
    obs.setZero();

    // Set state to observation matrix.
    // These entries convert lin acl state to lin acl measurement
    H(0, 6) = H(1, 7) = H(2, 8) = 1.0;
    // These entries convert depth state to depth measurement
    H(3, 2) = 1.0;
    // These entries convert position state to position measurement
    H(4, 0) = H(5, 1) = H(6, 2) = 1.0;

    // Load all necessary parmeters.
    if(!ros::param::has("localization/"))
    {
        ROS_FATAL("localization parameters not found");
        ros::shutdown();
    }

    getParamCachedMatrix("localization/kalman_filter/X0", x);
    getParamCachedMatrix("localization/kalman_filter/P", P);
    reload_params();

    ROS_INFO_STREAM("H:\n" << H);
    ROS_INFO_STREAM("Q:\n" << Q);
    ROS_INFO_STREAM("R:\n" << R);

    ROS_INFO_STREAM("P initial:\n" << P);
    ROS_INFO_STREAM("X initial:\n" << x);

    // Intialize output lin velocity metadata.
    new_abs_lin_velocity = false;
    abs_lin_velocity_dt = 0.0;
    last_abs_lin_velocity_time = ros::Time::now();

    // Intialize inputted position metadata.
    new_position = false;
    position_dt = 0.0;

    ROS_INFO_STREAM("Finished KF init");
}

void LinAccelKalmanFilter::reload_params()
{
    getParamCachedMatrix("localization/kalman_filter/Q", Q);
    getParamCachedMatrix("localization/kalman_filter/R", R);

    PRINT_THROTTLE(ROS_DEBUG_STREAM("Q:\n" << Q););
    PRINT_THROTTLE(ROS_DEBUG_STREAM("R:\n" << R););
}

void LinAccelKalmanFilter::update_A(double dt)
{
    // Update system update matrix with dt.
    A.setIdentity();
    A(0, 3) = A(1, 4) = A(2, 5) = A(3, 6) = A(4, 7) = A(5, 8) = dt;
    A(0, 6) = A(1, 7) = A(2, 8) = 0.5 * (dt * dt);
}

void LinAccelKalmanFilter::run_filter()
{
    // If a position message has been inputted tell the filter we have taken
    // into account the position information.
    if(new_position)
    {
        new_position = false;
    }
    // If no position message has been inputted, tell the filter to ignore the
    // position portion of the obs matrix. To do this set the position
    // measurement variances extremely high. R is reloaded every iteration so
    // no need bother resetting R to its previous value.
    else
    {
        R(4, 4) = R(5, 5) = R(6, 6) = 100000000000000000.0;
    }

    // Start of kalman filter main loop.
    // TODO: Explain sources of kfilter equation.
    PRINT_THROTTLE(ROS_DEBUG_STREAM("**Kalman Main Loop**"););

    // Push state forward in time.
    x = A * x;
    PRINT_THROTTLE(ROS_DEBUG_STREAM("x predicted:\n" << x););

    // Estimate prediction covariance.
    P = A * P * A.transpose() + Q;
    PRINT_THROTTLE(ROS_DEBUG_STREAM("P predicted:\n" << P););

    // Get error between sensor inputs and predicted based on current state.
    Matrix<double, 7, 1> y;
    y = obs - H * x;
    PRINT_THROTTLE(ROS_DEBUG_STREAM("y:\n" << y););

    // Use observed covariance matrix (P) to modify known sensor noises.
    Matrix<double, 7, 7> s;
    s = H * P * H.transpose() + R;
    PRINT_THROTTLE(ROS_DEBUG_STREAM("s:\n" << s););

    // Find kalman gain.
    Matrix<double, 9, 7> k;
    k = P * H.transpose() * s.inverse();
    PRINT_THROTTLE(ROS_DEBUG_STREAM("k:\n" << k););

    // Update predicted state with kalman gain and error between sensors and
    // prediction.
    x = x + k * y;
    PRINT_THROTTLE(ROS_DEBUG_STREAM("x corrected:\n" << x););

    // Update predicted covariances with kalman gain.
    P = (Matrix<double, 9, 9>::Identity() - k * H) * P;
    PRINT_THROTTLE(ROS_DEBUG_STREAM("P corrected:\n" << P););

    // End of kalman filter main loop.

    // Set necessary linear velocity information.
    new_abs_lin_velocity = true;
    abs_lin_velocity_dt = (ros::Time::now() -
                           last_abs_lin_velocity_time).toSec();
    last_abs_lin_velocity_time = ros::Time::now();
}

void LinAccelKalmanFilter::update()
{
    reload_params();

    run_filter();

    num_iterations++;
}

