#include "localization/lin_accel_kalman_filter.h"
#include <eigen3/Eigen/Dense>

using namespace Eigen;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dead_reckoning");

    ros::NodeHandle nh;

    LinAccelKalmanFilter kfilter(nh);

    // Service for resetting position and velocity
    ros::ServiceServer reset_filter_service =
        nh.advertiseService("localization/reset_kalman_filter",
                            &LinAccelKalmanFilter::reset, &kfilter);

    ros::Subscriber accel_sub = nh.subscribe("acceleration/linear", 1,
                               &LinAccelKalmanFilter::InputLinAccel, &kfilter);
    ros::Subscriber orientation_sub = nh.subscribe("orientation", 1,
                            &LinAccelKalmanFilter::InputOrientation, &kfilter);
    ros::Subscriber depth_sub = nh.subscribe("depth", 1,
                                &LinAccelKalmanFilter::InputDepth, &kfilter);

    ros::spin();

    return 0;
}
