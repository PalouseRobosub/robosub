#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

using namespace sensor_msgs;
using namespace message_filters;

void callback(const ImageConstPtr& image1, const ImageConstPtr& image2)
{
    std::cout << "Starting Stereo Calibration" << std::endl;

    cv::FileStorage fsl("../../param/left_camera.yaml", cv::FileStorage::READ);
    cv::FileStorage fsr("../../param/right_camera.yaml", cv::FileStorage::READ);


    cv::Mat K1, K2, R, F, E;
    cv::Vec3d T;
    cv::Mat D1, D2;
    fsl["camera_matrix"] >> K1;
    fsr["camera_matrix"] >> K2;
    fsl["distortion_coefficients"] >> D1;
    fsr["distortion_coefficients"] >> D2;
    int flag = 0;
    flag |= CV_CALIB_FIX_INTRINSIC;

    std::cout << "Read intrinsics" << std::endl;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stereo_vision");

    ros::NodeHandle nh;
    message_filters::Subscriber<Image> image1_sub(nh,
                                       "camera/left/undistorted", 1);
    message_filters::Subscriber<Image> image2_sub(nh,
                                       "camera/right/undistorted", 1);
    typedef sync_policies::ApproximateTime<Image, Image> SyncPolicy;

    Synchronizer<SyncPolicy> sync(SyncPolicy(10), image1_sub, image2_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    return 0;

}
