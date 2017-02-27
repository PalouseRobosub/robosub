#include <ros/ros.h>
#include "wfov_camera_msgs/WFOVImage.h"
// The following #if is to use the correct version of the cv_bridge
// Kinetic by default uses OpenCV3 so we don't need the custom build
#if ROS_VERSION_MINIMUM(1, 12, 0)  // Running Kinetic
    #include <cv_bridge/cv_bridge.h>
#else  // Running Indigo
    #include <cv3_bridge/cv_bridge.h>
#endif
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/calib3d.hpp>
#include <string>

using namespace cv_bridge;
using namespace cv;
using std::string;

image_transport::Publisher rightPub;
image_transport::Publisher leftPub;
image_transport::Publisher bottomPub;

Mat rectifyMap[2][2];
Mat bottomCamMat;
Mat bottomDistCoeffs;

void rightCallback(const wfov_camera_msgs::WFOVImage::ConstPtr& msg)
{
    ROS_DEBUG_STREAM("Right Cam Callback.");

    cv_bridge::CvImagePtr image_ptr;
    image_ptr = toCvCopy(msg->image, sensor_msgs::image_encodings::BGR8);

    Mat temp = image_ptr->image.clone();

    remap(temp, image_ptr->image, rectifyMap[0][0], rectifyMap[0][1],
          INTER_LINEAR);

    sensor_msgs::Image outMsg;

    image_ptr->toImageMsg(outMsg);

    rightPub.publish(outMsg);
}

void leftCallback(const wfov_camera_msgs::WFOVImage::ConstPtr &msg)
{
    ROS_DEBUG_STREAM("Right Cam Callback.");

    cv_bridge::CvImagePtr image_ptr;
    image_ptr = toCvCopy(msg->image, sensor_msgs::image_encodings::BGR8);

    Mat temp = image_ptr->image.clone();

    remap(temp, image_ptr->image, rectifyMap[0][0], rectifyMap[0][1],
          INTER_LINEAR);

    sensor_msgs::Image outMsg;

    image_ptr->toImageMsg(outMsg);

    leftPub.publish(outMsg);
}

void bottomCallback(const wfov_camera_msgs::WFOVImage::ConstPtr &msg)
{
    cv_bridge::CvImagePtr image_ptr;
    image_ptr = toCvCopy(msg->image, sensor_msgs::image_encodings::BGR8);

    Mat temp = image_ptr->image.clone();

    fisheye::undistortImage(temp, image_ptr->image, bottomCamMat,
                            bottomDistCoeffs, Matx33d::eye());

    sensor_msgs::Image outMsg;

    image_ptr->toImageMsg(outMsg);

    bottomPub.publish(outMsg);
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "undistMono");

    ROS_INFO_STREAM("Init done");

    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    ROS_INFO_STREAM("Creating subscribers");
    ros::Subscriber leftSub = n.subscribe("/camera/left/image", 1,
                                           leftCallback);

    ros::Subscriber rightSub = n.subscribe("/camera/right/image", 1,
                                           rightCallback);

    ros::Subscriber bottomSub = n.subscribe("/camera/bottom/image", 1,
                                            bottomCallback);

    ROS_INFO_STREAM("Subscribers created");

    leftPub = it.advertise("camera/left/undistorted", 1);
    rightPub = it.advertise("camera/right/undistorted", 1);
    bottomPub = it.advertise("camera/bottom/undistorted", 1);

    ROS_INFO_STREAM("Publishers created");
    string stereoCalibFile = "";
    n.getParam("stereo_calib_file", stereoCalibFile);

    string bottomCalibFile = "";
    n.getParam("bottom_calib_file", bottomCalibFile);

    ROS_INFO_STREAM("Fetched params");
    ROS_INFO_STREAM("Reading stereo calibration file: " << stereoCalibFile);

    FileStorage fs(stereoCalibFile, FileStorage::READ);

    Mat K1, K2, D1, D2, R1, R2, P1, P2;
    Size imageSize;

    if (!fs.isOpened())
    {
        ROS_FATAL_STREAM("Stereo calibration file could not be opened from: " <<
                         stereoCalibFile);
        exit(1);
    }
    else
    {
        ROS_INFO_STREAM("Valid calibration file");
        string calibTime;
        fs["calibration_time"] >> calibTime;
        ROS_INFO_STREAM("Calibration was performed on: " << calibTime);
        fs["K1"] >> K1;
        fs["D1"] >> D1;
        fs["K2"] >> K2;
        fs["D2"] >> D2;
        fs["R1"] >> R1;
        fs["R2"] >> R2;
        fs["P1"] >> P1;
        fs["P2"] >> P2;
        fs["image_size"] >> imageSize;
    }

    ROS_INFO_STREAM("Stereo calibration file read");

    ROS_INFO_STREAM("Reading bottom calibration file: " << bottomCalibFile);

    fs.open(bottomCalibFile, FileStorage::READ);

    if (!fs.isOpened())
    {
        ROS_FATAL_STREAM("Bottom calibration file could not be opened from: " <<
                         bottomCalibFile);
        exit(2);
    }
    else
    {
        ROS_INFO_STREAM("Valid calibration file");
        string calibTime;
        fs["calibration_time"] >> calibTime;
        ROS_INFO_STREAM("Calibration was performed on: " << calibTime);
        fs["camera_matrix"] >> bottomCamMat;
        fs["distortion_coefficients"] >> bottomDistCoeffs;
    }

    ROS_INFO_STREAM("Bottom calibration file read");

    ROS_INFO_STREAM("Initializing stereo undistortion maps");

    fisheye::initUndistortRectifyMap(K1, D2, R1, P1,
                                     imageSize, CV_16SC2, rectifyMap[0][0],
                                     rectifyMap[0][1]);

    fisheye::initUndistortRectifyMap(K2, D2, R2, P2,
                                     imageSize, CV_16SC2, rectifyMap[1][0],
                                     rectifyMap[1][1]);

    ROS_INFO_STREAM("Completed initialization");

    ros::spin();
}
