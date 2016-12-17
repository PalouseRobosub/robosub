#include "wfov_camera_msgs/WFOVImage.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/calib3d.hpp>
#include <string>

using namespace cv_bridge;
using namespace cv;
using std::string;

image_transport::Publisher rightPub;

Mat camMat;
Mat distCoeffs;

bool isCalibrated = false;
bool useFisheye = false;

void rightCallback(const wfov_camera_msgs::WFOVImage::ConstPtr& msg)
{
    ROS_DEBUG_STREAM("Right Cam Callback.");

    cv_bridge::CvImagePtr image_ptr;
    image_ptr = toCvCopy(msg->image, sensor_msgs::image_encodings::BGR8);

    Mat temp = image_ptr->image.clone();

    if (isCalibrated)
    {
        if (useFisheye)
        {
            fisheye::undistortImage(temp, image_ptr->image, camMat, distCoeffs,
                                    Matx33d::eye());
        }
        else
        {
            undistort(temp, image_ptr->image, camMat, distCoeffs);
        }
    }

    sensor_msgs::Image outMsg;

    image_ptr->toImageMsg(outMsg);

    rightPub.publish(outMsg);
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "undistMono");

    ROS_INFO_STREAM("Init done");

    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    ros::Subscriber rightSub = n.subscribe("/camera/left/image", 1,
                                           rightCallback);

    rightPub = it.advertise("camera/left/undistorted", 1);

    string rightCalibFile;
    n.getParam("right_calibration", rightCalibFile);

    ROS_INFO_STREAM("Reading calibration file: " << rightCalibFile);

    FileStorage fs(rightCalibFile, FileStorage::READ);

    if (!fs.isOpened())
    {
        ROS_WARN_STREAM("Right calibration file could not be opened from: " <<
                        rightCalibFile);
    }
    else
    {
        ROS_INFO_STREAM("Valid calibration file");
        isCalibrated = true;
        string calibTime;
        fs["calibration_time"] >> calibTime;
        ROS_INFO_STREAM("Calibration was performed on: " << calibTime);
        fs["fisheye_model"] >> useFisheye;
        ROS_INFO_STREAM_COND(useFisheye, "Using fisheye model.");
        fs["camera_matrix"] >> camMat;
        fs["distortion_coefficients"] >> distCoeffs;
    }

    ros::spin();
}
