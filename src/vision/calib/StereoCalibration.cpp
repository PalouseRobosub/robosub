#include "StereoCalibrator.hpp"
#include "wfov_camera_msgs/WFOVImage.h"
#include <cv3_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <vector>

using namespace cv_bridge;

Settings settings;
StereoCalibrator *stereoCalib;

void leftCallback(const wfov_camera_msgs::WFOVImage::ConstPtr& msg)
{
    Mat view = toCvShare(msg->image, msg,
                         sensor_msgs::image_encodings::BGR8)->image;

    stereoCalib->submitLeftImg(view);
}

void rightCallback(const wfov_camera_msgs::WFOVImage::ConstPtr& msg)
{
    Mat view = toCvShare(msg->image, msg,
                         sensor_msgs::image_encodings::BGR8)->image;

    stereoCalib->submitRightImg(view);
}

int main (int argc, char* argv[])
{
    ros::init(argc, argv, "stereo_calib");

    ros::NodeHandle n;

    //Read file
    if (argc <= 1)
    {
        ROS_FATAL("No input settings specified");
        return 1;
    }

    ROS_INFO_STREAM("Reading settings file");

    const string inputSettingsFile = argv[1];
    FileStorage fs(inputSettingsFile, FileStorage::READ);
    if (!fs.isOpened())
    {
        ROS_FATAL_STREAM("Could not open configuration file: " <<
                         inputSettingsFile);
        return -1;
    }
    ROS_DEBUG_STREAM("Settings file opened");

    fs["Settings"] >> settings;
    ROS_DEBUG_STREAM("Settings data fetched");
    fs.release();

    if (!settings.goodInput)
    {
        ROS_FATAL_STREAM("Invalid settings input detected!");
        return -1;
    }
    ROS_DEBUG_STREAM("Settings input validated");

    ros::Subscriber rsub = n.subscribe("/camera/right/image", 1, rightCallback);
    ros::Subscriber lsub = n.subscribe("/camera/left/image", 1, leftCallback);

    stereoCalib = new StereoCalibrator(settings.boardSize, settings.squareSize,
                                       settings.outputFileName,
                                       false, settings.showUndistorted);

    ROS_INFO_STREAM("Settings read and validated");

    ros::Rate r(10);
    while (ros::ok() && stereoCalib->getNumValidPairs() < settings.nrFrames)
    {
        ros::spinOnce();
        r.sleep();
    }

    stereoCalib->calibrate();

    ROS_INFO_STREAM("Calibration complete!");
}
