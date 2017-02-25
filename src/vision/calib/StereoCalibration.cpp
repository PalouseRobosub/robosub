#include "StereoCalibrator.hpp"
#include "wfov_camera_msgs/WFOVImage.h"
#include <cv3_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace cv_bridge;
using wfov_camera_msgs::WFOVImage;

using message_filters::Synchronizer;
using message_filters::sync_policies::ApproximateTime;

Settings settings;
StereoCalibrator *stereoCalib;

void callback(const WFOVImage::ConstPtr &rightImg,
              const WFOVImage::ConstPtr &leftImg)
{
    Mat rview = toCvShare(rightImg->image, rightImg,
                         sensor_msgs::image_encodings::BGR8)->image;

    Mat lview = toCvShare(leftImg->image, leftImg,
                         sensor_msgs::image_encodings::BGR8)->image;

    ros::Duration timeDelta;

    if (rightImg->header.stamp > leftImg->header.stamp)
    {
        timeDelta = rightImg->header.stamp - leftImg->header.stamp;
    }
    else
    {
        timeDelta = leftImg->header.stamp - rightImg->header.stamp;
    }

    if (timeDelta > ros::Duration(.1))
    {
        ROS_INFO_STREAM("Images time difference too great, skipping pair");
    }
    else
    {
        stereoCalib->submitImgs(rview, lview);
    }
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

    message_filters::Subscriber<WFOVImage> rsub(n, "/camera/right/image", 1);
    message_filters::Subscriber<WFOVImage> lsub(n, "/camera/left/image", 1);

    Synchronizer<ApproximateTime<WFOVImage, WFOVImage>> sync(
                         ApproximateTime<WFOVImage, WFOVImage>(5), rsub, lsub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ROS_INFO_STREAM("Output filename: " << settings.outputFileName);

    stereoCalib = new StereoCalibrator(settings.boardSize, settings.squareSize,
                                       settings.outputFileName,
                                       false, settings.showUndistorted);

    ROS_INFO_STREAM("Settings read and validated");

    ros::Rate r(10);
    int numGathered = 0;
    while (ros::ok() && stereoCalib->getNumValidPairs() < settings.nrFrames)
    {
        if(stereoCalib->getNumValidPairs() > numGathered)
        {
            ROS_INFO_STREAM("Completed " << numGathered << "/" <<
                            settings.nrFrames);
            numGathered = stereoCalib->getNumValidPairs();
        }
        ros::spinOnce();
        r.sleep();
    }

    stereoCalib->calibrate();

    ROS_INFO_STREAM("Calibration complete!");

    destroyAllWindows();
}
