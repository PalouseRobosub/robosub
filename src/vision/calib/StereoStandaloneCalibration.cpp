#include "StereoStandaloneCalibrator.hpp"
// The following #if is to use the correct version of the cv_bridge
// Kinetic by default uses OpenCV3 so we don't need the custom build
#if ROS_VERSION_MINIMUM(1, 12, 0)  // Running Kinetic
    #include <cv_bridge/cv_bridge.h>
#else // Running indigo
    #include <cv3_bridge/cv_bridge.h>
#endif
#include <opencv2/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace cv_bridge;
using sensor_msgs::Image;

using message_filters::Synchronizer;
using message_filters::sync_policies::ApproximateTime;

Settings settings;
StereoStandaloneCalibrator *stereoCalib;
Mat K1, K2, D1, D2;

void callback(const Image::ConstPtr &rightImg,
              const Image::ConstPtr &leftImg)
{
    ros::NodeHandle nh("~");

    Mat rview = toCvShare(rightImg,
                         sensor_msgs::image_encodings::BGR8)->image;

    Mat lview = toCvShare(leftImg,
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

    string leftCalibFile, rightCalibFile;
    fs["Settings"] >> settings;
    fs["LeftCalib"] >> leftCalibFile;
    fs["RightCalib"] >> rightCalibFile;

    ROS_DEBUG_STREAM("Settings data fetched");
    ROS_INFO_STREAM("Left calib: " << leftCalibFile);
    ROS_INFO_STREAM("Right calib: " << rightCalibFile);
    
    ROS_INFO_STREAM("Fetching left camera data...");

    FileStorage leftFs(leftCalibFile, FileStorage::READ);

    if (!leftFs.isOpened())
    {
        ROS_FATAL_STREAM("Invalid left camera calibration file: " << leftCalibFile);
        exit(1);
    }
    else
    {
        string calibTime;
	leftFs["calibration_time"] >> calibTime;
        ROS_INFO_STREAM("Left calibration was performed on: " << calibTime);
        leftFs["camera_matrix"] >> K1;
        leftFs["distortion_coefficients"] >> D1;
    }

    ROS_INFO_STREAM("Fetching right camera data...");
    FileStorage rightFs(rightCalibFile, FileStorage::READ);

    if (!rightFs.isOpened())
    {
        ROS_FATAL_STREAM("Invalid right camera calibration file: " << rightCalibFile);
        exit(1);
    }
    else
    {
        string calibTime;
	rightFs["calibration_time"] >> calibTime;
        ROS_INFO_STREAM("Right calibration was performed on: " << calibTime);
        rightFs["camera_matrix"] >> K2;
        rightFs["distortion_coefficients"] >> D2;
    }

    fs.release();
    leftFs.release();
    rightFs.release();

    if (!settings.goodInput)
    {
        ROS_FATAL_STREAM("Invalid settings input detected!");
        return -1;
    }
    ROS_DEBUG_STREAM("Settings input validated");

    message_filters::Subscriber<Image> rsub(n, "/camera/right/image_raw", 1);
    message_filters::Subscriber<Image> lsub(n, "/camera/left/image_raw", 1);

    Synchronizer<ApproximateTime<Image, Image>> sync(
                         ApproximateTime<Image, Image>(5), rsub, lsub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ROS_INFO_STREAM("Output filename: " << settings.outputFileName);

    stereoCalib = new StereoStandaloneCalibrator(K1, D1, K2, D2,
                                       settings.boardSize, settings.squareSize,
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
