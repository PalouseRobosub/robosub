#include <ros/ros.h>
#include "MonoCalibrator.hpp"
#include "wfov_camera_msgs/WFOVImage.h"
#if ROS_VERSION_MINIMUM(1, 12, 0)
    #include <cv_bridge/cv_bridge.h>
#else
    #include <cv3_bridge/cv_bridge.h>
#endif
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <vector>

using namespace cv_bridge;

Settings settings;

bool calibFinished = false;
vector<vector<Point2f>> imagePoints;
Mat cameraMatrix, distCoeffs;
Size imageSize;
CalibrationMode mode = CalibrationMode::CAPTURING;
clock_t prevTimestamp = 0;
const Scalar RED(0, 0, 255), GREEN(0, 255, 0);
const char ESC_KEY = 27;

void imageCallback(const wfov_camera_msgs::WFOVImage::ConstPtr& msg)
{
    ROS_DEBUG_STREAM("Callback, CalibrationMode: " << modeToString(mode));
    Mat view = toCvShare(msg->image, msg,
                         sensor_msgs::image_encodings::BGR8)->image;

    //When capturing and we have enough images
    if (mode == CalibrationMode::CAPTURING &&
        imagePoints.size() >= (size_t)settings.nrFrames)
    {
        //Try to calibrate
        ROS_DEBUG_STREAM("Calibrating and saving during capture");
        if (runCalibrationAndSave(settings, imageSize, cameraMatrix, distCoeffs,
                                  imagePoints))
        {
            //If successful, we have calibrated
            mode = CalibrationMode::CALIBRATED;
        }
        else
        {
            //Otherwise, go back to detecting.
            mode = CalibrationMode::DETECTION;
        }
    }

    if (view.empty())
    {
        ROS_DEBUG_STREAM("Empty view");
        //Nothing can be seen
        if (mode != CalibrationMode::CALIBRATED && !imagePoints.empty())
        {
            //If there is data and we haven't already, try calibrating
            runCalibrationAndSave(settings, imageSize, cameraMatrix, distCoeffs,
                                  imagePoints);
        }
        //There is no more data, we have finished
        calibFinished = true;
        ROS_DEBUG_STREAM("Exiting after empty view. CalibrationMode: " <<
                         modeToString(mode));
        ros::shutdown();
        return;
    }

    imageSize = view.size();
    if (settings.flipVertical)
    {
        flip(view, view, 0);
    }

    vector<Point2f> pointBuf;

    //Defines if we have found a board
    bool found;

    int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;

    if (!settings.useFisheye)
    {
        chessBoardFlags |= CALIB_CB_FAST_CHECK;
    }

    switch(settings.calibrationPattern)
    {
        case Settings::Pattern::CHESSBOARD:
            ROS_DEBUG_STREAM("Finding Chessboard Corners");
            found = findChessboardCorners(view, settings.boardSize, pointBuf,
                                          chessBoardFlags);
            break;
        case Settings::Pattern::CIRCLES_GRID:
            found = findCirclesGrid(view, settings.boardSize, pointBuf);
            break;
        case Settings::Pattern::ASYMMETRIC_CIRCLES_GRID:
            found = findCirclesGrid(view, settings.boardSize, pointBuf,
                                    CALIB_CB_ASYMMETRIC_GRID);
            break;
        default:
            ROS_ERROR_STREAM("Uknown Pattern used in settings");
            found = false;
            break;
    }

    if (found)
    {
        //There is a board in view and we have seen it!
        ROS_DEBUG_STREAM("Pattern found");
        if (settings.calibrationPattern == Settings::Pattern::CHESSBOARD)
        {
            /*
             * With a chessboard, there are internal corners being tracked,
             * cornerSubPix gives a more accurate location of these corners.
            */
            Mat viewGray;
            cvtColor(view, viewGray, COLOR_BGR2GRAY);
            cornerSubPix(viewGray, pointBuf, Size(11, 11), Size(-1, -1),
                         TermCriteria(TermCriteria::EPS+TermCriteria::COUNT,
                                      30, 0.1));
        }

        imagePoints.push_back(pointBuf);

        ROS_DEBUG_STREAM("Drawing corners...");
        //Visualize corners for user
        drawChessboardCorners(view, settings.boardSize, Mat(pointBuf), found);
    }
    else
    {
        ROS_DEBUG_STREAM("Pattern not found");
    }

    string outMsg = (mode == CalibrationMode::CALIBRATED) ?
                    "Calibrated" : "Calibrating";

    int baseLine = 0;
    Size textSize = getTextSize(outMsg, 1, 1.0, 1, &baseLine);
    Point textOrigin(view.cols - 2*textSize.width - 10, 10);

    //Add further information for user of how much data has been captured
    if (mode == CalibrationMode::CAPTURING)
    {
        ROS_DEBUG_STREAM("Capturing");
        if (settings.showUndistorted)
        {
            outMsg += format(" %d/%d Undist",
                            static_cast<int>(imagePoints.size()),
                            settings.nrFrames);
        }
        else
        {
            outMsg += format(" %d/%d", static_cast<int>(imagePoints.size()),
                            settings.nrFrames);
        }
    }

    if (mode == CalibrationMode::CALIBRATED && settings.showUndistorted)
    {
        ROS_DEBUG_STREAM("Undistorting image...");
        outMsg += " Undist."; //Tell user when showing undistorted
        Mat temp = view.clone();
        if (settings.useFisheye)
        {
            fisheye::undistortImage(temp, view, cameraMatrix, distCoeffs,
                                    Matx33d::eye());
        }
        else
        {
            undistort(temp, view, cameraMatrix, distCoeffs);
        }
    }

    ROS_DEBUG_STREAM("Adding text to image");

    //Show info to user
    putText(view, outMsg, textOrigin, 1, 1,
            mode == CalibrationMode::CALIBRATED ? GREEN : RED);
    imshow("Image", view);
    char key = static_cast<char>(waitKey(settings.delay));

    if (key == ESC_KEY)
    {
        ROS_DEBUG_STREAM("User requested stop");
        calibFinished = true;
        ros::shutdown();
        return;
    }

    if (key == 'u' && mode == CalibrationMode::CALIBRATED)
    {
        settings.showUndistorted = !settings.showUndistorted;
        ROS_INFO_STREAM("Now showing " <<
                        (settings.showUndistorted ?
                        "Undistorted" : "Distorted") <<
                        " image.");
    }
}

int main (int argc, char* argv[])
{
    ros::init(argc, argv, "camCalib");

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

    ros::Subscriber sub = n.subscribe("/camera/right/image", 1, imageCallback);

    ROS_INFO_STREAM("Settings read and validated");

    ROS_INFO_STREAM("To exit calibration and attempt to save, press ESC");
    ros::spin();

    ROS_INFO_STREAM("Calibration complete!");
}
