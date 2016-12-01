#include "MonoCalibrator.hpp"
#include "wfov_camera_msgs/WFOVImage.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv_bridge;

Settings s;

bool calibFinished = false;
vector<vector<Point2f>> imagePoints;
Mat cameraMatrix, distCoeffs;
Size imageSize;
int mode = DETECTION;
clock_t prevTimestamp = 0;
const Scalar RED(0,0,255), GREEN(0,255,0);
const char ESC_KEY = 27;

void imageCallback(const wfov_camera_msgs::WFOVImage::ConstPtr& msg)
{
    ROS_DEBUG_STREAM("Callback");
    Mat view = toCvShare(msg->image, msg, sensor_msgs::image_encodings::BGR8)->image;
    
    if (mode == CAPTURING && imagePoints.size() >= (size_t)s.nrFrames)
    {
        if (runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs, imagePoints))
            mode = CALIBRATED;
        else
            mode = DETECTION;
    }

    if (view.empty())
    {
        if (mode != CALIBRATED && !imagePoints.empty())
        {
            runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs, imagePoints);
        }
        calibFinished = true;
        ros::shutdown();
        return;
    }

    imageSize = view.size();
    if (s.flipVertical) flip(view, view, 0);

    vector<Point2f> pointBuf;

    bool found;

    int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;

    if (!s.useFisheye)
    {
        chessBoardFlags |= CALIB_CB_FAST_CHECK;
    }

    switch(s.calibrationPattern)
    {
        case Settings::CHESSBOARD:
            found = findChessboardCorners(view, s.boardSize, pointBuf, chessBoardFlags);
            break;
        case Settings::CIRCLES_GRID:
            found = findCirclesGrid(view, s.boardSize, pointBuf);
            break;
        case Settings::ASYMMETRIC_CIRCLES_GRID:
            found = findCirclesGrid(view, s.boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID);
            break;
        default:
            found = false;
            break;
    }

    if (found)
    {
        if (s.calibrationPattern == Settings::CHESSBOARD)
        {
            Mat viewGray;
            cvtColor(view, viewGray, COLOR_BGR2GRAY);
            cornerSubPix(viewGray, pointBuf, Size(11,11), Size(-1,-1), TermCriteria(TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1));
        }

        imagePoints.push_back(pointBuf);

        drawChessboardCorners(view, s.boardSize, Mat(pointBuf), found);
    }

    string outMsg = (mode == CALIBRATED) ? "Calibrated" : "Calibrating";

    int baseLine = 0;
    Size textSize = getTextSize(outMsg, 1, 1.0, 1, &baseLine);
    Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);

    if (mode == CAPTURING)
    {
        if (s.showUndistorted)
        {
            outMsg = format("%d/%d Undist", (int)imagePoints.size(), s.nrFrames);
        }
        else
        {
            outMsg = format("%d/%d", (int)imagePoints.size(), s.nrFrames);
        }
    }

    putText(view, outMsg, textOrigin, 1, 1, mode == CALIBRATED ? GREEN : RED);

    if (mode == CALIBRATED && s.showUndistorted)
    {
        Mat temp = view.clone();
        if (s.useFisheye)
        {
            fisheye::undistortImage(temp, view, cameraMatrix, distCoeffs);
        }
        else
        {
            undistort(temp, view, cameraMatrix, distCoeffs);
        }
    }

    imshow("Image", view);
    char key = (char)waitKey(s.delay);

    if (key == ESC_KEY)
    {
        calibFinished = true;
        ros::shutdown();
        return;
    }

    if (key == 'u' && mode == CALIBRATED)
    {
        s.showUndistorted = !s.showUndistorted;
    }

}

int main (int argc, char* argv[])
{
    ros::init(argc,argv, "camCalib");

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
        ROS_ERROR_STREAM("Could not open configuration file: " << inputSettingsFile);
        return -1;
    }
    ROS_DEBUG_STREAM("File opened");

    fs["Settings"] >> s;
    ROS_DEBUG_STREAM("Data stored");
    fs.release();

    if (!s.goodInput)
    {
        ROS_ERROR_STREAM("Invalid input detected!");
        return -1;
    }
    ROS_DEBUG_STREAM("Input validated");

    ros::Subscriber sub = n.subscribe("/camera/right/image", 1, imageCallback);
 
    ROS_INFO_STREAM("Settings Read and validated");

    ROS_DEBUG_STREAM("Spinning...");

    ros::spin();

}
