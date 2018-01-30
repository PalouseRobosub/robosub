#ifndef MONOCALIBRATOR_HPP
#define MONOCALIBRATOR_HPP

#include <ros/ros.h>
#include "CalibrationSettings.hpp"
#include <ctime>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <vector>

using namespace cv;
using namespace std;


enum class CalibrationMode
{
    DETECTION = 0,
    CAPTURING = 1,
    CALIBRATED = 2
};

std::string modeToString(CalibrationMode &mode);

bool runCalibration(Settings& s, Size& imageSize, Mat& cameraMatrix,
                    Mat& distCoeffs, vector<vector<Point2f>> imagePoints,
                    vector<Mat>& rvecs, vector<Mat>& tvecs,
                    vector<float>& reprojErrs, double& totalAvgErr);

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat& cameraMatrix,
                           Mat& distCoeffs,
                           vector<vector<Point2f>> imagePoints);

void saveCameraParams(Settings& s, Size& imageSize, Mat& cameraMatrix,
                      Mat& distCoeffs, const vector<Mat>& rvecs,
                      const vector<Mat>& tvecs, const vector<float>& reprojErrs,
                      const vector<vector<Point2f>>& imagePoints,
                      double totalAvgErr);

void calcBoardCornerPositions(Size boardSize, float squareSize,
                              vector<Point3f>& corners,
                              Settings::Pattern patternType);

double computeReprojectionErrors(const vector<vector<Point3f>>& objectPoints,
                                 const vector<vector<Point2f>>& imagePoints,
                                 const vector<Mat>& rvecs,
                                 const vector<Mat>& tvecs,
                                 const Mat& cameraMatrix,
                                 const Mat& distCoeffs,
                                 vector<float>& perViewErrors, bool fisheye);

#endif //MONOCALIBRATOR_HPP