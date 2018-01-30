#ifndef STEREOCALIBRATOR_HPP
#define STEREOCALIBRATOR_HPP

#include <ros/ros.h>
#include "CalibrationSettings.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <ctime>


using namespace cv;
using std::string;
using std::vector;
using std::pair;

class StereoCalibrator
{
    public:
        StereoCalibrator(Size boardSize, float squareSize,
                         string outputFilename, bool displayCorners = false,
                         bool useCalibrated = true, bool showRectified = true,
                         int cropRadius = -1);
        ~StereoCalibrator();

        void submitLeftImg(const Mat &leftImg);
        void submitRightImg(const Mat &rightImg);
        void submitImgs(const Mat &rightImg, const Mat &leftImg);

        void calibrate();

        int getNumValidPairs();

    private:
        Size boardSize;
        float squareSize;
        bool displayCorners;
        bool useCalibrated;
        bool showRectified;
        int cropRadius;

        vector<pair<Mat, Mat>> validPairs;

        vector<vector<Point2f>> imagePoints1;
        vector<vector<Point2f>> imagePoints2;
        vector<vector<Point3f>> objectPointsf;
        vector<vector<Point3d>> objectPoints;

        //Specifically needed for fisheye
        vector<vector<Point2d>> leftImgPoints;
        vector<vector<Point2d>> rightImgPoints;

        FileStorage outputFile;

        void saveIntrinsics(Mat camMat1, Mat distCoeffs1,
                            Mat camMat2, Mat distCoeffs2,
                            Size imageSize, int cropRadius);
        void saveExtrinsics(Mat R, Mat T, Mat R1, Mat R2,
                            Mat P1, Mat P2, Mat F, Mat E, Mat Q);
};
#endif //STEREOCALIBRATOR_HPP