#ifndef STEREOPROCESSOR_HPP
#define STEREOPROCESSOR_HPP
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv3_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <map>

#include "robosub/visionPos.h"
#include "robosub/visionPosArray.h"

using std::string;
using std::vector;
using std::map;
using ros::NodeHandle;
using namespace cv;
using cv_bridge::toCvCopy;
using cv_bridge::CvImagePtr;
using sensor_msgs::Image;

bool compareContourAreas(vector<Point> contour1, vector<Point> contour2);


class StereoProcessor
{
    public:
        StereoProcessor();
        ~StereoProcessor();

        void locateCentroids(robosub::visionPosArray &vpa);
        void updateLeft(const Image& image);
        void updateRight(const Image& image);

    private:
        NodeHandle n;

        Mat leftImg;
        Mat rightImg;

        void process(Mat &contourMat, Mat &disparityMat);
        void getScalarParamSet(string mapName, vector<Scalar> &scalars);
        void getLowerBoundParams(vector<Scalar> &lower_bounds);
        void getUpperBoundParams(vector<Scalar> &upper_bounds);

        Mat toOpenCV(const Image& image);
};
#endif // STEREOPROCESSOR_HPP
