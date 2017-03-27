#ifndef STEREOPROCESSOR_HPP
#define STEREOPROCESSOR_HPP
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
// The following #if is to use the correct version of the cv_bridge
// Kinetic by default uses OpenCV3 so we don't need the custom build
#if ROS_VERSION_MINIMUM(1, 12, 0)  // Running Kinetic
    #include <cv_bridge/cv_bridge.h>
#else  // Running Indigo
    #include <cv3_bridge/cv_bridge.h>
#endif
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

        void init();

        void process(const Image& leftImage, const Image& rightImage,
                     const Mat &Q, Mat &disparityMat, Mat &_3dImageMat);

    private:
        NodeHandle *n;

        bool initialized;

        Mat toOpenCV(const Image& image);
};
#endif // STEREOPROCESSOR_HPP
