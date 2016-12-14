#ifndef VISIONPROCESSOR_HPP
#define VISIONPROCESSOR_HPP
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
//#include <opencv-3.1.0-dev/opencv2/imgproc.hpp>
//#include <opencv-3.1.0-dev/opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <map>

using std::string;
using std::vector;
using std::map;
using ros::NodeHandle;
using namespace cv;
using cv_bridge::toCvCopy;
using cv_bridge::CvImagePtr;
using sensor_msgs::Image;

class VisionProcessor
{
    public:
        VisionProcessor();
        ~VisionProcessor();
        
        Mat process(const Image& image);

    private:
        NodeHandle n;

        void getScalarParamSet(string mapName, vector<Scalar> &scalars);
        void getLowerBoundParams(vector<Scalar> &lower_bounds);
        void getUpperBoundParams(vector<Scalar> &upper_bounds);

        Mat toOpenCV(const Image& image);
};
#endif // VISIONPROCESSOR_HPP
