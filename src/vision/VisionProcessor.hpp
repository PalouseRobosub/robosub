#ifndef VISIONPROCESSOR_HPP
#define VISIONPROCESSOR_HPP
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
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <map>

#include "filters/FilterSet.hpp"

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

        void init();

        Mat process(const Image& image);

    private:
        NodeHandle *n;

        bool initialized;

        FilterSet filterSet;

        Mat toOpenCV(const Image& image);
};
#endif // VISIONPROCESSOR_HPP
