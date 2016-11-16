#ifndef VISION_PROCESSOR_H
#define VISION_PROCESSOR_H
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>

using std::string;
using namespace cv;
using cv_bridge::toCvCopy;
using cv_bridge::CvImagePtr;
using sensor_msgs::Image;

class VisionProcessor
{
    public:
        VisionProcessor(string paramGroup);
        ~VisionProcessor();
        
        Mat process(Image& image);

    private:
        string paramGroup;
    
        Mat toOpenCV(Image& image);
        
};
#endif
