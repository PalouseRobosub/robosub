#ifndef VISION_PROCESSOR_H
#define VISION_PROCESSOR_H
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


class VisionProcessor
{
    public:
        VisionProcessor(); //Construct with settings file?
        ~VisionProcessor();
        
        cv::Mat process(const sensor_msgs::Image& image);

    private:
        cv::Mat toOpenCV(const sensor_msgs::Image& image);
        
};
#endif
