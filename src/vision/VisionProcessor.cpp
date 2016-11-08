#include "VisionProcessor.hpp"

VisionProcessor::VisionProcessor()
{
    
}

VisionProcessor::~VisionProcessor()
{

}


cv::Mat VisionProcessor::process(const sensor_msgs::Image& image)
{
    cv::Mat toProcess = toOpenCV(image);

    cv::Mat hsv;
    cv::cvtColor(toProcess, hsv, cv::COLOR_BGR2HSV);

    cv::Scalar lower_bound(0, 10, 10);
    cv::Scalar upper_bound(10,255,255);

    cv::Mat mask;
    cv::inRange(hsv, lower_bound, upper_bound, mask);

    return mask;
}

///////Private functions///////

cv::Mat VisionProcessor::toOpenCV(const sensor_msgs::Image& image)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        exit(1);
    }

    return cv_ptr->image;
}
