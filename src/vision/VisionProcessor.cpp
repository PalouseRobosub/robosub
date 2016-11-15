#include "VisionProcessor.hpp"

VisionProcessor::VisionProcessor()
{
    
}

VisionProcessor::~VisionProcessor()
{

}


cv::Mat VisionProcessor::process(sensor_msgs::Image& image)
{
    //Create a nodehandle for param fetching
    ros::NodeHandle n;
    
    //Convert image message to OpenCV Mat
    cv::Mat toProcess = toOpenCV(image);
    
    //std::cout << "Processed" << std::endl;

    //Convert to HSV color space
    cv::Mat hsv;
    cv::cvtColor(toProcess, hsv, cv::COLOR_BGR2HSV);

    //std::cout << "Converted to hsv" << std::endl;

    //Create the lowerbound of thresholding with defaults
    cv::Scalar lower_bound(0, 10, 10);
    
    //Get lowerbound params
    n.getParamCached("/vision/red/min/hue", lower_bound[0]);
    n.getParamCached("/vision/red/min/sat", lower_bound[1]);
    n.getParamCached("/vision/red/min/val", lower_bound[2]);

    //Create upperbound of thresholding with defaults
    cv::Scalar upper_bound(10,255,255);
 
    //Get upperbound params
    n.getParamCached("/vision/red/max/hue", upper_bound[0]);
    n.getParamCached("/vision/red/max/sat", upper_bound[1]);
    n.getParamCached("/vision/red/max/val", upper_bound[2]);   

    //std::cout << "Masking" << std::endl;
    cv::Mat mask;
    //Blur the image for a bit more smoothness
    cv::medianBlur(mask, mask, 3);
    //Mask the image within the threshold range
    cv::inRange(hsv, lower_bound, upper_bound, mask);

    //Add an open filter which reduces small noise.
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));

    //std::cout << "Masked" << std::endl;
    return mask;
}

///////Private functions///////

cv::Mat VisionProcessor::toOpenCV(sensor_msgs::Image& image)
{
    //std::cout << "Image size: " << sizeof(image.data) << std::endl;
    if (sizeof(image.data) == 0)
    {
        //std::cout << "Empty image" << std::endl;
        return cv::Mat();
    }

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //std::cout << "Copying..." << std::endl;
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        exit(1);
    }
    
    //std::cout << "Returning Mat" << std::endl;
    return cv_ptr->image;
}
