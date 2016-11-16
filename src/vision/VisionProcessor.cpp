#include "VisionProcessor.hpp"

//paramGroup defines the subdirectory under /vision to look for and use in params
VisionProcessor::VisionProcessor(string paramGroup)
{
    this->paramGroup = paramGroup;
}

VisionProcessor::~VisionProcessor()
{

}

//Processes the image using color filtering with parameters under given subgroup
Mat VisionProcessor::process(Image& image)
{
    //Create a nodehandle for param fetching
    ros::NodeHandle n;
    
    //Convert image message to OpenCV Mat
    Mat toProcess = toOpenCV(image);
    
    ROS_DEBUG_STREAM("Image now OpenCv Mat");

    //Convert to HSV color space
    Mat hsv;
    cvtColor(toProcess, hsv, cv::COLOR_BGR2HSV);

    ROS_DEBUG_STREAM("Converted image to hsv");

    //Create the lowerbound of thresholding with defaults
    Scalar lower_bound(0, 10, 10);
    
    //Get lowerbound params
    n.getParamCached("/vision/" + paramGroup + "/min/hue", lower_bound[0]);
    n.getParamCached("/vision/" + paramGroup + "/min/sat", lower_bound[1]);
    n.getParamCached("/vision/" + paramGroup + "/min/val", lower_bound[2]);

    //Create upperbound of thresholding with defaults
    Scalar upper_bound(10,255,255);
 
    //Get upperbound params
    n.getParamCached("/vision/" + paramGroup + "/max/hue", upper_bound[0]);
    n.getParamCached("/vision/" + paramGroup + "/max/sat", upper_bound[1]);
    n.getParamCached("/vision/" + paramGroup + "/max/val", upper_bound[2]);   

    ROS_DEBUG_STREAM("Performing Masking and/or other processing");
    Mat mask;
    //Blur the image for a bit more smoothness
    medianBlur(mask, mask, 3);
    //Mask the image within the threshold range
    inRange(hsv, lower_bound, upper_bound, mask);

    //Add an open filter which reduces small noise.
    morphologyEx(mask, mask, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(3,3)));

    ROS_DEBUG_STREAM("Image Masked and/or processed otherwise");
    return mask;
}

///////Private functions///////

//Converts a ros image_transport Image to an OpenCV Mat
Mat VisionProcessor::toOpenCV(Image& image)
{
    ROS_DEBUG_STREAM("Image size in bytes: " << sizeof(image.data));
    //Determine if the image is empty, may be deprecated.
    if (sizeof(image.data) == 0)
    {
        ROS_ERROR_STREAM("Empty image");
        //Return empty mat on empty image
        return Mat();
    }

    //Construct a pointer to contain the converted image
    CvImagePtr cv_ptr;
    try
    {
        ROS_DEBUG_STREAM("Copying/sharing image...");
        //Copy the image using cv_bridge, in future should change to "toCvShare"
        cv_ptr = toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        exit(1);
    }
    
    ROS_DEBUG_STREAM("Returning Mat");
    return cv_ptr->image;
}
