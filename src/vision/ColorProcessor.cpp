#include "ColorProcessor.hpp"
#include <XmlRpcException.h>
#include <string>
#include <vector>
#include <map>

ColorProcessor::ColorProcessor() :
    initialized(false)
{
}

ColorProcessor::~ColorProcessor()
{
    delete n;
}

void ColorProcessor::init()
{
    if (!initialized)
    {
        //The NodeHandle is dynamically allocated here to prevent the
        //  constructor from creating it. This is so ros::init() can be called
        //  before the NodeHandle is constructed.
        this->n = new NodeHandle("~processing");
        this->initialized = true;
    }
}

//Processes the image using color filtering with parameters under given subgroup
Mat ColorProcessor::process(const Image& image)
{
    if(!initialized)
    {
        ROS_FATAL_STREAM("Vision Processor process called before init.");
        ros::shutdown();
        return Mat();
    }

    // Fetch filter params
    XmlRpcValue params;
    string key;
    if (!n->searchParam("filters", key))
    {
        ROS_FATAL_STREAM("Could not locate filter params for "
                         << ros::this_node::getName());
        ros::shutdown();
        return Mat();
    }
    if (!n->getParamCached(key, params))
    {
        ROS_FATAL_STREAM("Filter params not present for "
                         << ros::this_node::getName());
        ros::shutdown();
        return Mat();
    }

    // Fetch doImShow param
    bool doImShow = false;

    if (!n->getParamCached("doImShow", doImShow))
    {
        ROS_WARN("Could not get doImShow parameter, defaulting to false");
    }

    // Set doImShow within FilterSet
    filterSet.setImShow(doImShow);

    //Set params within FilterSet
    filterSet.setParams(params);

    //Convert image message to OpenCV Mat
    Mat toProcess = toOpenCV(image);

    ROS_DEBUG_STREAM("Image now OpenCv Mat");

    // Filter image and receive mask
    Mat mask;
    filterSet.apply(toProcess, mask);

    ROS_DEBUG_STREAM("Returning mask");

    // Return result
    return mask;
}

//Converts a ros image_transport Image to an OpenCV Mat
Mat ColorProcessor::toOpenCV(const Image& image)
{
    ROS_DEBUG_STREAM("Image size in bytes: " << sizeof(image.data));
    //Determine if the image is empty, may be deprecated.
    if (sizeof(image.data) == 0)
    {
        ROS_ERROR_STREAM("Conversion from Image to Mat failed: Empty image");
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
        ros::shutdown();
        return Mat();
    }

    ROS_DEBUG_STREAM("Returning Mat");
    return cv_ptr->image;
}
