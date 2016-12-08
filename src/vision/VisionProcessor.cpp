#include "VisionProcessor.hpp"
#include <XmlRpcException.h>

//paramGroup defines the subdirectory under /vision to look for and use in params
VisionProcessor::VisionProcessor(string paramGroup)
{
    this->paramGroup = paramGroup;
    this->n = NodeHandle("~vision");
}

VisionProcessor::~VisionProcessor()
{

}

//Processes the image using color filtering with parameters under given subgroup
Mat VisionProcessor::process(Image& image)
{
    //Create a nodehandle for param fetching
    //ros::NodeHandle n("~vision");
    
    //Convert image message to OpenCV Mat
    Mat toProcess = toOpenCV(image);
    
    ROS_DEBUG_STREAM("Image now OpenCv Mat");

    //Convert to HSV color space
    Mat hsv;
    cvtColor(toProcess, hsv, cv::COLOR_BGR2HSV);

    ROS_DEBUG_STREAM("Converted image to hsv");

    //Create the lowerbound of thresholding with defaults
    vector<Scalar> lower_bounds;
    
    getLowerBoundParams(lower_bounds); 

    //Create upperbound of thresholding with defaults
    vector<Scalar> upper_bounds;
 
    getUpperBoundParams(upper_bounds);
    
    //Initialize blur iteration num with default
    int numBlurIters = 3;

    //Get blur param
    n.getParamCached("/vision/blur_iters", numBlurIters);

    //Initialize open filter size with default of 3x3
    int openSize[] = {3,3};

    //Initialize open filter iteration num with default
    int numOpenIters = 1;

    //Get open params
    n.getParamCached("/vision/open/width", openSize[0]);
    n.getParamCached("/vision/open/height", openSize[1]);
    n.getParamCached("/vision/open/iters", numOpenIters);

    //Initialize open filter size with default of 3x3
    int closeSize[] = {3,3};

    //Initialize open filter iteration num with default
    int numCloseIters = 1;

    //Get open params
    n.getParamCached("/vision/close/width", closeSize[0]);
    n.getParamCached("/vision/close/height", closeSize[1]);
    n.getParamCached("/vision/close/iters", numCloseIters);

    //Sizes must be odd for open filter
    if (openSize[0] % 2 != 1)
    {
        openSize[0]++;
    }

    if (openSize[1] % 2 != 1)
    {
        openSize[1]++;
    }
    
    if (closeSize[0] % 2 != 1)
    {
        closeSize[0]++;
    }

    if (closeSize[1] % 2 != 1)
    {
        closeSize[1]++;
    }
    
    ROS_DEBUG_STREAM("Performing Masking and/or other processing");

    //Blur the image for a bit more smoothness
    medianBlur(hsv, hsv, 3);
    
    Mat mask;
    //Mask the image within the threshold ranges
    for (int i = 0; i < lower_bounds.size() && i < upper_bounds.size(); i++)
    {
        ROS_DEBUG_STREAM("Masking color range " << i << " from " 
                            << lower_bounds[i] << " to " << upper_bounds[i]);
        Mat temp;
        inRange(hsv, lower_bounds[i], upper_bounds[i], temp);
        imshow("[" + std::to_string(i) + "] Found", temp);
        if (!mask.empty())
        {
            bitwise_or(mask, temp, mask);
        }
        else
        {
            temp.copyTo(mask);
        }

    }

    //Add a close filter to remove holes in objects
    morphologyEx(mask, mask, MORPH_CLOSE, getStructuringElement(MORPH_RECT, Size(closeSize[0], closeSize[1])), Point(-1,-1), numCloseIters);

    //Add an open filter which reduces small noise.
    morphologyEx(mask, mask, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(openSize[0], openSize[1])), Point(-1,-1), numOpenIters);

    ROS_DEBUG_STREAM("Image Masked and/or processed otherwise");
/*
    Mat red = Mat::zeros(rangeMasks[0].size(), CV_32F);
    ROS_DEBUG_STREAM("Red zeroed");
    red.setTo(Scalar(255,0,0));
    ROS_DEBUG_STREAM("Red set");
    Mat redOut = Mat::zeros(rangeMasks[0].size(), CV_32F);
    red.copyTo(redOut, rangeMasks[0]);

    ROS_DEBUG_STREAM("Copied to red");
    
    Mat green = Mat::zeros(rangeMasks[1].size(), CV_32F);
    green.setTo(Scalar(0,255,0));
    Mat greenOut = Mat::zeros(rangeMasks[1].size(), CV_32F);
    green.copyTo(greenOut, rangeMasks[1]);

    ROS_DEBUG_STREAM("Copied to green");

    vector<Mat> maskedColors;
    maskedColors.push_back(redOut);
    maskedColors.push_back(greenOut);

    Mat color;
    merge(maskedColors, color);

*/
    
    ROS_DEBUG_STREAM("Returning mask");
    return mask;
}

///////Private functions///////

void VisionProcessor::getScalarParamSet(string mapName,vector<Scalar> &scalars)
{
    
    string key;
    //Get lowerbound params
    if (n.searchParam(mapName, key))
    {
        XmlRpc::XmlRpcValue map;   
        n.getParamCached(key, map);
        ROS_DEBUG_STREAM("Found: " << map.size() << " sets in key " << key);
        
        for (int i = 0; i < map.size(); i++)
        {
            std::map<std::string, double> parameters;
            try 
            {
                for (auto it = map[i].begin(); it != map[i].end(); it++)
                {
                    double value = 0.0;
                    switch(it->second.getType())
                    {
                        case XmlRpc::XmlRpcValue::TypeDouble:
                            value = it->second;
                            break;
                        case XmlRpc::XmlRpcValue::TypeInt:
                            value = static_cast<double>(static_cast<int>(it->second));
                            break;
                        default:
                            ROS_ERROR_STREAM("Invalid parameter type for " << mapName << ": " << it->first);
                            return;
                            break;
                    }

                    parameters.emplace(it->first, value);
                }    
            }
            catch (XmlRpc::XmlRpcException e)
            {
                ROS_ERROR_STREAM("XmlRpcException code " << e.getCode() << ": " << e.getMessage());
                ros::shutdown();
            }
            
            Scalar s;
            s[0] = parameters["hue"];//map[i]["hue"];
            s[1] = parameters["sat"];
            s[2] = parameters["val"];
            ROS_DEBUG_STREAM("Scalar: " << s);
            scalars.push_back(s);
            
        }
        ROS_INFO_STREAM(mapName << " Params fetched");
    }
    else
    {
        ROS_ERROR_STREAM("Node does not contain " << mapName << " params");
    }
    ROS_DEBUG_STREAM("Size of scalars: " << scalars.size()); 
}

void VisionProcessor::getLowerBoundParams(vector<Scalar> &lower_bounds)
{
    getScalarParamSet("min", lower_bounds);
}
void VisionProcessor::getUpperBoundParams(vector<Scalar> &upper_bounds)
{
    getScalarParamSet("max", upper_bounds); 
}

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
