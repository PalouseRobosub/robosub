#include "VisionProcessor.hpp"
#include <XmlRpcException.h>
#include <string>
#include <vector>
#include <map>

VisionProcessor::VisionProcessor()
    : initialized(false)
{
}

VisionProcessor::~VisionProcessor()
{
    delete n;
}

void VisionProcessor::init()
{
    //The NodeHandle is dynamically allocated here to prevent the constructor
    //  from creating it. This is so ros::init() can be called before
    //  the NodeHandle is constructed.
    this->n = new NodeHandle("~processing");
    this->initialized = true;
}

//Processes the image using color filtering with parameters under given subgroup
Mat VisionProcessor::process(const Image& image)
{
    if(!initialized)
    {
        ROS_FATAL_STREAM("Vision Processor process called before init.");
        ros::shutdown();
        return Mat();
    }
    
    XmlRpcValue params;
    string key;
    if (!n.searchParam("filters", key))
    {
        ROS_FATAL_STREAM("Could not locate filter params");
        ros::shutdown();
        return Mat();
    }
    if (!n.getParamCached(key, params))
    {
        ROS_FATAL_STREAM("Filter params not present!!");
        ros::shutdown();
        return Mat();
    }

    filterSet.setParams(params);
    
    //Convert image message to OpenCV Mat
    Mat toProcess = toOpenCV(image);

    ROS_DEBUG_STREAM("Image now OpenCv Mat");

    //Mask according to the filterSet
    Mat mask;
    
    filterSet.apply(toProcess, mask);

    return mask;
}

///////Private functions///////

// Fetches a list of parameters from the parameter server and represents
//   them as a vector of Scalars
void VisionProcessor::getScalarParamSet(string mapName,
                                        vector<Scalar> &scalars)
{
    string key;
    //Get params based upon the list name
    if (n->searchParam(mapName, key))
    {
        // Fetch param
        XmlRpc::XmlRpcValue map;
        n->getParamCached(key, map);
        ROS_DEBUG_STREAM("Found: " << map.size() << " sets in key " << key);

        // For every value in the list of values
        for (int i = 0; i < map.size(); i++)
        {
            std::map<std::string, double> parameters;
            try
            {
                // Iterate over every element within the map contained in this
                //   element in the overall list
                for (auto it = map[i].begin(); it != map[i].end(); it++)
                {
                    double value = 0.0;
                    //Double check the type is valid and cast if necessary
                    switch(it->second.getType())
                    {
                        case XmlRpc::XmlRpcValue::TypeDouble:
                            value = it->second;
                            break;
                        case XmlRpc::XmlRpcValue::TypeInt:
                            value = static_cast<double>(
                                                 static_cast<int>(it->second));
                            break;
                        default:
                            ROS_ERROR_STREAM("Invalid parameter type for " <<
                                             mapName << ": " << it->first);
                            return;
                            break;
                    }

                    // Add this value to the map of values containing hue,
                    //   sat, and val
                    parameters.emplace(it->first, value);
                }
            }
            catch (XmlRpc::XmlRpcException e)
            {
                ROS_ERROR_STREAM("XmlRpcException code " << e.getCode() <<
                                 ": " << e.getMessage());
                ros::shutdown();
            }

            Scalar s;
            s[0] = parameters["hue"];//map[i]["hue"];
            s[1] = parameters["sat"];
            s[2] = parameters["val"];
            ROS_DEBUG_STREAM("Scalar: " << s);
            scalars.push_back(s);
        }
        ROS_DEBUG_STREAM(mapName << " Params fetched");
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
Mat VisionProcessor::toOpenCV(const Image& image)
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
