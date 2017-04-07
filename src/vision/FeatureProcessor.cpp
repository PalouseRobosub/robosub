#include "FeatureProcessor.hpp"

FeatureProcessor::FeatureProcessor() :
    initialized(false)
{
}

FeatureProcessor::~FeatureProcessor()
{
    delete n;
    delete type;
}

void FeatureProcessor::init()
{
    if (!initialized)
    {
        //The NodeHandle is dynamically allocated here to prevent the constructor
        //  from creating it. This is so ros::init() can be called before
        //  the NodeHandle is constructed.
        this->n = new NodeHandle("~processing/features");
        this->initialized = true;
    }
}

void FeatureProcessor::process(const Mat &leftMask, const Mat &rightMask,
                               const Mat &disp, const Mat &_3dImg,
                               vector<visionPos> &messages)
{
    if (!initialized)
    {
        ROS_FATAL_STREAM("Feature Processor process called before init.");
        ros::shutdown();
        return;
    }

    if (!updateType())
    {
        return;
    }

    type->process(leftMask, rightMask, disp, _3dImg, messages);
}

void FeatureProcessor::process(const Mat &leftMask, const Mat &rightMask,
                               const Mat &bottomMask, const Mat &disp,
                               const Mat &_3dImg, vector<visionPos> &stereoMessages,
                               vector<visionPos> &bottomMessages)
{
    if (!initialized)
    {
        ROS_FATAL_STREAM("Feature Processor process called before init.");
        ros::shutdown();
        return;
    }

    if (!updateType())
    {
        return;
    }

    type->process(leftMask, rightMask, disp, _3dImg, stereoMessages);
    type->process(bottomMask, bottomMessages);
}

bool FeatureProcessor::updateType()
{
    string processType = "";
    if(!n->getParamCached("type", processType))
    {
        ROS_WARN_ONCE("Could not get feature processor type, defaulting to "
                      "CENTROID. (This prints only once)");
        processType = "CENTROID";
    }

    if (boost::iequals(processType, "CENTROID"))
    {
        type = new CentroidProcessor();
    }
    else
    {
        ROS_FATAL_STREAM("Invalid process type: " << processType << ". "
                         "Consider adding it to updateType in "
                         "FeatureProcessor.");
        ros::shutdown();
        return false;
    }

    XmlRpcValue params;
    if (!n->getParamCached("params", params))
    {
        ROS_FATAL("Could not get params for feature processor.");
        ros::shutdown();
        return false;
    }
    
    type->setParams(params);
    return true;
}
